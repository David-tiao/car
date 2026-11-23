#include "pi_material/yolo_detector.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <utility>
#include <vector>

namespace pi_material {
namespace {

void check_config(const YoloConfig& y) {
    if (y.input_size < 160 || y.input_size > 1280 || y.input_size % 32 != 0 ||
        !(y.confidence > 0 && y.confidence <= 1) ||
        !(y.nms_iou > 0 && y.nms_iou < 1) ||
        y.max_candidates_per_class < 1 || y.max_candidates_per_class > 5000)
        throw std::invalid_argument("bad YOLO size/confidence/NMS settings");
}

struct RawAccessor {
    const float* data;
    int count;
    bool channel_first;
    float get(int anchor, int column) const {
        return channel_first ? data[column * count + anchor]
                             : data[anchor * 10 + column];
    }
};

RawAccessor parse_layout(const cv::Mat& m) {
    if (m.type() != CV_32FC1 || !m.isContinuous())
        throw std::invalid_argument("YOLO output must be contiguous CV_32F");
    if (m.dims == 3) {
        if (m.size[0] != 1)
            throw std::invalid_argument("YOLO ONNX batch must be 1");
        if (m.size[1] == 10 && m.size[2] != 10)
            return {m.ptr<float>(), m.size[2], true};
        if (m.size[2] == 10 && m.size[1] != 10)
            return {m.ptr<float>(), m.size[1], false};
    } else if (m.dims == 2 && m.size[1] == 10) {
        return {m.ptr<float>(), m.size[0], false};
    }
    throw std::invalid_argument(
        "unsupported YOLO output shape: expected [1,10,N], [1,N,10] or [N,10]; "
        "six-class raw ONNX only, nms=False, no objectness");
}

cv::Rect unproject(float cx, float cy, float w, float h,
                   const LetterboxInfo& box) {
    if (!std::isfinite(cx) || !std::isfinite(cy) ||
        !std::isfinite(w) || !std::isfinite(h) ||
        w <= 0 || h <= 0) return {};
    const double unscaled_x = (cx - box.pad_left) / box.scale_x;
    const double unscaled_y = (cy - box.pad_top) / box.scale_y;
    // Ignore predictions whose center falls in padding, outside the ROI.
    if (unscaled_x < 0 || unscaled_y < 0 ||
        unscaled_x >= box.roi.width || unscaled_y >= box.roi.height)
        return {};

    const double left = (cx - w * 0.5 - box.pad_left) / box.scale_x + box.roi.x;
    const double top = (cy - h * 0.5 - box.pad_top) / box.scale_y + box.roi.y;
    const double right = (cx + w * 0.5 - box.pad_left) / box.scale_x + box.roi.x;
    const double bottom = (cy + h * 0.5 - box.pad_top) / box.scale_y + box.roi.y;
    // Saturate outrageous malformed predictions before converting to int.
    if (!std::isfinite(left) || !std::isfinite(right) ||
        !std::isfinite(top) || !std::isfinite(bottom) ||
        std::max({std::abs(left), std::abs(right),
                  std::abs(top), std::abs(bottom)}) > 1e7)
        return {};
    const auto clamp_x = [&](double x) {
        return static_cast<int>(std::clamp(std::lround(x),
            static_cast<long>(box.roi.x),
            static_cast<long>(box.roi.x + box.roi.width)));
    };
    const auto clamp_y = [&](double y) {
        return static_cast<int>(std::clamp(std::lround(y),
            static_cast<long>(box.roi.y),
            static_cast<long>(box.roi.y + box.roi.height)));
    };
    const int x0 = clamp_x(left), x1 = clamp_x(right);
    const int y0 = clamp_y(top), y1 = clamp_y(bottom);
    if (x1 <= x0 || y1 <= y0) return {};
    return {x0, y0, x1 - x0, y1 - y0};
}

}  // namespace

LetterboxInfo compute_letterbox(cv::Size frame_size, cv::Rect roi,
                                int input_size) {
    if (frame_size.width <= 0 || frame_size.height <= 0 ||
        input_size <= 0 || roi.width < 20 || roi.height < 20 ||
        (roi & cv::Rect(0, 0, frame_size.width, frame_size.height)) != roi)
        throw std::invalid_argument("invalid letterbox frame/ROI");
    const double gain = std::min(
        static_cast<double>(input_size) / roi.width,
        static_cast<double>(input_size) / roi.height);
    const int rw = std::min(input_size,
        std::max(1, static_cast<int>(std::lround(roi.width * gain))));
    const int rh = std::min(input_size,
        std::max(1, static_cast<int>(std::lround(roi.height * gain))));
    return {roi, {rw, rh}, (input_size - rw) / 2,
            (input_size - rh) / 2, input_size,
            static_cast<double>(rw) / roi.width,
            static_cast<double>(rh) / roi.height};
}

Detection decode_yolo_raw(const cv::Mat& output, const LetterboxInfo& info,
                          int wanted_color, const YoloConfig& yolo,
                          const DetectorConfig& spatial) {
    check_config(yolo);
    if (wanted_color < 1 || wanted_color > 6)
        throw std::invalid_argument("YOLO class must map to material color 1..6");
    if (info.input_size != yolo.input_size)
        throw std::invalid_argument("letterbox and model sizes disagree");
    if (!(spatial.max_area_fraction > 0 && spatial.max_area_fraction <= 1))
        throw std::invalid_argument("bad max_area_fraction");
    const auto accessor = parse_layout(output);
    if (accessor.count <= 0 || accessor.count > 100000)
        throw std::invalid_argument("unsupported YOLO prediction count");

    struct Proposal { cv::Rect rect; float score; };
    std::vector<Proposal> proposals;
    proposals.reserve(256);
    const int wanted_index = wanted_color - 1;
    for (int row = 0; row < accessor.count; ++row) {
        float best_score = -1.f;
        int best_index = -1;
        bool score_bad = false;
        for (int cls = 0; cls < 6; ++cls) {
            const float s = accessor.get(row, cls + 4);
            if (!std::isfinite(s) || s < 0 || s > 1) {
                score_bad = true;
                break;
            }
            if (s > best_score) {
                best_score = s;
                best_index = cls;
            }
        }
        if (score_bad || best_index != wanted_index ||
            best_score < yolo.confidence) continue;
        const cv::Rect rect = unproject(
            accessor.get(row, 0), accessor.get(row, 1),
            accessor.get(row, 2), accessor.get(row, 3), info);
        if (rect.width < 10 || rect.height < 10 ||
            rect.area() > info.roi.area() * spatial.max_area_fraction)
            continue;
        const auto u = rect.x + rect.width * 0.5;
        const auto v = rect.y + rect.height * 0.5;
        if (spatial.disk) {
            const auto& disk = *spatial.disk;
            if (!std::isfinite(disk[0]) || !std::isfinite(disk[1]) ||
                !std::isfinite(disk[2]) || disk[2] <= 0)
                throw std::invalid_argument("invalid disk orbit calibration");
            const double distance = std::hypot(u - disk[0], v - disk[1]);
            if (distance < disk[2] * 0.39 ||
                distance > disk[2] * 0.96) continue;
        }
        proposals.push_back({rect, best_score});
    }

    std::sort(proposals.begin(), proposals.end(),
              [](const Proposal& a, const Proposal& b) {
                  return a.score > b.score;
              });
    if (proposals.size() > static_cast<std::size_t>(yolo.max_candidates_per_class))
        proposals.resize(yolo.max_candidates_per_class);
    std::vector<cv::Rect> boxes;
    std::vector<float> scores;
    for (const auto& p : proposals) {
        boxes.push_back(p.rect);
        scores.push_back(p.score);
    }
    std::vector<int> kept;
    if (!boxes.empty())
        cv::dnn::NMSBoxes(boxes, scores, yolo.confidence,
                          yolo.nms_iou, kept);

    Detection detection;
    for (const int id : kept) {
        const auto& prop = proposals.at(static_cast<std::size_t>(id));
        const Spot spot{
            prop.rect.x + prop.rect.width / 2.0,
            prop.rect.y + prop.rect.height / 2.0,
            (prop.rect.width + prop.rect.height) * 0.5,
            static_cast<double>(prop.rect.area())
        };
        ColorHit hit{};
        hit.spot = spot;
        hit.box = prop.rect;
        hit.confidence = prop.score;
        detection.candidates.push_back(hit);
    }
    detection.ambiguous = detection.candidates.size() > 1;
    if (detection.candidates.size() == 1) {
        detection.unique = detection.candidates.front().spot;
        detection.unique_confidence = detection.candidates.front().confidence;
    }
    return detection;
}

YoloDetector::YoloDetector(YoloConfig yolo, DetectorConfig spatial)
    : cfg_(std::move(yolo)), spatial_(std::move(spatial)) {
    check_config(cfg_);
    if (cfg_.onnx_path.empty())
        throw std::invalid_argument("YOLO --model must specify a six-class .onnx");
    try {
        net_ = cv::dnn::readNetFromONNX(cfg_.onnx_path);
        if (net_.empty())
            throw std::runtime_error("ONNX network is empty");
        net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
        net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    } catch (const cv::Exception& ex) {
        throw std::runtime_error(
            std::string("OpenCV DNN cannot load YOLO ONNX: ") + ex.what() +
            " (check OpenCV version, ONNX opset, six-class model, nms=False)");
    }
}

Detection YoloDetector::detect(const cv::Mat& bgr, int wanted_color) {
    if (bgr.empty() || bgr.type() != CV_8UC3)
        throw std::invalid_argument("YOLO input must be 8-bit BGR");
    const cv::Rect full(0, 0, bgr.cols, bgr.rows);
    const cv::Rect roi = spatial_.roi ? *spatial_.roi : full;
    const LetterboxInfo info = compute_letterbox(bgr.size(), roi, cfg_.input_size);
    cv::Mat resized, square;
    cv::resize(bgr(roi), resized, info.resized, 0, 0, cv::INTER_LINEAR);
    const int right = info.input_size - info.pad_left - info.resized.width;
    const int bottom = info.input_size - info.pad_top - info.resized.height;
    cv::copyMakeBorder(resized, square,
                       info.pad_top, bottom, info.pad_left, right,
                       cv::BORDER_CONSTANT, cv::Scalar(114, 114, 114));
    cv::Mat blob = cv::dnn::blobFromImage(square, 1.0 / 255.0,
                  {cfg_.input_size, cfg_.input_size},
                  cv::Scalar(0, 0, 0), true, false, CV_32F);
    try {
        net_.setInput(blob);
        const auto output = net_.forward();
        return decode_yolo_raw(output, info, wanted_color, cfg_, spatial_);
    } catch (const cv::Exception& ex) {
        throw std::runtime_error(
            std::string("YOLO ONNX inference failed: ") + ex.what() +
            " (wrong graph/opset/shape? See material/yolo/README.md)");
    }
}

}  // namespace pi_material
