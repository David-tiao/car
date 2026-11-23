#include "pi_material/yolo_detector.hpp"

#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {
void candidate(cv::Mat& t, bool channels_first, int index, int count,
               float x, float y, float w, float h,
               int class_index, float confidence) {
    float* data = t.ptr<float>();
    const auto put = [&](int channel, float value) {
        if (channels_first) data[channel * count + index] = value;
        else data[index * 10 + channel] = value;
    };
    put(0, x); put(1, y); put(2, w); put(3, h);
    for (int cls = 0; cls < 6; ++cls)
        put(cls + 4, cls == class_index ? confidence : 0.01f);
}
bool close(double a, double b, double eps = 2) {
    return std::abs(a-b) <= eps;
}
}

int main() {
    using namespace pi_material;
    const YoloConfig cfg{};
    const DetectorConfig spatial{};
    const cv::Rect full(0, 0, 640, 480);
    const auto lb = compute_letterbox({640, 480}, full, 640);
    assert(lb.resized == cv::Size(640, 480));
    assert(lb.pad_left == 0 && lb.pad_top == 80);

    // Ultralytics raw [1,4+6,N], no embedded NMS, BGR camera 640x480.
    int shape[3] = {1, 10, 4};
    cv::Mat output(3, shape, CV_32F, cv::Scalar(0));
    candidate(output, true, 0, 4, 220, 260, 60, 60, 0, 0.95f);  // red image (220,180)
    candidate(output, true, 1, 4, 221, 260, 60, 60, 0, 0.91f);  // same box, suppress
    candidate(output, true, 2, 4, 380, 280, 50, 50, 2, 0.97f);  // blue: not red
    candidate(output, true, 3, 4, 450, 380, 60, 60, 0, 0.20f);  // red, below conf

    auto red = decode_yolo_raw(output, lb, 1, cfg, spatial);
    assert(red.unique && !red.ambiguous);
    assert(red.candidates.size() == 1);
    assert(close(red.unique->u, 220) && close(red.unique->v, 180));
    assert(red.unique_confidence && close(*red.unique_confidence, 0.95, 0.01));
    auto blue = decode_yolo_raw(output, lb, 3, cfg, spatial);
    assert(blue.unique && close(blue.unique->u, 380) &&
           close(blue.unique->v, 200));
    assert(!decode_yolo_raw(output, lb, 5, cfg, spatial).unique);

    // The alternative [1,N,10] layout must produce the same geometry.
    int transposed_shape[3] = {1, 4, 10};
    cv::Mat transposed(3, transposed_shape, CV_32F, cv::Scalar(0));
    candidate(transposed, false, 0, 4, 220, 260, 60, 60, 0, 0.95f);
    candidate(transposed, false, 1, 4, 221, 260, 60, 60, 0, 0.91f);
    candidate(transposed, false, 2, 4, 380, 280, 50, 50, 2, 0.97f);
    candidate(transposed, false, 3, 4, 450, 380, 60, 60, 0, 0.20f);
    const auto t = decode_yolo_raw(transposed, lb, 1, cfg, spatial);
    assert(t.unique && close(t.unique->u, 220));

    // Different same-color objects, even after NMS -> ambiguity.
    candidate(output, true, 1, 4, 420, 250, 50, 50, 0, 0.91f);
    const auto two = decode_yolo_raw(output, lb, 1, cfg, spatial);
    assert(two.ambiguous && !two.unique && two.candidates.size() == 2);

    // Crop ROI [100,50,320x240] -> 640 square, pad top=80.
    const auto roi_lb = compute_letterbox({640, 480}, {100, 50, 320, 240}, 640);
    assert(roi_lb.pad_top == 80 && close(roi_lb.scale_x, 2, 1e-5));
    candidate(output, true, 0, 4, 240, 280, 60, 60, 0, 0.95f);
    candidate(output, true, 1, 4, 620, 620, 60, 60, 0, 0.01f);
    const auto cropped = decode_yolo_raw(output, roi_lb, 1, cfg, spatial);
    assert(cropped.unique);
    assert(close(cropped.unique->u, 220));
    assert(close(cropped.unique->v, 150));

    // With a calibrated turntable orbit gate, off-disk boxes are rejected.
    DetectorConfig gated;
    gated.disk = cv::Vec3d(200, 150, 20);  // center ~20 px -> at rim? require 8..19 px
    assert(!decode_yolo_raw(output, roi_lb, 1, cfg, gated).unique);

    // Wrong class count and end-to-end NMS tensors must fail closed.
    int coco_shape[3] = {1, 84, 8400};
    cv::Mat coco(3, coco_shape, CV_32F, cv::Scalar(0));
    bool wrong_layout = false;
    try { decode_yolo_raw(coco, lb, 1, cfg, spatial); }
    catch (const std::invalid_argument&) { wrong_layout = true; }
    assert(wrong_layout);
    int nms_shape[3] = {1, 300, 6};
    cv::Mat baked_nms(3, nms_shape, CV_32F, cv::Scalar(0));
    wrong_layout = false;
    try { decode_yolo_raw(baked_nms, lb, 1, cfg, spatial); }
    catch (const std::invalid_argument&) { wrong_layout = true; }
    assert(wrong_layout);
    std::cout << "YOLO raw output/letterbox/NMS/classes/gates tests passed\n";
}
