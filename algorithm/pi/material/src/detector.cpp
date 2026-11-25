#include "pi_material/detector.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

namespace pi_material {
namespace {
bool range_ok(const HsvRange& r) {
    return r.color >= 1 && r.color <= 6 &&
           0 <= r.h_lo && r.h_lo <= r.h_hi && r.h_hi <= 179 &&
           0 <= r.s_lo && r.s_lo <= r.s_hi && r.s_hi <= 255 &&
           0 <= r.v_lo && r.v_lo <= r.v_hi && r.v_hi <= 255;
}
}  // namespace

std::vector<HsvRange> default_hsv_ranges() {
    // Bench-only HSV starting values for Raspberry Pi BGR camera imagery.
    // OpenMV uses LAB, so these are NOT transferable calibration values.
    return {
        {1, 0, 12, 80, 255, 55, 255},      // red (wrap low)
        {1, 170, 179, 80, 255, 55, 255},   // red (wrap high)
        {2, 18, 37, 70, 255, 70, 255},     // yellow
        {3, 104, 135, 70, 255, 45, 255},   // blue
        {4, 38, 82, 65, 255, 40, 255},     // green
        {5, 0, 179, 0, 255, 0, 73},        // black: require ROI / geometry
        {6, 83, 103, 45, 255, 70, 255}    // light blue / cyan
    };
}

std::vector<HsvRange> load_hsv_ranges(const std::string& filename) {
    std::ifstream input(filename);
    if (!input) throw std::runtime_error("cannot open HSV file: " + filename);
    std::vector<HsvRange> ranges;
    std::array<bool, 7> found{};
    std::string line;
    int line_no = 0;
    while (std::getline(input, line)) {
        ++line_no;
        auto hash = line.find('#');
        if (hash != std::string::npos) line.erase(hash);
        if (line.find_first_not_of(" \t\r\n") == std::string::npos) continue;
        std::array<int, 7> n{};
        std::istringstream in(line);
        std::string token;
        for (int i = 0; i < 7; ++i) {
            if (!std::getline(in, token, ','))
                throw std::runtime_error("bad HSV CSV line " + std::to_string(line_no));
            std::size_t pos = 0;
            n[i] = std::stoi(token, &pos);
            if (token.find_first_not_of(" \t\r\n", pos) != std::string::npos)
                throw std::runtime_error("bad HSV integer at line " +
                                         std::to_string(line_no));
        }
        if (std::getline(in, token, ',') &&
            token.find_first_not_of(" \t\r\n") != std::string::npos)
            throw std::runtime_error("extra CSV field on line " + std::to_string(line_no));
        HsvRange r{n[0], n[1], n[2], n[3], n[4], n[5], n[6]};
        if (!range_ok(r)) throw std::runtime_error("out-of-range HSV values at line " +
                                                    std::to_string(line_no));
        ranges.push_back(r);
        found[r.color] = true;
    }
    for (int color = 1; color <= 6; ++color) {
        if (!found[color])
            throw std::runtime_error("missing HSV threshold for color " +
                                     std::to_string(color));
    }
    return ranges;
}

ColorDetector::ColorDetector(DetectorConfig cfg) : cfg_(std::move(cfg)) {
    if (cfg_.ranges.empty()) cfg_.ranges = default_hsv_ranges();
    for (const auto& range : cfg_.ranges)
        if (!range_ok(range)) throw std::invalid_argument("invalid HSV range");
    if (!std::isfinite(cfg_.min_area_px) || cfg_.min_area_px <= 0 ||
        !std::isfinite(cfg_.max_area_fraction) ||
        cfg_.max_area_fraction <= 0 || cfg_.max_area_fraction > 1 ||
        !std::isfinite(cfg_.min_circularity) ||
        cfg_.min_circularity <= 0 || cfg_.min_circularity > 1)
        throw std::invalid_argument("invalid material contour filter");
}

Detection ColorDetector::detect(const cv::Mat& bgr, int wanted_color) const {
    if (wanted_color < 1 || wanted_color > 6)
        throw std::invalid_argument("wanted_color outside 1..6");
    if (bgr.empty() || bgr.type() != CV_8UC3)
        throw std::invalid_argument("camera frame must be CV_8UC3 BGR");
    const cv::Rect bounds(0, 0, bgr.cols, bgr.rows);
    const cv::Rect area = cfg_.roi ? (*cfg_.roi & bounds) : bounds;
    if (area.width < 20 || area.height < 20)
        throw std::invalid_argument("material ROI does not intersect image");

    cv::Mat hsv;
    cv::cvtColor(bgr(area), hsv, cv::COLOR_BGR2HSV);
    cv::Mat binary = cv::Mat::zeros(hsv.size(), CV_8UC1);
    for (const auto& r : cfg_.ranges) {
        if (r.color != wanted_color) continue;
        cv::Mat segment;
        cv::inRange(hsv, cv::Scalar(r.h_lo, r.s_lo, r.v_lo),
                         cv::Scalar(r.h_hi, r.s_hi, r.v_hi), segment);
        cv::bitwise_or(binary, segment, binary);
    }
    // Remove isolated color noise and close small holes from reflections.
    const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, {3, 3});
    cv::morphologyEx(binary, binary, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(binary, binary, cv::MORPH_CLOSE, kernel);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    Detection out;
    for (const auto& contour : contours) {
        const double area_px = cv::contourArea(contour);
        if (area_px < cfg_.min_area_px ||
            area_px > cfg_.max_area_fraction * area.area()) continue;
        const double perimeter = cv::arcLength(contour, true);
        if (perimeter <= 0) continue;
        const double roundness = 4.0 * CV_PI * area_px / (perimeter * perimeter);
        if (roundness < cfg_.min_circularity) continue;
        const cv::Rect box = cv::boundingRect(contour);
        if (box.width < 10 || box.height < 10) continue;
        const double aspect = static_cast<double>(std::min(box.width, box.height)) /
                              std::max(box.width, box.height);
        if (aspect < 0.50) continue;
        const double fill = area_px / box.area();
        if (fill < 0.40) continue;
        const cv::Moments m = cv::moments(contour);
        if (m.m00 <= 0) continue;
        const double center_u = m.m10 / m.m00 + area.x;
        const double center_v = m.m01 / m.m00 + area.y;
        if (cfg_.disk) {
            const auto& disk = *cfg_.disk;
            if (!(disk[2] > 0) ||
                !std::isfinite(disk[0]) || !std::isfinite(disk[1]) ||
                !std::isfinite(disk[2]))
                throw std::invalid_argument("invalid calibrated disk geometry");
            const double r = std::hypot(center_u - disk[0], center_v - disk[1]);
            if (r < disk[2] * 0.39 || r > disk[2] * 0.96) continue;
        }
        cv::Point2f center;
        float outer_radius = 0;
        cv::minEnclosingCircle(contour, center, outer_radius);
        if (!(outer_radius > 0)) continue;
        const Spot spot{center_u, center_v, 2.0 * outer_radius, area_px};
        out.candidates.push_back({spot, box + cv::Point(area.x, area.y),
                                  roundness, fill});
    }

    std::sort(out.candidates.begin(), out.candidates.end(),
              [](const ColorHit& a, const ColorHit& b) {
                  return a.spot.area_px > b.spot.area_px;
              });
    // Never choose a "biggest" one when two same-color materials are visible.
    out.ambiguous = out.candidates.size() > 1;
    if (out.candidates.size() == 1) out.unique = out.candidates[0].spot;
    return out;
}

}  // namespace pi_material
