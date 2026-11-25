#pragma once

#include "pi_material/core.hpp"

#include <opencv2/core.hpp>

#include <optional>
#include <string>
#include <vector>

namespace pi_material {

struct HsvRange {
    int color, h_lo, h_hi, s_lo, s_hi, v_lo, v_hi;
};

struct ColorHit {
    Spot spot;
    cv::Rect box;
    double circularity = 0;
    double fill = 0;
    double confidence = 0;  // YOLO class confidence; HSV has no calibrated score
};

struct Detection {
    std::vector<ColorHit> candidates;
    std::optional<Spot> unique;  // only populated for precisely one candidate
    std::optional<double> unique_confidence;  // YOLO only
    bool ambiguous = false;
};

struct DetectorConfig {
    double min_area_px = 160;
    double max_area_fraction = 0.12;
    double min_circularity = 0.64;
    std::optional<cv::Rect> roi;
    std::optional<cv::Vec3d> disk; // (cx, cy, radius) in this camera's pixels
    std::vector<HsvRange> ranges;
};

std::vector<HsvRange> default_hsv_ranges();
// CSV lines: color,h_lo,h_hi,s_lo,s_hi,v_lo,v_hi; # comments allowed.
// Full replacement requires a range for every color 1..6.
std::vector<HsvRange> load_hsv_ranges(const std::string& filename);

class ColorDetector {
public:
    explicit ColorDetector(DetectorConfig cfg);
    Detection detect(const cv::Mat& bgr, int wanted_color) const;
private:
    DetectorConfig cfg_;
};

}  // namespace pi_material
