#include "pi_material/detector.hpp"

#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cmath>
#include <opencv2/imgproc.hpp>
#include <iostream>

int main() {
    pi_material::DetectorConfig cfg;
    pi_material::ColorDetector detector(cfg);
    cv::Mat image(480, 640, CV_8UC3, cv::Scalar(220, 220, 220));
    // Synthetic BGR colored circles; not evidence of real competition lighting.
    cv::circle(image, {225, 205}, 29, cv::Scalar(0, 0, 240), -1);
    cv::circle(image, {410, 255}, 30, cv::Scalar(230, 230, 230), -1);
    auto red = detector.detect(image, 1);
    assert(red.unique);
    assert(!red.ambiguous);
    assert(std::abs(red.unique->u - 225) < 2);
    assert(std::abs(red.unique->v - 205) < 2);
    assert(!detector.detect(image, 3).unique);

    cv::circle(image, {350, 150}, 28, cv::Scalar(0, 0, 230), -1);
    auto ambiguous = detector.detect(image, 1);
    assert(!ambiguous.unique && ambiguous.ambiguous);

    cfg.roi = cv::Rect(150, 155, 160, 130);
    pi_material::ColorDetector roi_detector(cfg);
    assert(roi_detector.detect(image, 1).unique);

    // A dark background cannot automatically be labeled as black material.
    cv::Mat black_frame(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
    assert(!detector.detect(black_frame, 5).unique);
    std::cout << "synthetic HSV material segmentation tests OK\n";
}
