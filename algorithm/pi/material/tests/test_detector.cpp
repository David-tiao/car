#include "pi_material/detector.hpp"

#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cmath>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <vector>

int main() {
    pi_material::DetectorConfig cfg;
    pi_material::ColorDetector detector(cfg);
    cv::Mat image(480, 640, CV_8UC3, cv::Scalar(220, 220, 220));
    // Synthetic BGR samples exercise ALL six color IDs, not just red.
    // These are NOT physical-camera/competition lighting results.
    struct Sample { int color; cv::Scalar bgr; };
    const std::vector<Sample> samples{
        {1, cv::Scalar(0, 0, 245)},    // red
        {2, cv::Scalar(0, 235, 245)},  // yellow
        {3, cv::Scalar(245, 85, 25)},  // blue
        {4, cv::Scalar(20, 200, 20)},  // green
        {5, cv::Scalar(15, 15, 15)},   // black
        {6, cv::Scalar(240, 210, 20)} // cyan
    };
    for (const auto& sample : samples) {
        image.setTo(cv::Scalar(220, 220, 220));
        cv::circle(image, {225, 205}, 29, sample.bgr, -1);
        const auto hit = detector.detect(image, sample.color);
        assert(hit.unique && !hit.ambiguous);
        assert(std::abs(hit.unique->u - 225) < 2);
        assert(std::abs(hit.unique->v - 205) < 2);
        for (int other = 1; other <= 6; ++other) {
            if (other != sample.color)
                assert(!detector.detect(image, other).unique);
        }
    }

    // Two red circles in a single frame must NOT be silently auto-selected.
    image.setTo(cv::Scalar(220, 220, 220));
    cv::circle(image, {225, 205}, 29, cv::Scalar(0, 0, 240), -1);
    cv::circle(image, {350, 150}, 28, cv::Scalar(0, 0, 230), -1);
    const auto ambiguous = detector.detect(image, 1);
    assert(!ambiguous.unique && ambiguous.ambiguous);

    cfg.roi = cv::Rect(150, 155, 160, 130);
    pi_material::ColorDetector roi_detector(cfg);
    assert(roi_detector.detect(image, 1).unique);

    // A dark background cannot automatically be labeled as black material.
    cv::Mat black_frame(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
    assert(!detector.detect(black_frame, 5).unique);
    std::cout << "synthetic HSV material segmentation tests OK\n";
}
