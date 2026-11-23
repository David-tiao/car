#pragma once

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include <sys/types.h>

namespace pi_material {

struct CameraOptions {
    std::string source = "pi";  // pi, usb, images
    std::string device = "/dev/video0";
    std::string images_file;
    int camera_index = 0;
    int width = 640;
    int height = 480;
    int fps = 15;
    int repeat_each_image = 3;  // OFFLINE ONLY; artificial repetitions
};

class FrameSource {
public:
    explicit FrameSource(const CameraOptions& options);
    FrameSource(const FrameSource&) = delete;
    FrameSource& operator=(const FrameSource&) = delete;
    ~FrameSource();

    // Returns false when image list is exhausted or shutdown requested.
    // Throws on invalid image or unexpected camera loss.
    bool next(cv::Mat& frame);
    bool offline() const { return options_.source == "images"; }

private:
    CameraOptions options_;
    cv::VideoCapture usb_;
    int pipe_fd_ = -1;
    pid_t child_ = -1;
    std::vector<unsigned char> raw_;
    std::vector<std::string> image_paths_;
    std::size_t image_index_ = 0;
    int image_repeated_ = 0;
    cv::Mat last_image_;
    int empty_usb_frames_ = 0;
};

void request_stop(int);
bool stop_requested();

}  // namespace pi_material
