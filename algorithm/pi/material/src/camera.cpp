#include "pi_material/camera.hpp"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <cerrno>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <stdexcept>
#include <utility>

#include <poll.h>
#include <sys/wait.h>
#include <unistd.h>

namespace pi_material {
namespace {
volatile std::sig_atomic_t shutdown_flag = 0;
}

void request_stop(int) { shutdown_flag = 1; }
bool stop_requested() { return shutdown_flag != 0; }

FrameSource::FrameSource(const CameraOptions& options) : options_(options) {
    if (options_.source == "images") {
        std::ifstream list(options_.images_file);
        if (!list) throw std::runtime_error("cannot read --image-list");
        std::string line;
        while (std::getline(list, line)) {
            if (line.empty() || line[0] == '#') continue;
            image_paths_.push_back(line);
        }
        if (image_paths_.empty())
            throw std::runtime_error("image list is empty");
        if (options_.repeat_each_image < 1 || options_.repeat_each_image > 100)
            throw std::invalid_argument("repeat-each must be 1..100");
    } else if (options_.source == "usb") {
        if (!usb_.open(options_.device, cv::CAP_V4L2))
            throw std::runtime_error("cannot open V4L2 camera: " + options_.device);
        usb_.set(cv::CAP_PROP_FRAME_WIDTH, options_.width);
        usb_.set(cv::CAP_PROP_FRAME_HEIGHT, options_.height);
        usb_.set(cv::CAP_PROP_FPS, options_.fps);
    } else if (options_.source == "pi") {
        if (options_.width % 2 || options_.height % 2)
            throw std::invalid_argument("CSI YUV420 dimensions must be even");
        raw_.resize(static_cast<std::size_t>(options_.width) * options_.height * 3 / 2);
        int fds[2];
        if (::pipe(fds) != 0) throw std::runtime_error("camera pipe failed");
        child_ = ::fork();
        if (child_ < 0) {
            ::close(fds[0]); ::close(fds[1]);
            throw std::runtime_error("camera fork failed");
        }
        if (child_ == 0) {
            ::close(fds[0]);
            if (::dup2(fds[1], STDOUT_FILENO) < 0) _exit(126);
            ::close(fds[1]);
            const auto w = std::to_string(options_.width);
            const auto h = std::to_string(options_.height);
            const auto fps = std::to_string(options_.fps);
            const auto camera = std::to_string(options_.camera_index);
            ::execlp("rpicam-vid", "rpicam-vid",
                     "--timeout", "0", "--nopreview", "--codec", "yuv420",
                     "--width", w.c_str(), "--height", h.c_str(),
                     "--framerate", fps.c_str(), "--camera", camera.c_str(),
                     "--output", "-", static_cast<char*>(nullptr));
            std::fprintf(stderr, "rpicam-vid not available: %s\n",
                         std::strerror(errno));
            _exit(127);
        }
        ::close(fds[1]);
        pipe_fd_ = fds[0];
    } else {
        throw std::invalid_argument("source must be pi, usb, or images");
    }
}

FrameSource::~FrameSource() {
    if (pipe_fd_ >= 0) ::close(pipe_fd_);
    if (child_ > 0) {
        ::kill(child_, SIGTERM);
        int status = 0;
        for (int retry = 0; retry < 20; ++retry) {
            const pid_t result = ::waitpid(child_, &status, WNOHANG);
            if (result == child_ || (result == -1 && errno == ECHILD)) return;
            ::usleep(50000);
        }
        ::kill(child_, SIGKILL);
        while (::waitpid(child_, &status, 0) < 0 && errno == EINTR) {}
    }
}

bool FrameSource::next(cv::Mat& frame) {
    if (stop_requested()) return false;

    if (options_.source == "images") {
        if (image_index_ >= image_paths_.size()) return false;
        if (image_repeated_ == 0) {
            last_image_ = cv::imread(image_paths_[image_index_], cv::IMREAD_COLOR);
            if (last_image_.empty())
                throw std::runtime_error("cannot read image: " + image_paths_[image_index_]);
        }
        frame = last_image_.clone();
        if (++image_repeated_ >= options_.repeat_each_image) {
            image_repeated_ = 0;
            ++image_index_;
        }
        return true;
    }

    if (options_.source == "usb") {
        if (usb_.read(frame) && !frame.empty()) {
            empty_usb_frames_ = 0;
            return true;
        }
        if (++empty_usb_frames_ > 20)
            throw std::runtime_error("USB camera stopped delivering frames");
        ::usleep(100000);
        frame.release();
        return true;  // caller must reset temporal filter
    }

    std::size_t bytes = 0;
    while (bytes < raw_.size() && !stop_requested()) {
        pollfd watcher{pipe_fd_, POLLIN, 0};
        int ready = ::poll(&watcher, 1, 8000);
        if (ready < 0 && errno == EINTR) continue;
        if (ready == 0) throw std::runtime_error("CSI camera stalled (>8 seconds)");
        if (ready < 0) throw std::runtime_error("CSI camera poll failed");
        const ssize_t n = ::read(pipe_fd_, raw_.data() + bytes, raw_.size() - bytes);
        if (n == 0) throw std::runtime_error("rpicam-vid stopped (camera unplugged?)");
        if (n < 0 && errno == EINTR) continue;
        if (n < 0) throw std::runtime_error("rpicam-vid read error");
        bytes += static_cast<std::size_t>(n);
    }
    if (stop_requested()) return false;
    cv::Mat yuv(options_.height * 3 / 2, options_.width, CV_8UC1, raw_.data());
    cv::cvtColor(yuv, frame, cv::COLOR_YUV2BGR_I420);
    return true;
}

}  // namespace pi_material
