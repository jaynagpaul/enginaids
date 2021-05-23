#include <time.h>

#include <iostream>
#include <librealsense2/rs.hpp>  // Include RealSense Cross Platform API
#include <string>
#include <tuple>
#include <vector>

// pixel dimensions of box for fragmentation algorithm
const int BOX_HEIGHT = 70;
const int BOX_WIDTH = 100;

const int AVERAGE_FRAME_LENGTH = 15;

using Point = std::tuple<int, int>;

float vector_average(std::vector<float>& v) {
    float sum = 0;
    for (float f : v) {
        sum += f;
    }
    return sum / v.size();
}

void beep() {
    std::string str = "aplay -c 2 sheesh.wav";
    const char* command = str.c_str();
    system(command);
}

// Calculate the average depth (meters) between bottom_left and top_right
// inclusive.
float calculate_average(rs2::depth_frame& depth, Point bottom_left,
                        Point top_right) {
    // Define range of box values
    auto y_start = std::get<1>(bottom_left);
    auto x_start = std::get<0>(bottom_left);

    auto x_end = std::get<0>(top_right);
    auto y_end = std::get<1>(top_right);

    float total = 0;
    int n_points = 0;
    for (int x = x_start; x <= x_end; x++) {
        for (int y = y_start; y <= y_end; y++) {
            total += depth.get_distance(x, y);
            n_points++;
        }
    }

    return total / n_points;
}

int main(int argc, char* argv[]) try {
    rs2::pipeline p;
    rs2::config cfg;
    // cfg.enable_device_from_file(
    //     "bag_files/better_curb.bag");  // comment out to run on live data
    p.start();

    std::vector<float> v;
    while (true) {
        time_t last_alerted;
        // stream frames from the pipeline
        rs2::frameset frames = p.wait_for_frames();
        rs2::depth_frame depth = frames.get_depth_frame();

        auto width = depth.get_width();
        auto height = depth.get_height();

        // Pixel coordinates of box
        Point bottom_left = {width / 2 - BOX_WIDTH / 2,
                             height / 2 - BOX_HEIGHT / 2};
        Point top_right = {width / 2 + BOX_WIDTH / 2,
                           height / 2 + BOX_HEIGHT / 2};

        auto avg = calculate_average(depth, bottom_left, top_right);
        if (v.size() >= AVERAGE_FRAME_LENGTH) {
            v.erase(v.begin());
        }
        v.push_back(avg);

        auto rolling_average = vector_average(v);
        if (avg >= 1.07 * rolling_average) {
            if (difftime(time(NULL), last_alerted) > 5) {
                std::cout << "Obstacle Detected!" << std::endl;
                beep();
                last_alerted = time(NULL);
            }
        }

        // std::cout << "Average: Depth: " << avg << " meters" << std::endl;
    }

    return EXIT_SUCCESS;
} catch (const rs2::error& e) {
    std::cerr << "RealSense error calling " << e.get_failed_function() << "("
              << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
} catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
