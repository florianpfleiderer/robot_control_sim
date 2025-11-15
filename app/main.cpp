// Created on Fri Nov 14 2025 by Florian Pfleiderer

#include "robot.h"
#include "controller.h"
#include "sensor.h"

#include <iostream>
#include <fstream>
#include <vector>

int main() {
    Robot robot{};
    Sensor sensor{0.05};
    Controller controller{0.8, 0.1, 0.05};

    const double target_x = 5.0;
    const double target_y = 5.0;
    const double dt       = 0.05;

    std::vector<double> true_xs, true_ys, sensor_xs, sensor_ys;

    for (int step = 0; step < 300; ++step) {
        auto [true_x, true_y] = robot.position();
        auto [sx, sy] = sensor.read(true_x, true_y);
        auto [ax, ay] = controller.compute(target_x, target_y, sx, sy, dt);
        
        robot.update(ax, ay, dt);

        true_xs.push_back(true_x);
        true_ys.push_back(true_y);
        sensor_xs.push_back(sx);
        sensor_ys.push_back(sy);

        if (step % 20 == 0) {
            std::cout << "Step " << step
                      << " true=(" << true_x << ", " << true_y << ")"
                      << " sensor=(" << sx << ", " << sy << ")\n";
        }
    }

    // Save to CSV file
    std::ofstream file("trajectory_data.csv");
    file << "true_x,true_y,sensor_x,sensor_y\n";
    for (size_t i = 0; i < true_xs.size(); ++i) {
        file << true_xs[i] << "," << true_ys[i] << "," 
             << sensor_xs[i] << "," << sensor_ys[i] << "\n";
    }
    file.close();

    return 0;
}
