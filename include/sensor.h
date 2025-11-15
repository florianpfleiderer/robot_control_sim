// Created on Sat Nov 15 2025 by Florian Pfleiderer

#pragma once

#include <random>
#include <utility>

class Sensor {
public:
    explicit Sensor(double noise_std);

    [[nodiscard]] std::pair<double, double>
    read(double true_x, double true_y);

private:
    double noise_std_{0.0};
    std::mt19937 rng_;
    std::normal_distribution<double> dist_;
};
