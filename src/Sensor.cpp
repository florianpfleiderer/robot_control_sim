// Created on Sat Nov 15 2025 by Florian Pfleiderer

#include "sensor.h"

Sensor::Sensor(double noise_std)
    : noise_std_ {noise_std},
      // Seed RNG using a non-deterministic source
      rng_ {std::random_device {}()},
      // Normal distribution with mean 0 and stddev noise_std
      dist_ {0.0, noise_std} {}

std::pair<double, double> Sensor::read(double true_x, double true_y) {
  // Generate independent Gaussian noise samples
  const double nx = true_x + dist_(rng_);
  const double ny = true_y + dist_(rng_);

  return {nx, ny};
}
