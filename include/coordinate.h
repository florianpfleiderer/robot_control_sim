// Created on Fri Nov 14 2025 by Florian Pfleiderer

#pragma once

#include <type_traits>

template <typename T> struct Coordinate {
  T x {};
  T y {};

  Coordinate operator+(const Coordinate &other) const noexcept {
    return {x + other.x, y + other.y};
  }

  Coordinate operator-(const Coordinate &other) const noexcept {
    return {x - other.x, y - other.y};
  }

  Coordinate operator*(T k) const noexcept {
    return {x * k, y * k};
  }

  Coordinate operator/(T k) const noexcept {
    return {x / k, y / k};
  }

  Coordinate &operator+=(const Coordinate &other) {
    x += other.x;
    y += other.y;
    return *this;
  }

  Coordinate &operator*=(T k) noexcept {
    x *= k;
    y *= k;
    return *this;
  }

  bool operator<(const Coordinate &other) {
    return x < other.x && y < other.y;
  }
};