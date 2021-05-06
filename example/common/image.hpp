#ifndef _IMAGE_H
#define _IMAGE_H
#include <fstream>
#include <iostream>
#include <string>

#include "core/vec3.hpp"

class Image {
 private:
  int width;
  int height;
  float* pixels;

 public:
  Image(int width, int height) : width(width), height(height) {
    pixels = new float[3 * width * height];
  }
  ~Image() { delete[] pixels; }

  Vec3 getPixel(int i, int j) const {
    if (i < 0 || i >= width || j < 0 || j >= height) {
      std::cerr << "invalid pixel index" << std::endl;
      std::exit(EXIT_FAILURE);
    }

    return Vec3(pixels[3 * i + 3 * width * j],
                pixels[3 * i + 3 * width * j + 1],
                pixels[3 * i + 3 * width * j + 2]);
  }

  void setPixel(int i, int j, const Vec3& c) {
    if (i < 0 || i >= width || j < 0 || j >= height) {
      std::cerr << "invalid pixel index" << std::endl;
      std::exit(EXIT_FAILURE);
    }

    pixels[3 * i + 3 * width * j] = c[0];
    pixels[3 * i + 3 * width * j + 1] = c[1];
    pixels[3 * i + 3 * width * j + 2] = c[2];
  }

  void writePPM(const std::string& filename) const {
    std::ofstream file(filename);
    if (!file) {
      std::cerr << "failed to open " << filename << std::endl;
      std::exit(EXIT_FAILURE);
    }

    file << "P3" << std::endl;
    file << width << " " << height << std::endl;
    file << "255" << std::endl;
    for (int j = 0; j < height; ++j) {
      for (int i = 0; i < width; ++i) {
        const Vec3 c = getPixel(i, j);
        const int R = std::clamp(static_cast<int>(255.0f * c[0]), 0, 255);
        const int G = std::clamp(static_cast<int>(255.0f * c[1]), 0, 255);
        const int B = std::clamp(static_cast<int>(255.0f * c[2]), 0, 255);
        file << R << " " << G << " " << B << std::endl;
      }
    }

    file.close();
  }

  void gammaCorrection() {
    for (int j = 0; j < height; ++j) {
      for (int i = 0; i < width; ++i) {
        Vec3 c = getPixel(i, j);
        c[0] = std::pow(c[0], 1 / 2.2f);
        c[1] = std::pow(c[1], 1 / 2.2f);
        c[2] = std::pow(c[2], 1 / 2.2f);
        setPixel(i, j, c);
      }
    }
  }
};

#endif