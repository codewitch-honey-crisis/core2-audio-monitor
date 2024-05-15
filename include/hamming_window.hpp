#pragma once
#include <stddef.h>
#include <inttypes.h>
#include <math.h>
template<size_t WindowSize = 512> class hamming_window {
 private:
  float m_coefficients[WindowSize];

 public:
  static constexpr const size_t window_size = WindowSize;
  hamming_window()
  {
    // create the constants for a hamming window
    const float arg = M_PI * 2.0 / window_size;
    for (int i = 0; i < window_size; i++) {
      float float_value = 0.5 - (0.5 * cos(arg * (i + 0.5)));
      // Scale it to fixed point and round it.
      m_coefficients[i] = float_value;
    }
  }
  void apply(float *input)
  {
    for (int i = 0; i < window_size; i++) {
      input[i] = input[i] * m_coefficients[i];
    }
  }
};
