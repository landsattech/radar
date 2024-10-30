#ifndef ANGULAR_SPEED_ESTIMATOR_H
#define ANGULAR_SPEED_ESTIMATOR_H

#include <map>
#include <chrono>
#include <cmath>

// Define a type alias for time points using std::chrono
using TimePoint = std::chrono::steady_clock::time_point;
using Duration = std::chrono::duration<double>;

struct AngularSpeedEstimator
{
  // Current estimated angular speed
  double angular_speed = 0.0;

  // Latest measured angular speed from sensor
  double measured_angular_speed = 0.0;

  // Error between prediction and measurement
  double prediction_error = 0.0;

  // Variance of the prediction
  double prediction_variance = 0.0;

  // Current estimate of variance
  double variance = 1.0;

  // Measurement and process noise variances
  const double measurement_variance = std::pow(0.045, 2.0);
  const double process_noise_variance = std::pow(0.0015, 2.0); // Allows for RPM change

  // Buffer to store angle measurements with their corresponding time points
  std::map<TimePoint, double> measurement_buffer;

  // Duration for which measurements are kept in the buffer
  const Duration measurement_buffer_duration = Duration(0.75); // 0.75 seconds

  // Maximum allowable gap between measurements before resetting estimates
  const Duration max_measurement_gap = Duration(0.45); // 0.45 seconds

  /**
   * @brief Updates the angular speed estimate based on a new angle measurement.
   *
   * @param t     The time point of the new measurement.
   * @param angle The measured angle in radians.
   * @return double The updated angular speed estimate.
   */
  double update(TimePoint t, double angle)
  {
    // Check if the buffer is empty or the new measurement is within the allowable time gap
    if (measurement_buffer.empty() ||
        (t > measurement_buffer.rbegin()->first &&
         (t - measurement_buffer.rbegin()->first) < max_measurement_gap))
    {
      // Remove measurements that are older than the buffer duration
      while (!measurement_buffer.empty() &&
             (t - measurement_buffer.begin()->first) > measurement_buffer_duration)
      {
        measurement_buffer.erase(measurement_buffer.begin());
      }

      if (!measurement_buffer.empty())
      {
        // Determine if the angle is increasing or decreasing
        bool positive = angle > measurement_buffer.rbegin()->second;

        double angle_difference = angle - measurement_buffer.rbegin()->second;

        // Correct for angle wrapping (assuming angles are in [0, 2π))
        if (std::abs(angle_difference) > M_PI)
        {
          positive = !positive;
        }

        angle_difference = angle - measurement_buffer.begin()->second;

        if (positive && angle_difference < 0.0)
        {
          angle_difference += 2.0 * M_PI;
        }
        if (!positive && angle_difference > 0.0)
        {
          angle_difference -= 2.0 * M_PI;
        }

        // Kalman Filter Prediction Step
        double prediction_variance_factor = prediction_variance / measurement_variance;
        double estimated_variance = variance + process_noise_variance * prediction_variance_factor;

        // Calculate time difference in seconds
        Duration time_diff = t - measurement_buffer.begin()->first;
        double time_diff_sec = time_diff.count();

        if (time_diff_sec <= 0.0)
        {
          // Avoid division by zero or negative time differences
          time_diff_sec = 1e-6; // Small positive value
        }

        // Measurement Update
        measured_angular_speed = angle_difference / time_diff_sec;

        // Kalman Gain
        double k = estimated_variance / (estimated_variance + measurement_variance);

        // Update prediction error
        prediction_error = measured_angular_speed - angular_speed;

        // Update prediction variance
        prediction_variance = k * prediction_variance + (1.0 - k) * prediction_error * prediction_error;

        // Update angular speed estimate
        angular_speed += k * prediction_error;

        // Update variance
        variance = (1.0 - k) * estimated_variance;
      }

      // Add the new measurement to the buffer
      measurement_buffer[t] = angle;
    }
    else
    {
      // If the time gap is too large, reset the estimator
      angular_speed = 0.0;
      measured_angular_speed = 0.0;
      variance = 1.0;
      measurement_buffer.clear();
      prediction_variance = 0.0;
    }

    return angular_speed;
  }
};

#endif // ANGULAR_SPEED_ESTIMATOR_H
