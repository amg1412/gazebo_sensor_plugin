/**
 * @file noise_imu_sensor.hpp
 * @brief Gazebo Harmonic plugin implementing a noise-augmented IMU sensor.
 *
 * This plugin uses the Entity Component System (ECS) API to simulate a 6-DOF IMU
 * (accelerometer, gyroscope, magnetometer) with configurable Gaussian noise.
 * Readings are published over Gazebo transport topics, which are bridged to ROS 2
 * via ros_gz_bridge.
 */

#ifndef GAZEBO_SENSOR_PLUGIN_NOISE_IMU_SENSOR_HPP_
#define GAZEBO_SENSOR_PLUGIN_NOISE_IMU_SENSOR_HPP_

#include <memory>
#include <string>
#include <random>

#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/Types.hh>

#include <gz/transport/Node.hh>
#include <gz/math/Vector3.hh>
#include <gz/math/Quaternion.hh>

namespace gazebo_sensor_plugin
{

/**
 * @class NoiseAugmentedIMUSensor
 * @brief Gazebo plugin simulating a noise-augmented IMU sensor.
 *
 * This plugin simulates a 6-DOF IMU (3-axis accelerometer, 3-axis gyroscope,
 * 3-axis magnetometer) attached to a robot link. It applies configurable
 * Gaussian noise to all measurements and publishes them via Gazebo transport.
 *
 * Configuration (from SDF):
 * ```xml
 * <plugin name="noise_imu_sensor" filename="libnoise_imu_sensor.so">
 *   <link_name>imu_link</link_name>
 *   <accel_noise_sigma>0.01</accel_noise_sigma>
 *   <gyro_noise_sigma>0.001</gyro_noise_sigma>
 *   <mag_noise_sigma>0.1</mag_noise_sigma>
 *   <update_rate>100</update_rate>
 * </plugin>
 * ```
 */
class NoiseAugmentedIMUSensor : public gz::sim::System,
                                 public gz::sim::ISystemConfigure,
                                 public gz::sim::ISystemPreUpdate,
                                 public gz::sim::ISystemUpdate,
                                 public gz::sim::ISystemPostUpdate
{
public:
  /// Constructor
  NoiseAugmentedIMUSensor();

  /// Destructor
  virtual ~NoiseAugmentedIMUSensor() = default;

  /**
   * @brief Configure the plugin from SDF.
   * @param entity The entity (model) this plugin is attached to.
   * @param sdf The SDF element for this plugin.
   * @param ecm Reference to the EntityComponentManager.
   * @param event_mgr Reference to the EventManager.
   */
  void Configure(const gz::sim::Entity &entity,
                 const std::shared_ptr<const sdf::Element> &sdf,
                 gz::sim::EntityComponentManager &ecm,
                 gz::sim::EventManager &event_mgr) override;

  /**
   * @brief Called just before each physics update.
   * @param info The simulation update info (time step, etc).
   * @param ecm Reference to the EntityComponentManager.
   */
  void PreUpdate(const gz::sim::UpdateInfo &info,
                 gz::sim::EntityComponentManager &ecm) override;

  /**
   * @brief Called during each physics update.
   * @param info The simulation update info.
   * @param ecm Reference to the EntityComponentManager.
   */
  void Update(const gz::sim::UpdateInfo &info,
              gz::sim::EntityComponentManager &ecm) override;

  /**
   * @brief Called just after each physics update.
   * @param info The simulation update info.
   * @param ecm Reference to the EntityComponentManager.
   */
  void PostUpdate(const gz::sim::UpdateInfo &info,
                  const gz::sim::EntityComponentManager &ecm) override;

private:
  /**
   * @brief Compute and publish IMU data.
   * @param ecm Reference to the EntityComponentManager.
   * @param info The simulation update info.
   */
  void PublishIMUData(const gz::sim::EntityComponentManager &ecm,
                       const gz::sim::UpdateInfo &info);

  /**
   * @brief Add Gaussian noise to a 3D vector.
   * @param value The original value.
   * @param sigma Standard deviation of the noise.
   * @return Noisy value.
   */
  gz::math::Vector3d AddNoise(const gz::math::Vector3d &value, double sigma);

  /**
   * @brief Sample from a normal distribution.
   * @param mean The mean of the distribution.
   * @param sigma The standard deviation.
   * @return Random sample.
   */
  double SampleNormal(double mean, double sigma);

  // ──── Configuration Parameters ────
  std::string link_name_;                ///< Name of the link with the IMU
  double accel_noise_sigma_;             ///< Acceleration noise standard deviation (m/s²)
  double gyro_noise_sigma_;              ///< Angular velocity noise std (rad/s)
  double mag_noise_sigma_;               ///< Magnetic field noise std (Tesla)
  double update_rate_;                   ///< Publish rate (Hz)
  std::string output_topic_;             ///< Gazebo transport topic for IMU data

  // ──── State ────
  gz::sim::Entity model_entity_;         ///< Model entity
  gz::sim::Entity imu_link_entity_;      ///< IMU link entity
  std::unique_ptr<gz::transport::Node> gz_node_;  ///< Gazebo transport node
  gz::transport::Node::Publisher imu_pub_;        ///< IMU data publisher
  std::mt19937 rng_;                     ///< Random number generator
  int64_t last_update_ns_;               ///< Timestamp of last update (nanoseconds)
};

}  // namespace gazebo_sensor_plugin

#endif  // GAZEBO_SENSOR_PLUGIN_NOISE_IMU_SENSOR_HPP_
