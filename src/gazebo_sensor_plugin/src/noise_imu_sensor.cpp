/**
 * @file noise_imu_sensor.cpp
 * @brief Implementation of the NoiseAugmentedIMUSensor plugin.
 */

#include "gazebo_sensor_plugin/noise_imu_sensor.hpp"

#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/LinearAcceleration.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>

#include <gz/common/Profiler.hh>

#include <sdf/sdf.hh>

#include <iostream>
#include <cmath>

GZ_ADD_PLUGIN(gazebo_sensor_plugin::NoiseAugmentedIMUSensor,
              gz::sim::System,
              gazebo_sensor_plugin::NoiseAugmentedIMUSensor::ISystemConfigure,
              gazebo_sensor_plugin::NoiseAugmentedIMUSensor::ISystemPreUpdate,
              gazebo_sensor_plugin::NoiseAugmentedIMUSensor::ISystemUpdate,
              gazebo_sensor_plugin::NoiseAugmentedIMUSensor::ISystemPostUpdate)

namespace gazebo_sensor_plugin
{

// ─────────────────────────────────────────────────────────────────────────────
// Constructor
// ─────────────────────────────────────────────────────────────────────────────

NoiseAugmentedIMUSensor::NoiseAugmentedIMUSensor()
    : link_name_(""),
      accel_noise_sigma_(0.01),
      gyro_noise_sigma_(0.001),
      mag_noise_sigma_(0.1),
      update_rate_(100.0),
      output_topic_("/gz/sim/imu/data"),
      model_entity_(gz::sim::kNullEntity),
      imu_link_entity_(gz::sim::kNullEntity),
      last_update_ns_(0)
{
  // Seed RNG with current time
  std::random_device rd;
  rng_.seed(rd());

  std::cout << "[NoiseAugmentedIMUSensor] Plugin instantiated" << std::endl;
}

// ─────────────────────────────────────────────────────────────────────────────
// Configure (lifecycle hook)
// ─────────────────────────────────────────────────────────────────────────────

void NoiseAugmentedIMUSensor::Configure(
    const gz::sim::Entity &entity,
    const std::shared_ptr<const sdf::Element> &sdf,
    gz::sim::EntityComponentManager &ecm,
    gz::sim::EventManager &/*event_mgr*/)
{
  GZ_PROFILE("NoiseAugmentedIMUSensor::Configure");

  std::cout << "[NoiseAugmentedIMUSensor] Configuring plugin..." << std::endl;

  model_entity_ = entity;

  // Parse SDF parameters
  if (sdf->HasElement("link_name"))
  {
    link_name_ = sdf->GetElement("link_name")->GetValue()->GetAsString();
    std::cout << "  - Link name: " << link_name_ << std::endl;
  }
  else
  {
    std::cerr << "  ERROR: <link_name> not specified in SDF!" << std::endl;
    return;
  }

  if (sdf->HasElement("accel_noise_sigma"))
  {
    accel_noise_sigma_ = sdf->GetElement("accel_noise_sigma")->GetValue()->GetAsDouble();
    std::cout << "  - Accel noise sigma: " << accel_noise_sigma_ << std::endl;
  }

  if (sdf->HasElement("gyro_noise_sigma"))
  {
    gyro_noise_sigma_ = sdf->GetElement("gyro_noise_sigma")->GetValue()->GetAsDouble();
    std::cout << "  - Gyro noise sigma: " << gyro_noise_sigma_ << std::endl;
  }

  if (sdf->HasElement("mag_noise_sigma"))
  {
    mag_noise_sigma_ = sdf->GetElement("mag_noise_sigma")->GetValue()->GetAsDouble();
    std::cout << "  - Mag noise sigma: " << mag_noise_sigma_ << std::endl;
  }

  if (sdf->HasElement("update_rate"))
  {
    update_rate_ = sdf->GetElement("update_rate")->GetValue()->GetAsDouble();
    std::cout << "  - Update rate: " << update_rate_ << " Hz" << std::endl;
  }

  if (sdf->HasElement("output_topic"))
  {
    output_topic_ = sdf->GetElement("output_topic")->GetValue()->GetAsString();
    std::cout << "  - Output topic: " << output_topic_ << std::endl;
  }

  // Find the IMU link within the model
  auto model = gz::sim::Model(entity);
  imu_link_entity_ = model.LinkByName(ecm, link_name_);

  if (imu_link_entity_ == gz::sim::kNullEntity)
  {
    std::cerr << "  ERROR: Link '" << link_name_ << "' not found in model!" << std::endl;
    return;
  }

  std::cout << "  - Found IMU link (entity ID: " << imu_link_entity_ << ")" << std::endl;

  // Create Gazebo transport node and publisher for IMU data
  gz_node_ = std::make_unique<gz::transport::Node>();
  imu_pub_ = gz_node_->Advertise<gz::msgs::IMU>(output_topic_);

  std::cout << "[NoiseAugmentedIMUSensor] Configuration complete!" << std::endl;
}

// ─────────────────────────────────────────────────────────────────────────────
// PreUpdate (before physics step)
// ─────────────────────────────────────────────────────────────────────────────

void NoiseAugmentedIMUSensor::PreUpdate(const gz::sim::UpdateInfo &/*info*/,
                                         gz::sim::EntityComponentManager &/*ecm*/)
{
  GZ_PROFILE("NoiseAugmentedIMUSensor::PreUpdate");
  // Currently no pre-update work needed
}

// ─────────────────────────────────────────────────────────────────────────────
// Update (during physics step)
// ─────────────────────────────────────────────────────────────────────────────

void NoiseAugmentedIMUSensor::Update(const gz::sim::UpdateInfo &/*info*/,
                                      gz::sim::EntityComponentManager &/*ecm*/)
{
  GZ_PROFILE("NoiseAugmentedIMUSensor::Update");
  // Currently no update work needed (publishing happens in PostUpdate)
}

// ─────────────────────────────────────────────────────────────────────────────
// PostUpdate (after physics step)
// ─────────────────────────────────────────────────────────────────────────────

void NoiseAugmentedIMUSensor::PostUpdate(const gz::sim::UpdateInfo &info,
                                          const gz::sim::EntityComponentManager &ecm)
{
  GZ_PROFILE("NoiseAugmentedIMUSensor::PostUpdate");

  // Check if enough time has passed since last update
  int64_t update_period_ns = static_cast<int64_t>(1e9 / update_rate_);
  if (info.simTime.count() - last_update_ns_ >= update_period_ns)
  {
    last_update_ns_ = info.simTime.count();
    PublishIMUData(ecm, info);
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// Publish IMU Data
// ─────────────────────────────────────────────────────────────────────────────

void NoiseAugmentedIMUSensor::PublishIMUData(
    const gz::sim::EntityComponentManager &ecm,
    const gz::sim::UpdateInfo &info)
{
  GZ_PROFILE("NoiseAugmentedIMUSensor::PublishIMUData");

  if (imu_link_entity_ == gz::sim::kNullEntity)
    return;

  // Get link velocities from ECS
  auto linear_vel_ptr = ecm.Component<gz::sim::components::LinearVelocity>(imu_link_entity_);
  auto angular_vel_ptr = ecm.Component<gz::sim::components::AngularVelocity>(imu_link_entity_);
  auto linear_accel_ptr = ecm.Component<gz::sim::components::LinearAcceleration>(imu_link_entity_);
  auto pose_ptr = ecm.Component<gz::sim::components::Pose>(imu_link_entity_);

  if (!linear_vel_ptr || !angular_vel_ptr || !linear_accel_ptr || !pose_ptr)
  {
    // Components not yet available; skip this update
    return;
  }

  // Extract kinematic data
  const auto &linear_vel = linear_vel_ptr->Data();
  const auto &angular_vel = angular_vel_ptr->Data();
  const auto &linear_accel = linear_accel_ptr->Data();
  const auto &pose = pose_ptr->Data();

  // Create IMU message
  gz::msgs::IMU imu_msg;
  imu_msg.set_entity_name(link_name_);

  // Set header (timestamp from simulation)
  imu_msg.mutable_header()->mutable_stamp()->set_sec(info.simTime.count() / 1000000000);
  imu_msg.mutable_header()->mutable_stamp()->set_nsec(info.simTime.count() % 1000000000);
  imu_msg.mutable_header()->set_frame_id("imu_link");

  // ──── Accelerometer data (simulated from linear acceleration) ────
  auto accel_with_noise = AddNoise(linear_accel, accel_noise_sigma_);
  auto accel_msg = imu_msg.mutable_linear_acceleration();
  accel_msg->set_x(accel_with_noise.X());
  accel_msg->set_y(accel_with_noise.Y());
  accel_msg->set_z(accel_with_noise.Z());

  // ──── Gyroscope data (angular velocity with noise) ────
  auto gyro_with_noise = AddNoise(angular_vel, gyro_noise_sigma_);
  auto gyro_msg = imu_msg.mutable_angular_velocity();
  gyro_msg->set_x(gyro_with_noise.X());
  gyro_msg->set_y(gyro_with_noise.Y());
  gyro_msg->set_z(gyro_with_noise.Z());

  // ──── Magnetometer data (simulated as Earth's magnetic field + noise) ────
  // Earth's magnetic field magnitude ~ 50 μT, pointing roughly North and down
  gz::math::Vector3d magnetic_field(50e-6 * std::cos(0.6), 0, -50e-6 * std::sin(0.6));
  auto mag_with_noise = AddNoise(magnetic_field, mag_noise_sigma_);
  auto mag_msg = imu_msg.mutable_magnetic_field();
  mag_msg->set_x(mag_with_noise.X());
  mag_msg->set_y(mag_with_noise.Y());
  mag_msg->set_z(mag_with_noise.Z());

  // ──── Orientation (quaternion from pose) ────
  const auto &orientation = pose.Rot();
  auto orient_msg = imu_msg.mutable_orientation();
  orient_msg->set_x(orientation.X());
  orient_msg->set_y(orientation.Y());
  orient_msg->set_z(orientation.Z());
  orient_msg->set_w(orientation.W());

  // Publish the IMU message
  imu_pub_.Publish(imu_msg);
}

// ─────────────────────────────────────────────────────────────────────────────
// Noise Addition
// ─────────────────────────────────────────────────────────────────────────────

gz::math::Vector3d NoiseAugmentedIMUSensor::AddNoise(const gz::math::Vector3d &value,
                                                      double sigma)
{
  if (sigma <= 0.0)
    return value;

  return gz::math::Vector3d(
      value.X() + SampleNormal(0.0, sigma),
      value.Y() + SampleNormal(0.0, sigma),
      value.Z() + SampleNormal(0.0, sigma)
  );
}

// ─────────────────────────────────────────────────────────────────────────────
// Normal Distribution Sampling
// ─────────────────────────────────────────────────────────────────────────────

double NoiseAugmentedIMUSensor::SampleNormal(double mean, double sigma)
{
  if (sigma <= 0.0)
    return mean;

  std::normal_distribution<double> dist(mean, sigma);
  return dist(rng_);
}

}  // namespace gazebo_sensor_plugin
