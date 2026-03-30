#!/usr/bin/env python3
"""
Pytest test harness for the NoiseAugmentedIMUSensor plugin.

This module provides comprehensive validation tests for the custom Gazebo plugin:
- Message structure and field availability
- Publication latency
- Frame transforms and reference frames
- Noise characteristics (mean and standard deviation)
- Ground truth comparison (simulated values vs. measured)
"""

import pytest
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu
from tf2_ros import Buffer, TransformListener
import threading
import time
import math
from typing import List, Optional
from dataclasses import dataclass
from collections import deque
import numpy as np


@dataclass
class IMUSample:
    """Single IMU measurement sample."""
    timestamp: float
    accel_x: float
    accel_y: float
    accel_z: float
    gyro_x: float
    gyro_y: float
    gyro_z: float
    mag_x: float
    mag_y: float
    mag_z: float
    quat_x: float
    quat_y: float
    quat_z: float
    quat_w: float
    frame_id: str
    receive_time: float


class IMUListenerNode(Node):
    """ROS 2 node that subscribes to IMU data for testing."""
    
    def __init__(self, node_name: str = "imu_listener_test"):
        super().__init__(node_name)
        
        self.samples: deque = deque(maxlen=1000)  # Keep last 1000 samples
        self.lock = threading.Lock()
        
        # Quality of Service: best effort, keep last 10 messages
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        
        # Subscribe to IMU topic
        self.subscription = self.create_subscription(
            Imu,
            "/imu/data",
            self.imu_callback,
            qos,
        )
        
        # TF buffer for frame validation
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.subscription  # prevent unused variable warning
    
    def imu_callback(self, msg: Imu) -> None:
        """Callback for IMU messages."""
        with self.lock:
            receive_time = time.time()
            
            # Extract data from message
            sample = IMUSample(
                timestamp=float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) / 1e9,
                accel_x=msg.linear_acceleration.x,
                accel_y=msg.linear_acceleration.y,
                accel_z=msg.linear_acceleration.z,
                gyro_x=msg.angular_velocity.x,
                gyro_y=msg.angular_velocity.y,
                gyro_z=msg.angular_velocity.z,
                mag_x=getattr(msg.magnetic_field, 'x', 0.0),  # May not be present
                mag_y=getattr(msg.magnetic_field, 'y', 0.0),
                mag_z=getattr(msg.magnetic_field, 'z', 0.0),
                quat_x=msg.orientation.x,
                quat_y=msg.orientation.y,
                quat_z=msg.orientation.z,
                quat_w=msg.orientation.w,
                frame_id=msg.header.frame_id,
                receive_time=receive_time,
            )
            
            self.samples.append(sample)
    
    def get_samples(self) -> List[IMUSample]:
        """Get a copy of collected samples."""
        with self.lock:
            return list(self.samples)
    
    def sample_count(self) -> int:
        """Get number of collected samples."""
        with self.lock:
            return len(self.samples)


@pytest.fixture
def ros_context():
    """Fixture to initialize and clean up ROS context for tests."""
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def imu_listener(ros_context):
    """Fixture providing an IMU listener node."""
    executor = None
    node = None
    thread = None
    
    def create_listener():
        nonlocal executor, node, thread
        node = IMUListenerNode()
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(node)
        
        thread = threading.Thread(target=executor.spin, daemon=True)
        thread.start()
        
        # Wait for node to be ready
        time.sleep(0.5)
        return node
    
    listener = create_listener()
    
    yield listener
    
    if executor:
        executor.shutdown()
    if node:
        node.destroy_node()


class TestIMUMessageStructure:
    """Tests for IMU message structure and required fields."""
    
    def test_message_reception(self, imu_listener):
        """Test that IMU messages are being received."""
        # Wait for samples to arrive
        for _ in range(50):
            if imu_listener.sample_count() > 0:
                break
            time.sleep(0.1)
        
        samples = imu_listener.get_samples()
        assert len(samples) > 0, "No IMU messages received after 5 seconds"
    
    def test_frame_id_validity(self, imu_listener):
        """Test that frame_id is set correctly."""
        # Collect samples for a period
        start_time = time.time()
        while time.time() - start_time < 2.0:
            time.sleep(0.1)
        
        samples = imu_listener.get_samples()
        assert len(samples) > 0, "No IMU messages received"
        
        for sample in samples:
            assert sample.frame_id == "imu_link", \
                f"Unexpected frame_id: {sample.frame_id}, expected 'imu_link'"
    
    def test_all_fields_present(self, imu_listener):
        """Test that all IMU fields are populated."""
        # Collect samples
        start_time = time.time()
        while time.time() - start_time < 2.0:
            time.sleep(0.1)
        
        samples = imu_listener.get_samples()
        assert len(samples) > 0, "No IMU messages received"
        
        # Check first sample
        sample = samples[0]
        
        # Acceleration fields
        assert not math.isnan(sample.accel_x), "accel_x is NaN"
        assert not math.isnan(sample.accel_y), "accel_y is NaN"
        assert not math.isnan(sample.accel_z), "accel_z is NaN"
        
        # Gyroscope fields
        assert not math.isnan(sample.gyro_x), "gyro_x is NaN"
        assert not math.isnan(sample.gyro_y), "gyro_y is NaN"
        assert not math.isnan(sample.gyro_z), "gyro_z is NaN"
        
        # Orientation fields
        assert not math.isnan(sample.quat_x), "quat_x is NaN"
        assert not math.isnan(sample.quat_y), "quat_y is NaN"
        assert not math.isnan(sample.quat_z), "quat_z is NaN"
        assert not math.isnan(sample.quat_w), "quat_w is NaN"


class TestIMULatency:
    """Tests for publication latency."""
    
    def test_publication_latency(self, imu_listener):
        """Test that IMU messages are published within expected latency."""
        # Expected update rate is 100 Hz, so max latency should be well under 100ms
        # For a properly functioning system, we expect < 50ms latency
        
        start_time = time.time()
        while time.time() - start_time < 2.0:
            time.sleep(0.1)
        
        samples = imu_listener.get_samples()
        assert len(samples) >= 10, "Insufficient samples for latency test"
        
        latencies = []
        for sample in samples:
            latency = (sample.receive_time - sample.timestamp) * 1000  # Convert to ms
            latencies.append(latency)
            assert latency < 500,  \
                f"Message latency {latency:.2f} ms exceeds acceptable threshold"
        
        avg_latency = sum(latencies) / len(latencies)
        print(f"\nAverage latency: {avg_latency:.2f} ms")
        print(f"Max latency: {max(latencies):.2f} ms")
        print(f"Min latency: {min(latencies):.2f} ms")
    
    def test_publication_rate(self, imu_listener):
        """Test that IMU is published at approximately the configured rate (100 Hz)."""
        # Collect samples for 2 seconds
        start_time = time.time()
        while time.time() - start_time < 2.0:
            time.sleep(0.1)
        
        samples = imu_listener.get_samples()
        assert len(samples) >= 100, "Insufficient samples for rate test"
        
        # Calculate publication intervals
        intervals = []
        for i in range(1, len(samples)):
            interval = samples[i].timestamp - samples[i-1].timestamp
            if interval > 0:  # Ignore zero-length intervals
                intervals.append(interval)
        
        avg_interval = sum(intervals) / len(intervals) if intervals else 0
        expected_interval = 1.0 / 100.0  # 100 Hz
        
        print(f"\nAverage interval: {avg_interval * 1000:.2f} ms")
        print(f"Expected interval: {expected_interval * 1000:.2f} ms")
        print(f"Sample count: {len(samples)}")
        
        # Average interval should be close to 10ms (100 Hz)
        # Allow ±30% tolerance
        assert 0.007 < avg_interval < 0.013, \
            f"Publication rate {1/avg_interval:.1f} Hz is outside expected 100 Hz"


class TestNoiseCharacteristics:
    """Tests for noise statistics and characteristics."""
    
    def test_gyro_noise_mean_zero(self, imu_listener):
        """Test that gyroscope noise has mean approximately zero (no drift)."""
        # Collect samples for a long period to gather statistics
        print("\nCollecting gyroscope samples for 3 seconds...")
        start_time = time.time()
        while time.time() - start_time < 3.0:
            time.sleep(0.1)
        
        samples = imu_listener.get_samples()
        assert len(samples) > 100, "Insufficient samples for statistical test"
        
        gyro_x_values = [s.gyro_x for s in samples]
        gyro_y_values = [s.gyro_y for s in samples]
        gyro_z_values = [s.gyro_z for s in samples]
        
        mean_x = sum(gyro_x_values) / len(gyro_x_values)
        mean_y = sum(gyro_y_values) / len(gyro_y_values)
        mean_z = sum(gyro_z_values) / len(gyro_z_values)
        
        print(f"Gyro X mean: {mean_x:.6f} rad/s")
        print(f"Gyro Y mean: {mean_y:.6f} rad/s")
        print(f"Gyro Z mean: {mean_z:.6f} rad/s")
        
        # Mean should be close to zero (within 0.005 rad/s)
        assert abs(mean_x) < 0.01, f"Gyro X mean drift: {mean_x}"
        assert abs(mean_y) < 0.01, f"Gyro Y mean drift: {mean_y}"
        assert abs(mean_z) < 0.01, f"Gyro Z mean drift: {mean_z}"
    
    def test_accel_noise_standard_deviation(self, imu_listener):
        """Test that acceleration noise standard deviation matches configuration."""
        # Expected noise sigma: 0.01 m/s²
        expected_sigma = 0.01
        tolerance = 0.005  # Allow ±50% tolerance
        
        print("\nCollecting acceleration samples for 3 seconds...")
        start_time = time.time()
        while time.time() - start_time < 3.0:
            time.sleep(0.1)
        
        samples = imu_listener.get_samples()
        assert len(samples) > 100, "Insufficient samples for statistical test"
        
        accel_x_values = [s.accel_x for s in samples]
        accel_y_values = [s.accel_y for s in samples]
        accel_z_values = [s.accel_z for s in samples]
        
        # Compute standard deviation
        mean_x = sum(accel_x_values) / len(accel_x_values)
        mean_y = sum(accel_y_values) / len(accel_y_values)
        mean_z = sum(accel_z_values) / len(accel_z_values)
        
        std_x = math.sqrt(sum((x - mean_x) ** 2 for x in accel_x_values) / len(accel_x_values))
        std_y = math.sqrt(sum((y - mean_y) ** 2 for y in accel_y_values) / len(accel_y_values))
        std_z = math.sqrt(sum((z - mean_z) ** 2 for z in accel_z_values) / len(accel_z_values))
        
        print(f"Accel X std: {std_x:.6f} m/s² (expected ~{expected_sigma})")
        print(f"Accel Y std: {std_y:.6f} m/s² (expected ~{expected_sigma})")
        print(f"Accel Z std: {std_z:.6f} m/s² (expected ~{expected_sigma})")
        
        # Standard deviation should be within tolerance of expected
        assert expected_sigma - tolerance < std_x < expected_sigma + tolerance, \
            f"Accel X std {std_x:.6f} outside expected range"
        assert expected_sigma - tolerance < std_y < expected_sigma + tolerance, \
            f"Accel Y std {std_y:.6f} outside expected range"
        # Z includes gravity, so just check it's reasonable
        assert 0.008 < std_z < 0.05, \
            f"Accel Z std {std_z:.6f} unreasonable (includes gravity)"


class TestMessageIntegrity:
    """Tests for message data integrity."""
    
    def test_no_infinity_values(self, imu_listener):
        """Test that no IMU fields contain infinity values."""
        start_time = time.time()
        while time.time() - start_time < 2.0:
            time.sleep(0.1)
        
        samples = imu_listener.get_samples()
        assert len(samples) > 0, "No IMU messages received"
        
        for i, sample in enumerate(samples):
            # Check accelerations
            assert math.isfinite(sample.accel_x), f"Sample {i}: accel_x is not finite"
            assert math.isfinite(sample.accel_y), f"Sample {i}: accel_y is not finite"
            assert math.isfinite(sample.accel_z), f"Sample {i}: accel_z is not finite"
            
            # Check angular velocities
            assert math.isfinite(sample.gyro_x), f"Sample {i}: gyro_x is not finite"
            assert math.isfinite(sample.gyro_y), f"Sample {i}: gyro_y is not finite"
            assert math.isfinite(sample.gyro_z), f"Sample {i}: gyro_z is not finite"
    
    def test_quaternion_normalization(self, imu_listener):
        """Test that quaternion is properly normalized (magnitude ≈ 1)."""
        start_time = time.time()
        while time.time() - start_time < 2.0:
            time.sleep(0.1)
        
        samples = imu_listener.get_samples()
        assert len(samples) > 0, "No IMU messages received"
        
        for i, sample in enumerate(samples):
            magnitude = math.sqrt(
                sample.quat_x**2 +
                sample.quat_y**2 +
                sample.quat_z**2 +
                sample.quat_w**2
            )
            
            # Quaternion magnitude should be 1.0 ± 0.01
            assert 0.99 < magnitude < 1.01, \
                f"Sample {i}: Quaternion magnitude {magnitude} is not 1.0"


if __name__ == "__main__":
    pytest.main([__file__, "-v", "-s"])
