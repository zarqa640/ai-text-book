---
sidebar_position: 3
---

# Chapter 3: Sensor Simulation

## Simulating Robot Sensors

Learn how to simulate various robot sensors including LiDAR, depth cameras, and IMU. Sensor simulation is crucial for humanoid robotics as it allows you to test perception algorithms in a safe, controlled environment before deploying on physical hardware.

## LiDAR Sensor Simulation in Gazebo

Gazebo provides realistic LiDAR simulation for humanoid robots. Here's how to configure a LiDAR sensor in your robot's URDF:

```xml
<!-- LiDAR sensor definition -->
<gazebo reference="lidar_link">
  <sensor type="ray" name="lidar_sensor">
    <pose>0 0 0 0 0 0</pose>
    <visualize>true</visualize>
    <update_rate>10</update_rate>
    <ray>
      <scan>
        <horizontal>
          <samples>720</samples>
          <resolution>1</resolution>
          <min_angle>-1.570796</min_angle> <!-- -90 degrees -->
          <max_angle>1.570796</max_angle>   <!-- 90 degrees -->
        </horizontal>
      </scan>
      <range>
        <min>0.1</min>
        <max>30.0</max>
        <resolution>0.01</resolution>
      </range>
    </ray>
    <plugin name="lidar_controller" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <namespace>/humanoid_robot</namespace>
        <remapping>~/out:=scan</remapping>
      </ros>
      <output_type>sensor_msgs/LaserScan</output_type>
    </plugin>
  </sensor>
</gazebo>
```

## Depth Camera Simulation

For humanoid robots that require 3D perception, depth cameras are essential:

```xml
<!-- Depth camera sensor -->
<gazebo reference="camera_link">
  <sensor type="depth" name="depth_camera">
    <always_on>true</always_on>
    <visualize>true</visualize>
    <update_rate>30</update_rate>
    <camera name="head">
      <horizontal_fov>1.047</horizontal_fov> <!-- 60 degrees -->
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.1</near>
        <far>10.0</far>
      </clip>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_openni_kinect.so">
      <ros>
        <namespace>/humanoid_robot</namespace>
        <remapping>~/rgb/image_raw:=/camera/color/image_raw</remapping>
        <remapping>~/depth/image_raw:=/camera/depth/image_raw</remapping>
        <remapping>~/rgb/camera_info:=/camera/color/camera_info</remapping>
      </ros>
      <camera_name>camera</camera_name>
      <image_topic_name>rgb/image_raw</image_topic_name>
      <depth_image_topic_name>depth/image_raw</depth_image_topic_name>
      <depth_image_camera_info_topic_name>depth/camera_info</depth_image_camera_info_topic_name>
      <point_cloud_topic_name>depth/points</point_cloud_topic_name>
      <frame_name>camera_depth_optical_frame</frame_name>
      <baseline>0.1</baseline>
      <distortion_k1>0.0</distortion_k1>
      <distortion_k2>0.0</distortion_k2>
      <distortion_k3>0.0</distortion_k3>
      <distortion_t1>0.0</distortion_t1>
      <distortion_t2>0.0</distortion_t2>
      <point_cloud_cutoff>0.5</point_cloud_cutoff>
      <point_cloud_cutoff_max>3.0</point_cloud_cutoff_max>
    </plugin>
  </sensor>
</gazebo>
```

## IMU Sensor Simulation

Inertial Measurement Units (IMUs) are critical for humanoid robot balance and orientation:

```xml
<!-- IMU sensor -->
<gazebo reference="imu_link">
  <sensor name="imu_sensor" type="imu">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <visualize>false</visualize>
    <imu>
      <angular_velocity>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
          </noise>
        </z>
      </angular_velocity>
      <linear_acceleration>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
          </noise>
        </z>
      </linear_acceleration>
    </imu>
    <plugin filename="libgazebo_ros_imu_sensor.so" name="imu_plugin">
      <ros>
        <namespace>/humanoid_robot</namespace>
        <remapping>~/out:=imu/data</remapping>
      </ros>
      <frame_name>imu_link</frame_name>
      <body_name>imu_link</body_name>
      <update_rate>100</update_rate>
    </plugin>
  </sensor>
</gazebo>
```

## Sensor Processing Node

Create a ROS 2 node to process and fuse sensor data from multiple sources:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, Imu, PointCloud2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped
import numpy as np
import cv2
from cv_bridge import CvBridge
from tf2_ros import TransformListener, Buffer
import tf2_geometry_msgs
import tf_transformations

class SensorProcessor(Node):
    def __init__(self):
        super().__init__('sensor_processor')

        # Initialize sensor subscribers
        self.lidar_subscription = self.create_subscription(
            LaserScan,
            '/humanoid_robot/scan',
            self.lidar_callback,
            10
        )

        self.camera_subscription = self.create_subscription(
            Image,
            '/humanoid_robot/camera/color/image_raw',
            self.camera_callback,
            10
        )

        self.imu_subscription = self.create_subscription(
            Imu,
            '/humanoid_robot/imu/data',
            self.imu_callback,
            10
        )

        # Publishers for processed data
        self.obstacle_publisher = self.create_publisher(
            PointCloud2,
            '/processed_obstacles',
            10
        )

        self.fused_sensor_publisher = self.create_publisher(
            Odometry,
            '/fused_sensor_data',
            10
        )

        # Initialize CV bridge
        self.cv_bridge = CvBridge()

        # Sensor data storage
        self.lidar_data = None
        self.camera_data = None
        self.imu_data = None

        # TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def lidar_callback(self, msg):
        """Process LiDAR data for obstacle detection"""
        self.lidar_data = msg
        self.process_lidar_data(msg)

    def camera_callback(self, msg):
        """Process camera data for visual perception"""
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
            self.camera_data = cv_image
            self.process_camera_data(cv_image)
        except Exception as e:
            self.get_logger().error(f'Error processing camera image: {e}')

    def imu_callback(self, msg):
        """Process IMU data for orientation and balance"""
        self.imu_data = msg
        self.process_imu_data(msg)

    def process_lidar_data(self, lidar_msg):
        """Process LiDAR data for obstacle detection"""
        # Convert to numpy array for processing
        ranges = np.array(lidar_msg.ranges)

        # Filter out invalid ranges
        valid_ranges = ranges[(ranges > lidar_msg.range_min) &
                             (ranges < lidar_msg.range_max)]

        # Detect obstacles within a certain distance
        obstacle_threshold = 1.0  # meters
        obstacle_indices = np.where(ranges < obstacle_threshold)[0]

        if len(obstacle_indices) > 0:
            self.get_logger().info(f'Detected {len(obstacle_indices)} obstacles')

            # Calculate obstacle positions in robot frame
            angles = np.linspace(lidar_msg.angle_min,
                               lidar_msg.angle_max,
                               len(lidar_msg.ranges))

            obstacle_positions = []
            for idx in obstacle_indices:
                angle = angles[idx]
                distance = ranges[idx]

                x = distance * np.cos(angle)
                y = distance * np.sin(angle)

                obstacle_positions.append([x, y, 0.0])  # z=0 for ground obstacles

    def process_camera_data(self, cv_image):
        """Process camera data for object detection"""
        # Example: Simple color-based object detection
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Define range for red color detection (for testing)
        lower_red = np.array([0, 50, 50])
        upper_red = np.array([10, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red, upper_red)

        lower_red = np.array([170, 50, 50])
        upper_red = np.array([180, 255, 255])

        mask2 = cv2.inRange(hsv, lower_red, upper_red)

        mask = mask1 + mask2

        # Find contours of detected objects
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 100:  # Filter out small detections
                # Calculate centroid of the object
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])

                    # Convert pixel coordinates to real-world coordinates
                    # This is a simplified conversion - in practice, you'd use camera calibration
                    real_x = (cx - cv_image.shape[1] / 2) * 0.01  # Approximate conversion
                    real_y = (cy - cv_image.shape[0] / 2) * 0.01

                    self.get_logger().info(f'Detected object at ({real_x:.2f}, {real_y:.2f})')

    def process_imu_data(self, imu_msg):
        """Process IMU data for orientation and balance"""
        # Extract orientation from IMU
        orientation = imu_msg.orientation

        # Convert quaternion to euler angles
        euler = tf_transformations.euler_from_quaternion([
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w
        ])

        roll, pitch, yaw = euler

        # Check if robot is tilting too much (for balance control)
        tilt_threshold = 0.3  # radians
        if abs(roll) > tilt_threshold or abs(pitch) > tilt_threshold:
            self.get_logger().warn(f'Robot tilt detected: roll={roll:.2f}, pitch={pitch:.2f}')

        # Process angular velocity for motion detection
        angular_vel = imu_msg.angular_velocity
        linear_acc = imu_msg.linear_acceleration

        # Publish fused sensor data
        self.publish_fused_data(roll, pitch, yaw, linear_acc)

    def publish_fused_data(self, roll, pitch, yaw, linear_acc):
        """Publish fused sensor data as odometry"""
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        # For simulation, we'll use IMU orientation directly
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = np.sin(yaw/2)
        odom_msg.pose.pose.orientation.w = np.cos(yaw/2)

        # Publish to fused sensor topic
        self.fused_sensor_publisher.publish(odom_msg)

class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('sensor_fusion')

        # Subscriptions for different sensor types
        self.lidar_sub = self.create_subscription(
            LaserScan, '/humanoid_robot/scan', self.lidar_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, '/humanoid_robot/imu/data', self.imu_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        # Publisher for fused sensor data
        self.fused_pub = self.create_publisher(Odometry, '/fused_odom', 10)

        # Sensor fusion parameters
        self.lidar_data = None
        self.imu_data = None
        self.odom_data = None
        self.fusion_weights = {'lidar': 0.3, 'imu': 0.4, 'odom': 0.3}

    def lidar_callback(self, msg):
        self.lidar_data = msg

    def imu_callback(self, msg):
        self.imu_data = msg

    def odom_callback(self, msg):
        self.odom_data = msg

    def fuse_sensors(self):
        """Fuse sensor data using weighted average"""
        if self.lidar_data and self.imu_data and self.odom_data:
            # Create fused odometry message
            fused_msg = Odometry()
            fused_msg.header.stamp = self.get_clock().now().to_msg()
            fused_msg.header.frame_id = 'map'
            fused_msg.child_frame_id = 'base_footprint'

            # Combine position estimates (simplified)
            fused_msg.pose.pose.position.x = (
                self.fusion_weights['lidar'] * self.lidar_data.range_min +
                self.fusion_weights['imu'] * self.odom_data.pose.pose.position.x +
                self.fusion_weights['odom'] * self.odom_data.pose.pose.position.x
            )

            # Combine orientation from IMU
            fused_msg.pose.pose.orientation = self.imu_data.orientation

            # Publish fused data
            self.fused_pub.publish(fused_msg)

def main(args=None):
    rclpy.init(args=args)

    sensor_processor = SensorProcessor()
    sensor_fusion = SensorFusionNode()

    # Create executor and add nodes
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(sensor_processor)
    executor.add_node(sensor_fusion)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        sensor_processor.destroy_node()
        sensor_fusion.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Unity Sensor Simulation

For Unity-based sensor simulation, you can create virtual sensors that publish to ROS:

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Sensor;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Geometry;

public class UnitySensorSimulation : MonoBehaviour
{
    [Header("Sensor Topics")]
    [SerializeField] private string lidarTopic = "/unity_lidar_scan";
    [SerializeField] private string cameraTopic = "/unity_camera_image";
    [SerializeField] private string imuTopic = "/unity_imu_data";

    [Header("Sensor Parameters")]
    [SerializeField] private int lidarResolution = 360;
    [SerializeField] private float lidarRange = 10.0f;
    [SerializeField] private float lidarUpdateRate = 10.0f; // Hz

    private ROSConnection ros;
    private Camera sensorCamera;
    private RenderTexture sensorRenderTexture;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();

        // Register publishers
        ros.RegisterPublisher<sensor_msgs.LaserScan>(lidarTopic);
        ros.RegisterPublisher<sensor_msgs.Image>(cameraTopic);
        ros.RegisterPublisher<sensor_msgs.Imu>(imuTopic);

        // Set up camera for depth sensing
        sensorCamera = GetComponent<Camera>();
        sensorRenderTexture = new RenderTexture(640, 480, 24);
        sensorCamera.targetTexture = sensorRenderTexture;

        // Start sensor simulation
        InvokeRepeating("SimulateLidar", 0.0f, 1.0f / lidarUpdateRate);
        InvokeRepeating("SimulateCamera", 0.0f, 0.1f); // 10 Hz
        InvokeRepeating("SimulateIMU", 0.0f, 0.01f);   // 100 Hz
    }

    void SimulateLidar()
    {
        // Generate simulated LiDAR data
        float[] ranges = new float[lidarResolution];
        for (int i = 0; i < lidarResolution; i++)
        {
            float angle = (2 * Mathf.PI * i) / lidarResolution;

            // Raycast in the direction of the sensor beam
            Vector3 direction = new Vector3(
                Mathf.Cos(angle),
                0,
                Mathf.Sin(angle)
            );

            RaycastHit hit;
            if (Physics.Raycast(transform.position, direction, out hit, lidarRange))
            {
                ranges[i] = hit.distance;
            }
            else
            {
                ranges[i] = lidarRange; // No obstacle detected
            }
        }

        // Create and publish LaserScan message
        sensor_msgs.LaserScan lidarMsg = new sensor_msgs.LaserScan
        {
            header = new std_msgs.Header
            {
                stamp = ROSConnection.GetTime(),
                frame_id = "lidar_frame"
            },
            angle_min = 0,
            angle_max = 2 * Mathf.PI,
            angle_increment = (2 * Mathf.PI) / lidarResolution,
            time_increment = 0,
            scan_time = 1.0f / lidarUpdateRate,
            range_min = 0.1f,
            range_max = lidarRange,
            ranges = ranges,
            intensities = new float[lidarResolution] // All zeros for now
        };

        ros.Publish(lidarTopic, lidarMsg);
    }

    void SimulateCamera()
    {
        // Capture camera image and publish as ROS message
        RenderTexture.active = sensorRenderTexture;
        Texture2D imageTex = new Texture2D(sensorRenderTexture.width, sensorRenderTexture.height, TextureFormat.RGB24, false);
        imageTex.ReadPixels(new Rect(0, 0, sensorRenderTexture.width, sensorRenderTexture.height), 0, 0);
        imageTex.Apply();

        sensor_msgs.Image imageMsg = new sensor_msgs.Image
        {
            header = new std_msgs.Header
            {
                stamp = ROSConnection.GetTime(),
                frame_id = "camera_frame"
            },
            height = (uint)sensorRenderTexture.height,
            width = (uint)sensorRenderTexture.width,
            encoding = "rgb8",
            is_bigendian = 0,
            step = (uint)(sensorRenderTexture.width * 3),
            data = imageTex.GetRawTextureData<byte>()
        };

        ros.Publish(cameraTopic, imageMsg);
        Destroy(imageTex);
    }

    void SimulateIMU()
    {
        // Simulate IMU data based on Unity physics
        Vector3 acceleration = Physics.gravity + GetComponent<Rigidbody>().acceleration;
        Vector3 angularVelocity = GetComponent<Rigidbody>().angularVelocity;

        sensor_msgs.Imu imuMsg = new sensor_msgs.Imu
        {
            header = new std_msgs.Header
            {
                stamp = ROSConnection.GetTime(),
                frame_id = "imu_frame"
            },
            orientation = new geometry_msgs.Quaternion
            {
                x = transform.rotation.x,
                y = transform.rotation.y,
                z = transform.rotation.z,
                w = transform.rotation.w
            },
            angular_velocity = new geometry_msgs.Vector3
            {
                x = angularVelocity.x,
                y = angularVelocity.y,
                z = angularVelocity.z
            },
            linear_acceleration = new geometry_msgs.Vector3
            {
                x = acceleration.x,
                y = acceleration.y,
                z = acceleration.z
            }
        };

        ros.Publish(imuTopic, imuMsg);
    }
}
```

## Sensor Calibration and Validation

Validate your simulated sensors against real-world expectations:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, Imu
from std_msgs.msg import Float64
import numpy as np
from scipy import stats

class SensorValidator(Node):
    def __init__(self):
        super().__init__('sensor_validator')

        # Subscriptions for sensor validation
        self.lidar_sub = self.create_subscription(
            LaserScan, '/humanoid_robot/scan', self.validate_lidar, 10)
        self.imu_sub = self.create_subscription(
            Imu, '/humanoid_robot/imu/data', self.validate_imu, 10)

        # Publishers for validation metrics
        self.lidar_quality_pub = self.create_publisher(
            Float64, '/lidar_quality_score', 10)
        self.imu_stability_pub = self.create_publisher(
            Float64, '/imu_stability_score', 10)

        # Statistics for validation
        self.lidar_ranges_history = []
        self.imu_orientation_history = []

    def validate_lidar(self, msg):
        """Validate LiDAR sensor data quality"""
        ranges = np.array(msg.ranges)
        valid_ranges = ranges[(ranges > msg.range_min) & (ranges < msg.range_max)]

        # Calculate quality metrics
        if len(valid_ranges) > 0:
            # Range consistency (low variance in static environment)
            if len(self.lidar_ranges_history) > 10:
                prev_ranges = self.lidar_ranges_history[-1]
                range_diff = np.mean(np.abs(valid_ranges - prev_ranges))
                quality_score = max(0.0, 1.0 - range_diff)  # Higher score = more consistent
            else:
                quality_score = 1.0

            # Publish quality score
            quality_msg = Float64()
            quality_msg.data = float(quality_score)
            self.lidar_quality_pub.publish(quality_msg)

            # Store for next comparison
            self.lidar_ranges_history.append(valid_ranges.copy())

            self.get_logger().debug(f'LiDAR quality score: {quality_score:.3f}')

    def validate_imu(self, msg):
        """Validate IMU sensor data stability"""
        orientation = np.array([
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        ])

        # Calculate stability metrics
        if len(self.imu_orientation_history) > 0:
            prev_orientation = self.imu_orientation_history[-1]
            orientation_diff = np.mean(np.abs(orientation - prev_orientation))
            stability_score = max(0.0, 1.0 - orientation_diff * 10)  # Normalize difference
        else:
            stability_score = 1.0

        # Publish stability score
        stability_msg = Float64()
        stability_msg.data = float(stability_score)
        self.imu_stability_pub.publish(stability_msg)

        # Store for next comparison
        self.imu_orientation_history.append(orientation.copy())

        self.get_logger().debug(f'IMU stability score: {stability_score:.3f}')

def main(args=None):
    rclpy.init(args=args)
    validator = SensorValidator()

    try:
        rclpy.spin(validator)
    except KeyboardInterrupt:
        pass
    finally:
        validator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Exercise: Configure Sensors for Your Robot

1. Add LiDAR, camera, and IMU sensors to your humanoid robot URDF
2. Configure sensor parameters appropriate for your robot's size and environment
3. Implement a sensor processing node that fuses data from multiple sensors
4. Test sensor simulation in both Gazebo and Unity environments
5. Validate sensor data quality and consistency
6. Create a sensor validation node to monitor sensor health

---
This completes Module 2 of the Physical AI & Humanoid Robotics book.