---
sidebar_position: 2
---

# Chapter 2: Unity Integration

## Introduction to Unity for Robotics

Unity provides high-fidelity visualization and simulation capabilities for robotics applications. Unlike Gazebo which focuses on physics simulation, Unity excels at creating photorealistic environments and complex visualizations. For humanoid robotics, Unity can complement Gazebo by providing more realistic rendering and user interfaces.

## Setting up Unity for Robotics

### Installing Unity Hub and Unity Editor

1. Download Unity Hub from the [Unity website](https://unity.com/download)
2. Install Unity Hub and log in with your Unity ID
3. Through Unity Hub, install Unity Editor version 2022.3 LTS (recommended for robotics projects)

### Unity Robotics Package Installation

Unity provides specific packages for robotics integration:

```bash
# In Unity Package Manager, install:
- ROS-TCP-Connector
- Unity-Robotics-Helpers
- URDF-Importer
```

Or add them via the Package Manager window in Unity:
1. Go to Window → Package Manager
2. Click the + button → Add package from git URL
3. Add: `com.unity.robotics.ros-tcp-connector` and `com.unity.robotics.urdf-importer`

## URDF Importer for Humanoid Robots

Unity's URDF Importer allows you to import your ROS 2 robot models directly:

```csharp
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Std;
using Unity.Robotics.URDFImporter;
using UnityEngine;

public class HumanoidRobotController : MonoBehaviour
{
    [SerializeField] private string robotDescriptionTopic = "robot_description";
    [SerializeField] private string jointStatesTopic = "joint_states";

    private ROSConnection ros;
    private ArticulationBody[] joints;

    void Start()
    {
        ros = ROSConnection.instance;

        // Subscribe to joint states
        ros.Subscribe<sensor_msgs.JointState>(jointStatesTopic, OnJointStateReceived);
    }

    void OnJointStateReceived(sensor_msgs.JointState jointState)
    {
        // Update robot joints based on ROS joint states
        for (int i = 0; i < jointState.name.Count; i++)
        {
            string jointName = jointState.name[i];
            float position = (float)jointState.position[i];

            // Find and update the corresponding joint in Unity
            UpdateJoint(jointName, position);
        }
    }

    void UpdateJoint(string jointName, float position)
    {
        // Find the joint in the Unity hierarchy and update its position
        Transform jointTransform = FindJointByName(jointName);
        if (jointTransform != null)
        {
            ArticulationBody body = jointTransform.GetComponent<ArticulationBody>();
            if (body != null)
            {
                ArticulationDrive drive = body.xDrive;
                drive.target = position;
                body.xDrive = drive;
            }
        }
    }

    Transform FindJointByName(string name)
    {
        Transform[] allTransforms = GetComponentsInChildren<Transform>();
        foreach (Transform t in allTransforms)
        {
            if (t.name == name)
                return t;
        }
        return null;
    }
}
```

## ROS-TCP-Connector Setup

Set up the ROS-TCP connection between Unity and ROS 2:

```csharp
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Sensor;

public class UnityRobotBridge : MonoBehaviour
{
    [Header("ROS Connection Settings")]
    [SerializeField] private string rosIP = "127.0.0.1";
    [SerializeField] private int rosPort = 10000;

    [Header("Robot Topics")]
    [SerializeField] private string cameraTopic = "/unity_camera/image_raw";
    [SerializeField] private string depthTopic = "/unity_depth/image_raw";

    private ROSConnection ros;
    private Camera unityCamera;
    private RenderTexture renderTexture;

    void Start()
    {
        // Initialize ROS connection
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<sensor_msgs.Image>(cameraTopic);
        ros.RegisterPublisher<sensor_msgs.Image>(depthTopic);

        unityCamera = GetComponent<Camera>();

        // Set up render texture for camera feed
        renderTexture = new RenderTexture(640, 480, 24);
        unityCamera.targetTexture = renderTexture;
    }

    void Update()
    {
        // Capture and send camera feed to ROS
        if (Time.frameCount % 30 == 0) // Send every 30 frames
        {
            SendCameraFeed();
        }
    }

    void SendCameraFeed()
    {
        RenderTexture.active = renderTexture;
        Texture2D imageTex = new Texture2D(renderTexture.width, renderTexture.height, TextureFormat.RGB24, false);
        imageTex.ReadPixels(new Rect(0, 0, renderTexture.width, renderTexture.height), 0, 0);
        imageTex.Apply();

        // Convert to ROS image format and publish
        sensor_msgs.Image rosImage = new sensor_msgs.Image
        {
            header = new std_msgs.Header { frame_id = "unity_camera" },
            height = (uint)imageTex.height,
            width = (uint)imageTex.width,
            encoding = "rgb8",
            is_bigendian = 0,
            step = (uint)(imageTex.width * 3),
            data = imageTex.GetRawTextureData<byte>()
        };

        ros.Publish(cameraTopic, rosImage);
        Destroy(imageTex);
    }
}
```

## Creating Digital Twin Environments

Unity excels at creating detailed digital twin environments. Here's how to set up a humanoid robot testing environment:

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;

public class DigitalTwinEnvironment : MonoBehaviour
{
    [Header("Environment Settings")]
    [SerializeField] private Transform robotSpawnPoint;
    [SerializeField] private GameObject[] obstacles;
    [SerializeField] private Light[] environmentLights;

    [Header("Simulation Parameters")]
    [SerializeField] private float gravityScale = 1.0f;
    [SerializeField] private bool enablePhysics = true;

    private ROSConnection ros;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();

        // Configure physics
        Physics.gravity = new Vector3(0, -9.81f * gravityScale, 0);

        // Initialize environment
        SetupEnvironment();

        // Send environment information to ROS
        SendEnvironmentInfo();
    }

    void SetupEnvironment()
    {
        // Place obstacles randomly
        foreach (GameObject obstacle in obstacles)
        {
            Vector3 randomPosition = new Vector3(
                Random.Range(-5f, 5f),
                0.5f, // Half the obstacle height
                Random.Range(-5f, 5f)
            );
            obstacle.transform.position = randomPosition;
        }
    }

    void SendEnvironmentInfo()
    {
        // Send environment configuration to ROS
        // This could include obstacle positions, lighting conditions, etc.
        Debug.Log("Environment information sent to ROS");
    }

    void OnValidate()
    {
        // Validate settings in editor
        if (gravityScale <= 0)
            gravityScale = 1.0f;
    }
}
```

## Unity-ROS Bridge for Humanoid Control

Create a comprehensive control interface for your humanoid robot:

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Geometry;

public class HumanoidControlInterface : MonoBehaviour
{
    [Header("Control Topics")]
    [SerializeField] private string cmdVelTopic = "/cmd_vel";
    [SerializeField] private string jointCmdTopic = "/joint_group_position_controller/commands";

    [Header("Humanoid Parameters")]
    [SerializeField] private float walkSpeed = 1.0f;
    [SerializeField] private float turnSpeed = 1.0f;
    [SerializeField] private float armSpeed = 0.5f;

    private ROSConnection ros;
    private Animator animator;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<geometry_msgs.Twist>(cmdVelTopic);
        ros.RegisterPublisher<sensor_msgs.JointState>(jointCmdTopic);

        animator = GetComponent<Animator>();
    }

    public void SendVelocityCommand(float linearX, float angularZ)
    {
        geometry_msgs.Twist cmd = new geometry_msgs.Twist
        {
            linear = new geometry_msgs.Vector3 { x = linearX, y = 0, z = 0 },
            angular = new geometry_msgs.Vector3 { x = 0, y = 0, z = angularZ }
        };

        ros.Publish(cmdVelTopic, cmd);
    }

    public void SendJointCommands(float[] jointPositions, string[] jointNames)
    {
        sensor_msgs.JointState jointCmd = new sensor_msgs.JointState
        {
            name = jointNames,
            position = jointPositions,
            header = new std_msgs.Header { stamp = ROSConnection.GetTime() }
        };

        ros.Publish(jointCmdTopic, jointCmd);
    }

    // Example: Handle user input for humanoid control
    void Update()
    {
        // Example: Keyboard input for testing
        float moveX = Input.GetAxis("Vertical") * walkSpeed;
        float turnY = Input.GetAxis("Horizontal") * turnSpeed;

        if (Input.anyKey)
        {
            SendVelocityCommand(moveX, turnY);
        }
    }
}
```

## Advanced Visualization Features

Unity's advanced rendering capabilities can be used for robot visualization:

```csharp
using UnityEngine;
using UnityEngine.Rendering;

public class AdvancedRobotVisualization : MonoBehaviour
{
    [Header("Visualization Settings")]
    [SerializeField] private Material robotMaterial;
    [SerializeField] private Shader outlineShader;
    [SerializeField] private Color selectedColor = Color.yellow;

    [Header("Debug Visualization")]
    [SerializeField] private bool showTrajectory = true;
    [SerializeField] private bool showSensorRange = true;
    [SerializeField] private float sensorRange = 5.0f;

    private LineRenderer trajectoryRenderer;
    private Vector3[] trajectoryPoints;
    private int trajectoryIndex = 0;

    void Start()
    {
        SetupTrajectoryVisualization();
    }

    void SetupTrajectoryVisualization()
    {
        trajectoryRenderer = gameObject.AddComponent<LineRenderer>();
        trajectoryRenderer.material = new Material(Shader.Find("Sprites/Default"));
        trajectoryRenderer.widthMultiplier = 0.1f;
        trajectoryRenderer.positionCount = 100;
        trajectoryPoints = new Vector3[100];
    }

    void Update()
    {
        if (showTrajectory)
        {
            UpdateTrajectory();
        }

        if (showSensorRange)
        {
            DrawSensorRange();
        }
    }

    void UpdateTrajectory()
    {
        trajectoryPoints[trajectoryIndex] = transform.position;
        trajectoryIndex = (trajectoryIndex + 1) % trajectoryPoints.Length;

        trajectoryRenderer.positionCount = trajectoryPoints.Length;
        for (int i = 0; i < trajectoryPoints.Length; i++)
        {
            int index = (trajectoryIndex + i) % trajectoryPoints.Length;
            trajectoryRenderer.SetPosition(i, trajectoryPoints[i]);
        }
    }

    void DrawSensorRange()
    {
        // Visualize sensor range as a wire sphere
        Gizmos.color = Color.red;
        Gizmos.DrawWireSphere(transform.position, sensorRange);
    }

    void OnDrawGizmos()
    {
        if (showSensorRange)
        {
            Gizmos.color = new Color(1, 0, 0, 0.3f);
            Gizmos.DrawSphere(transform.position, sensorRange);
        }
    }
}
```

## Exercise: Integrate Unity with Your Robot

1. Install Unity and the robotics packages
2. Import your humanoid robot URDF model into Unity
3. Set up ROS-TCP-Connector for communication with ROS 2
4. Create a digital twin environment with realistic lighting and obstacles
5. Implement bidirectional communication between Unity and ROS 2
6. Test the integration by controlling your robot from both Unity and ROS 2

---
This chapter covered Unity integration. In the next chapter, we'll explore sensor simulation.