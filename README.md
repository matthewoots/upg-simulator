# UPG-Simulator
Unity3d Environment for UAV simulation, with connection to PX4 and ROS [here](https://github.com/Unity-Technologies/Unity-Robotics-Hub/blob/main/tutorials/ros_unity_integration/setup.md "Unity-Technologies").

Please use this in **conjunction** with https://github.com/matthewoots/ros_tcp_connector and launch the `ROS-TCP-ENDPOINT` node

**[Features]**
- `PX4-UNITY-SIMULATOR` with PX4 SITL connection via TCP port
- `MAVROS` messages supported via subscriber class
- Undergoing testing for `point2` message to show global obstacle map
- `TinyWhoop` and `Dji Fpv` Models from `Sketchfab`

---

## Platform
**[HARDWARE]**
- Ubuntu 18.04
- Intel® Xeon(R) CPU E3-1535M v6 @ 3.10GHz × 8 
- NVIDIA Quadro M2200

**[SOFTWARE]**
- Unity 2020.3.6f1

**[NOT SUPPORTED ON UNITY-UBUNTU]**
- [HDRP] is not supported
- [Substance engine] is not supported

---

## ROS-Unity Setup

This is using `unity_robotics_demo` under the `ROS–Unity-Integration` tutorials, use this as a template to generate different message types

1. Open Package Manager and click the + button at the top left corner. Select "add package from git URL" and enter `https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector` to install the ROS-TCP-Connector package.
2. In the Unity menu bar, go to `Robotics -> Generate ROS Messages`.... In the `ROS Message Browser` window, click the `Browse` button at the top right to set the ROS message path to `tutorials/ros_unity_integration/ros_packages/unity_robotics_demo_msgs` in this repo. **(This applies to any message types)**
3. In the message browser, expand the `unity_robotics_demo_msgs subfolder` and click `Build 2 msgs` and `Build 2 srvs` to generate C# scripts from the ROS .msg and .srv files.

- Current message types already include `MAVROS`, `GEOMETRIC`, `GEOGRAPHIC`, `SENSOR` and `STD`.

