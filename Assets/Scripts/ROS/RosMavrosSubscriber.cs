// Attach Mavros Subscriber to the UAV Model

using UnityEngine;
using System.Collections;
using Unity.Robotics.ROSTCPConnector;

using RosBattMsg = RosMessageTypes.Sensor.MBatteryState;
using RosPoseMsg = RosMessageTypes.Geometry.MPoseStamped;
using RosVelMsg = RosMessageTypes.Geometry.MTwistStamped;
using RosStateMsg = RosMessageTypes.Mavros.MState;
using RosGlobalPosMsg = RosMessageTypes.Sensor.MNavSatFix;
using RosActuatorControlMsg = RosMessageTypes.Mavros.MActuatorControl;

public class RosMavrosSubscriber : MonoBehaviour
{
    public GameObject uav;
    RosMessageBus bus;
    public int uavid;   
    public float RosBattRate = 0.001f; 
    public float RosPoseRate = 0.001f; 
    public float RosVelRate = 0.001f; 
    public float RosStateRate = 0.001f; 
    public float RosGloballRate = 0.001f;
    private float lastBatttime; private float lastPosetime; private float lastVeltime; 
    private float lastStatetime; private float lastGlobaltime;

    void Start()
    {
        ROSConnection.instance.Subscribe<RosBattMsg>("mavros/battery", UavBattSubscriber);
        ROSConnection.instance.Subscribe<RosPoseMsg>("mavros/local_position/pose", UavEnuPoseSubscriber);
        ROSConnection.instance.Subscribe<RosVelMsg>("mavros/local_position/velocity_local", UavEnuVelSubscriber);
        ROSConnection.instance.Subscribe<RosStateMsg>("mavros/state", UavStateSubscriber);
        ROSConnection.instance.Subscribe<RosGlobalPosMsg>("mavros/global_position/local", UavGlobalPosSubscriber);

        bus = uav.GetComponent<RosMessageBus>();
    }

    void UavBattSubscriber(RosBattMsg uavBattMessage)
    {
        // Debug.Log("");
        if (Time.time - lastBatttime < RosBattRate)
        {
            return;
        }
        float[] cell_voltage; float voltage; float percentage; float current;
        cell_voltage = uavBattMessage.cell_voltage; 
        bus.voltage = uavBattMessage.voltage; 
        bus.percentage = uavBattMessage.percentage; 
        bus.current = uavBattMessage.current; 

        lastBatttime = Time.time;
    }

    void UavEnuPoseSubscriber(RosPoseMsg enuPoseMessage)
    {
        // Debug.Log("");
        if (Time.time - lastPosetime < RosPoseRate)
        {
            return;
        }

        // UNITY is in NUW while MAVROS is in ENU
        // UNITY does not rotate relative to its frame, and rotation is absolute

        UnityEngine.Vector3 pos; 
        pos.x = (float)enuPoseMessage.pose.position.y; 
        pos.y = (float)enuPoseMessage.pose.position.z;
        pos.z = -(float)enuPoseMessage.pose.position.x;

        Quaternion enuRotation = new Quaternion();
        Quaternion unityEulerAngles = new Quaternion();

        enuRotation.x = (float)enuPoseMessage.pose.orientation.x;
        enuRotation.y = (float)enuPoseMessage.pose.orientation.y;
        enuRotation.z = (float)enuPoseMessage.pose.orientation.z;
        enuRotation.w = (float)enuPoseMessage.pose.orientation.w;
        
        // Realised that ENU is in global frame, not the drone
        unityEulerAngles.eulerAngles = new Vector3(enuRotation.eulerAngles.y, 
            enuRotation.eulerAngles.z, 
            -enuRotation.eulerAngles.x);        
        // Debug.Log("unityEulerAngles: " + uav.transform.localRotation);
        Debug.Log("unityLocalPos: " + pos);

        bus.local_pos = pos;
        bus.local_rot = enuRotation.eulerAngles;

        uav.transform.position = pos;
        uav.transform.localRotation = unityEulerAngles;

        lastPosetime = Time.time;
    }

    void UavEnuVelSubscriber(RosVelMsg uavVelMessage)
    {
        // Debug.Log("");
        if (Time.time - lastVeltime < RosVelRate)
        {
            return;
        }

        float vel_x; float vel_y; float vel_z;
        bus.local_vel = new Vector3((float)uavVelMessage.twist.linear.y,
            (float)uavVelMessage.twist.linear.z, 
            -(float)uavVelMessage.twist.linear.x);

        lastVeltime = Time.time;
    }

    void UavStateSubscriber(RosStateMsg uavStateMessage)
    {
        // Debug.Log("");
        if (Time.time - lastStatetime < RosStateRate)
        {
            return;
        }

        bool guided; bool manual_input; byte system_status;
        bus.connected = uavStateMessage.connected;
        bus.armed = uavStateMessage.armed;
        guided = uavStateMessage.guided;
        manual_input = uavStateMessage.manual_input;
        bus.mode = uavStateMessage.mode;
        system_status = uavStateMessage.system_status;

        lastStatetime = Time.time;
    }

    void UavGlobalPosSubscriber(RosGlobalPosMsg uavGlobalMessage)
    {
        // Debug.Log("");
        if (Time.time - lastGlobaltime < RosGloballRate)
        {
            return;
        }

        bus.latitude = uavGlobalMessage.latitude;
        bus.longitude = uavGlobalMessage.longitude;
        bus.altitude = uavGlobalMessage.altitude;

        lastGlobaltime = Time.time;
    }

}
