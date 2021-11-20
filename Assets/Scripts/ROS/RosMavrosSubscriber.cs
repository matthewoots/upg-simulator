using UnityEngine;
using System.Collections;
using Unity.Robotics.ROSTCPConnector;

using RosBattMsg = RosMessageTypes.Sensor.BatteryStateMsg;
using RosPoseMsg = RosMessageTypes.Geometry.PoseStampedMsg;
using RosVelMsg = RosMessageTypes.Geometry.TwistStampedMsg;
using RosStateMsg = RosMessageTypes.Mavros.StateMsg;
using RosGlobalPosMsg = RosMessageTypes.Sensor.NavSatFixMsg;
using RosActuatorControlMsg = RosMessageTypes.Mavros.ActuatorControlMsg;

public class RosMavrosSubscriber : MonoBehaviour
{  
    public GameObject obj;
    public float RosBattHz = 15.0f; public float RosPoseHz = 15.0f; public float RosVelHz = 15.0f; 
    public float RosStateHz = 15.0f; public float RosGlobalHz = 15.0f;
    private float BattTimer = 0; private float PoseTimer = 0; private float VelTimer = 0; 
    private float StateTimer = 0; private float GlobalTimer = 0;

    void Start()
    {
        ROSConnection.instance.Subscribe<RosBattMsg>("mavros/battery", UavBattSubscriber);
        ROSConnection.instance.Subscribe<RosPoseMsg>("mavros/local_position/pose", UavEnuPoseSubscriber);
        ROSConnection.instance.Subscribe<RosVelMsg>("mavros/local_position/velocity_local", UavEnuVelSubscriber);
        ROSConnection.instance.Subscribe<RosStateMsg>("mavros/state", UavStateSubscriber);
        ROSConnection.instance.Subscribe<RosGlobalPosMsg>("mavros/global_position/global", UavGlobalPosSubscriber);
    }

    void UavBattSubscriber(RosBattMsg uavBattMessage)
    {
        BattTimer += Time.deltaTime;

        if (BattTimer > 1/RosBattHz)
            BattTimer = 0;
        else
            return;
        
        float[] cell_voltage; float voltage; float percentage; float current;
        cell_voltage = uavBattMessage.cell_voltage; 
        // voltage = uavBattMessage.voltage; 
        // percentage = uavBattMessage.percentage; 
        // current = uavBattMessage.current; 
    }

    void UavEnuPoseSubscriber(RosPoseMsg enuPoseMessage)
    {
        PoseTimer += Time.deltaTime;

        if (PoseTimer > 1/RosPoseHz)
            PoseTimer = 0;
        else
            return;

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

        // local_pos = pos;
        // local_rot = enuRotation.eulerAngles;

        obj.transform.position = pos;
        obj.transform.localRotation = unityEulerAngles;
    }

    void UavEnuVelSubscriber(RosVelMsg uavVelMessage)
    {
        VelTimer += Time.deltaTime;

        if (VelTimer > 1/RosVelHz)
            VelTimer = 0;
        else
            return;

        // local_vel = new Vector3((float)uavVelMessage.twist.linear.y,
            // (float)uavVelMessage.twist.linear.z, 
            // -(float)uavVelMessage.twist.linear.x);
    }

    void UavStateSubscriber(RosStateMsg uavStateMessage)
    {
        StateTimer += Time.deltaTime;

        if (StateTimer > 1/RosStateHz)
            StateTimer = 0;
        else
            return;

        // connected = uavStateMessage.connected;
        // armed = uavStateMessage.armed;
        // guided = uavStateMessage.guided;
        // manual_input = uavStateMessage.manual_input;
        // mode = uavStateMessage.mode;
        // system_status = uavStateMessage.system_status;
    }

    void UavGlobalPosSubscriber(RosGlobalPosMsg uavGlobalMessage)
    {
        GlobalTimer += Time.deltaTime;

        if (GlobalTimer > 1/RosGlobalHz)
            GlobalTimer = 0;
        else
            return;

        // latitude = uavGlobalMessage.latitude;
        // longitude = uavGlobalMessage.longitude;
        // altitude = uavGlobalMessage.altitude;
    }

}
