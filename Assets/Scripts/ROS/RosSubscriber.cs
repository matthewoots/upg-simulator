using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosPose = RosMessageTypes.Geometry.MPoseStamped;
using RosState = RosMessageTypes.Mavros.MState;
using RosVel = RosMessageTypes.Geometry.MTwistStamped;

public class RosSubscriber : MonoBehaviour
{
    public GameObject uav;
    public UnityEngine.Vector3 p;
    private UnityEngine.Vector3 newVector;
    public Quaternion ENURotation;
    public float heightoffset = 10f;

    void Start()
    {
        ROSConnection.instance.Subscribe<RosPose>("mavros/local_position/pose", enuPoseSubscribe);
        ROSConnection.instance.Subscribe<RosVel>("mavros/local_position/velocity_local", GroundSpeedSubscribe);
        ROSConnection.instance.Subscribe<RosState>("mavros/state", StateSubscribe);
        ENURotation = new Quaternion();
    }

    void enuPoseSubscribe(RosPose enuPoseMessage)
    {
        // Debug.Log("");

        // p is in SDE frame hence we have to assign to the correct frame
        p.x = -(float)enuPoseMessage.pose.position.y; 
        p.y = (float)enuPoseMessage.pose.position.z + heightoffset;
        p.z = (float)enuPoseMessage.pose.position.x;

        Quaternion ENUEulerAngles = new Quaternion();
        // Quaternion yaw = new Quaternion();
        // Quaternion otherRotations  = new Quaternion();

        ENURotation.x = (float)enuPoseMessage.pose.orientation.x;
        ENURotation.y = (float)enuPoseMessage.pose.orientation.y;
        ENURotation.z = (float)enuPoseMessage.pose.orientation.z;
        ENURotation.w = (float)enuPoseMessage.pose.orientation.w;

        ENUEulerAngles.eulerAngles = new Vector3(ENURotation.eulerAngles.y, 0, -ENURotation.eulerAngles.x);
        
        // x = pitch (+ve pitch = -ve value), y = yaw (+ve pitch = -ve value), z = roll (+ve pitch = -ve value) (Unity Frame)
        // Realised that ENU is in global frame, not the drone
        uav.transform.localRotation = ENUEulerAngles;
        Debug.Log("ENURotation: " + ENURotation.eulerAngles);
        uav.transform.position = p;
    }

    void StateSubscribe(RosState stateMessage)
    {
        // Debug.Log("");
        // RosState statemsg = uav.GetComponent<TestIgniteGUI>().state;
        // uav.GetComponent<TestIgniteGUI>().state.connected = stateMessage.connected;
        // uav.GetComponent<TestIgniteGUI>().state.armed = stateMessage.armed;
        // uav.GetComponent<TestIgniteGUI>().state.guided = stateMessage.guided;
        // uav.GetComponent<TestIgniteGUI>().state.manual_input = stateMessage.manual_input;
        // uav.GetComponent<TestIgniteGUI>().mode = stateMessage.mode;
        // uav.GetComponent<TestIgniteGUI>().state.system_status = stateMessage.system_status;
    }

    void GroundSpeedSubscribe(RosVel velMessage)
    {
        // Debug.Log("");
        // UnityEngine.Vector3 vel = uav.GetComponent<TestIgniteGUI>().uavVelocity;
        // uav.GetComponent<TestIgniteGUI>().uavVelocity.x = -(float)velMessage.twist.linear.y; 
        // uav.GetComponent<TestIgniteGUI>().uavVelocity.y = (float)velMessage.twist.linear.z;
        // uav.GetComponent<TestIgniteGUI>().uavVelocity.z = (float)velMessage.twist.linear.x;
    }

}
