//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Mavros
{
    [Serializable]
    public class StateMsg : Message
    {
        public const string k_RosMessageName = "mavros_msgs/State";
        public override string RosMessageName => k_RosMessageName;

        //  Current autopilot state
        // 
        //  Known modes listed here:
        //  http://wiki.ros.org/mavros/CustomModes
        // 
        //  For system_status values
        //  see https://mavlink.io/en/messages/common.html#MAV_STATE
        // 
        public Std.HeaderMsg header;
        public bool connected;
        public bool armed;
        public bool guided;
        public bool manual_input;
        public string mode;
        public byte system_status;
        public const string MODE_APM_PLANE_MANUAL = "MANUAL";
        public const string MODE_APM_PLANE_CIRCLE = "CIRCLE";
        public const string MODE_APM_PLANE_STABILIZE = "STABILIZE";
        public const string MODE_APM_PLANE_TRAINING = "TRAINING";
        public const string MODE_APM_PLANE_ACRO = "ACRO";
        public const string MODE_APM_PLANE_FBWA = "FBWA";
        public const string MODE_APM_PLANE_FBWB = "FBWB";
        public const string MODE_APM_PLANE_CRUISE = "CRUISE";
        public const string MODE_APM_PLANE_AUTOTUNE = "AUTOTUNE";
        public const string MODE_APM_PLANE_AUTO = "AUTO";
        public const string MODE_APM_PLANE_RTL = "RTL";
        public const string MODE_APM_PLANE_LOITER = "LOITER";
        public const string MODE_APM_PLANE_LAND = "LAND";
        public const string MODE_APM_PLANE_GUIDED = "GUIDED";
        public const string MODE_APM_PLANE_INITIALISING = "INITIALISING";
        public const string MODE_APM_PLANE_QSTABILIZE = "QSTABILIZE";
        public const string MODE_APM_PLANE_QHOVER = "QHOVER";
        public const string MODE_APM_PLANE_QLOITER = "QLOITER";
        public const string MODE_APM_PLANE_QLAND = "QLAND";
        public const string MODE_APM_PLANE_QRTL = "QRTL";
        public const string MODE_APM_COPTER_STABILIZE = "STABILIZE";
        public const string MODE_APM_COPTER_ACRO = "ACRO";
        public const string MODE_APM_COPTER_ALT_HOLD = "ALT_HOLD";
        public const string MODE_APM_COPTER_AUTO = "AUTO";
        public const string MODE_APM_COPTER_GUIDED = "GUIDED";
        public const string MODE_APM_COPTER_LOITER = "LOITER";
        public const string MODE_APM_COPTER_RTL = "RTL";
        public const string MODE_APM_COPTER_CIRCLE = "CIRCLE";
        public const string MODE_APM_COPTER_POSITION = "POSITION";
        public const string MODE_APM_COPTER_LAND = "LAND";
        public const string MODE_APM_COPTER_OF_LOITER = "OF_LOITER";
        public const string MODE_APM_COPTER_DRIFT = "DRIFT";
        public const string MODE_APM_COPTER_SPORT = "SPORT";
        public const string MODE_APM_COPTER_FLIP = "FLIP";
        public const string MODE_APM_COPTER_AUTOTUNE = "AUTOTUNE";
        public const string MODE_APM_COPTER_POSHOLD = "POSHOLD";
        public const string MODE_APM_COPTER_BRAKE = "BRAKE";
        public const string MODE_APM_COPTER_THROW = "THROW";
        public const string MODE_APM_COPTER_AVOID_ADSB = "AVOID_ADSB";
        public const string MODE_APM_COPTER_GUIDED_NOGPS = "GUIDED_NOGPS";
        public const string MODE_APM_ROVER_MANUAL = "MANUAL";
        public const string MODE_APM_ROVER_LEARNING = "LEARNING";
        public const string MODE_APM_ROVER_STEERING = "STEERING";
        public const string MODE_APM_ROVER_HOLD = "HOLD";
        public const string MODE_APM_ROVER_AUTO = "AUTO";
        public const string MODE_APM_ROVER_RTL = "RTL";
        public const string MODE_APM_ROVER_GUIDED = "GUIDED";
        public const string MODE_APM_ROVER_INITIALISING = "INITIALISING";
        public const string MODE_PX4_MANUAL = "MANUAL";
        public const string MODE_PX4_ACRO = "ACRO";
        public const string MODE_PX4_ALTITUDE = "ALTCTL";
        public const string MODE_PX4_POSITION = "POSCTL";
        public const string MODE_PX4_OFFBOARD = "OFFBOARD";
        public const string MODE_PX4_STABILIZED = "STABILIZED";
        public const string MODE_PX4_RATTITUDE = "RATTITUDE";
        public const string MODE_PX4_MISSION = "AUTO.MISSION";
        public const string MODE_PX4_LOITER = "AUTO.LOITER";
        public const string MODE_PX4_RTL = "AUTO.RTL";
        public const string MODE_PX4_LAND = "AUTO.LAND";
        public const string MODE_PX4_RTGS = "AUTO.RTGS";
        public const string MODE_PX4_READY = "AUTO.READY";
        public const string MODE_PX4_TAKEOFF = "AUTO.TAKEOFF";

        public StateMsg()
        {
            this.header = new Std.HeaderMsg();
            this.connected = false;
            this.armed = false;
            this.guided = false;
            this.manual_input = false;
            this.mode = "";
            this.system_status = 0;
        }

        public StateMsg(Std.HeaderMsg header, bool connected, bool armed, bool guided, bool manual_input, string mode, byte system_status)
        {
            this.header = header;
            this.connected = connected;
            this.armed = armed;
            this.guided = guided;
            this.manual_input = manual_input;
            this.mode = mode;
            this.system_status = system_status;
        }

        public static StateMsg Deserialize(MessageDeserializer deserializer) => new StateMsg(deserializer);

        private StateMsg(MessageDeserializer deserializer)
        {
            this.header = Std.HeaderMsg.Deserialize(deserializer);
            deserializer.Read(out this.connected);
            deserializer.Read(out this.armed);
            deserializer.Read(out this.guided);
            deserializer.Read(out this.manual_input);
            deserializer.Read(out this.mode);
            deserializer.Read(out this.system_status);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.header);
            serializer.Write(this.connected);
            serializer.Write(this.armed);
            serializer.Write(this.guided);
            serializer.Write(this.manual_input);
            serializer.Write(this.mode);
            serializer.Write(this.system_status);
        }

        public override string ToString()
        {
            return "StateMsg: " +
            "\nheader: " + header.ToString() +
            "\nconnected: " + connected.ToString() +
            "\narmed: " + armed.ToString() +
            "\nguided: " + guided.ToString() +
            "\nmanual_input: " + manual_input.ToString() +
            "\nmode: " + mode.ToString() +
            "\nsystem_status: " + system_status.ToString();
        }

#if UNITY_EDITOR
        [UnityEditor.InitializeOnLoadMethod]
#else
        [UnityEngine.RuntimeInitializeOnLoadMethod]
#endif
        public static void Register()
        {
            MessageRegistry.Register(k_RosMessageName, Deserialize);
        }
    }
}
