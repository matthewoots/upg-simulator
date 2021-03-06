//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Mavros
{
    [Serializable]
    public class WaypointListMsg : Message
    {
        public const string k_RosMessageName = "mavros_msgs/WaypointList";
        public override string RosMessageName => k_RosMessageName;

        //  WaypointList.msg
        // 
        //   :current_seq:   seq nr of currently active waypoint
        //                   waypoints[current_seq].is_current == True
        // 
        //   :waypoints:     list of waypoints
        public ushort current_seq;
        public WaypointMsg[] waypoints;

        public WaypointListMsg()
        {
            this.current_seq = 0;
            this.waypoints = new WaypointMsg[0];
        }

        public WaypointListMsg(ushort current_seq, WaypointMsg[] waypoints)
        {
            this.current_seq = current_seq;
            this.waypoints = waypoints;
        }

        public static WaypointListMsg Deserialize(MessageDeserializer deserializer) => new WaypointListMsg(deserializer);

        private WaypointListMsg(MessageDeserializer deserializer)
        {
            deserializer.Read(out this.current_seq);
            deserializer.Read(out this.waypoints, WaypointMsg.Deserialize, deserializer.ReadLength());
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.current_seq);
            serializer.WriteLength(this.waypoints);
            serializer.Write(this.waypoints);
        }

        public override string ToString()
        {
            return "WaypointListMsg: " +
            "\ncurrent_seq: " + current_seq.ToString() +
            "\nwaypoints: " + System.String.Join(", ", waypoints.ToList());
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
