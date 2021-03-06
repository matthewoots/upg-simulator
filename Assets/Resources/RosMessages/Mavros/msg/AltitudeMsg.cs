//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Mavros
{
    [Serializable]
    public class AltitudeMsg : Message
    {
        public const string k_RosMessageName = "mavros_msgs/Altitude";
        public override string RosMessageName => k_RosMessageName;

        //  Altitude information
        // 
        //  https://mavlink.io/en/messages/common.html#ALTITUDE
        public Std.HeaderMsg header;
        public float monotonic;
        public float amsl;
        public float local;
        public float relative;
        public float terrain;
        public float bottom_clearance;

        public AltitudeMsg()
        {
            this.header = new Std.HeaderMsg();
            this.monotonic = 0.0f;
            this.amsl = 0.0f;
            this.local = 0.0f;
            this.relative = 0.0f;
            this.terrain = 0.0f;
            this.bottom_clearance = 0.0f;
        }

        public AltitudeMsg(Std.HeaderMsg header, float monotonic, float amsl, float local, float relative, float terrain, float bottom_clearance)
        {
            this.header = header;
            this.monotonic = monotonic;
            this.amsl = amsl;
            this.local = local;
            this.relative = relative;
            this.terrain = terrain;
            this.bottom_clearance = bottom_clearance;
        }

        public static AltitudeMsg Deserialize(MessageDeserializer deserializer) => new AltitudeMsg(deserializer);

        private AltitudeMsg(MessageDeserializer deserializer)
        {
            this.header = Std.HeaderMsg.Deserialize(deserializer);
            deserializer.Read(out this.monotonic);
            deserializer.Read(out this.amsl);
            deserializer.Read(out this.local);
            deserializer.Read(out this.relative);
            deserializer.Read(out this.terrain);
            deserializer.Read(out this.bottom_clearance);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.header);
            serializer.Write(this.monotonic);
            serializer.Write(this.amsl);
            serializer.Write(this.local);
            serializer.Write(this.relative);
            serializer.Write(this.terrain);
            serializer.Write(this.bottom_clearance);
        }

        public override string ToString()
        {
            return "AltitudeMsg: " +
            "\nheader: " + header.ToString() +
            "\nmonotonic: " + monotonic.ToString() +
            "\namsl: " + amsl.ToString() +
            "\nlocal: " + local.ToString() +
            "\nrelative: " + relative.ToString() +
            "\nterrain: " + terrain.ToString() +
            "\nbottom_clearance: " + bottom_clearance.ToString();
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
