//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Mavros
{
    [Serializable]
    public class VibrationMsg : Message
    {
        public const string k_RosMessageName = "mavros_msgs/Vibration";
        public override string RosMessageName => k_RosMessageName;

        //  VIBRATION message data
        //  @description: Vibration levels and accelerometer clipping
        public Std.HeaderMsg header;
        public Geometry.Vector3Msg vibration;
        //  3-axis vibration levels
        public float[] clipping;
        //  Accelerometers clipping

        public VibrationMsg()
        {
            this.header = new Std.HeaderMsg();
            this.vibration = new Geometry.Vector3Msg();
            this.clipping = new float[3];
        }

        public VibrationMsg(Std.HeaderMsg header, Geometry.Vector3Msg vibration, float[] clipping)
        {
            this.header = header;
            this.vibration = vibration;
            this.clipping = clipping;
        }

        public static VibrationMsg Deserialize(MessageDeserializer deserializer) => new VibrationMsg(deserializer);

        private VibrationMsg(MessageDeserializer deserializer)
        {
            this.header = Std.HeaderMsg.Deserialize(deserializer);
            this.vibration = Geometry.Vector3Msg.Deserialize(deserializer);
            deserializer.Read(out this.clipping, sizeof(float), 3);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.header);
            serializer.Write(this.vibration);
            serializer.Write(this.clipping);
        }

        public override string ToString()
        {
            return "VibrationMsg: " +
            "\nheader: " + header.ToString() +
            "\nvibration: " + vibration.ToString() +
            "\nclipping: " + System.String.Join(", ", clipping.ToList());
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
