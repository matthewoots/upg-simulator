//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Std
{
    [Serializable]
    public class UInt32MultiArrayMsg : Message
    {
        public const string k_RosMessageName = "std_msgs/UInt32MultiArray";
        public override string RosMessageName => k_RosMessageName;

        //  Please look at the MultiArrayLayout message definition for
        //  documentation on all multiarrays.
        public MultiArrayLayoutMsg layout;
        //  specification of data layout
        public uint[] data;
        //  array of data

        public UInt32MultiArrayMsg()
        {
            this.layout = new MultiArrayLayoutMsg();
            this.data = new uint[0];
        }

        public UInt32MultiArrayMsg(MultiArrayLayoutMsg layout, uint[] data)
        {
            this.layout = layout;
            this.data = data;
        }

        public static UInt32MultiArrayMsg Deserialize(MessageDeserializer deserializer) => new UInt32MultiArrayMsg(deserializer);

        private UInt32MultiArrayMsg(MessageDeserializer deserializer)
        {
            this.layout = MultiArrayLayoutMsg.Deserialize(deserializer);
            deserializer.Read(out this.data, sizeof(uint), deserializer.ReadLength());
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.layout);
            serializer.WriteLength(this.data);
            serializer.Write(this.data);
        }

        public override string ToString()
        {
            return "UInt32MultiArrayMsg: " +
            "\nlayout: " + layout.ToString() +
            "\ndata: " + System.String.Join(", ", data.ToList());
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
