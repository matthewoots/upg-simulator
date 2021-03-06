//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Mavros
{
    [Serializable]
    public class ESCStatusMsg : Message
    {
        public const string k_RosMessageName = "mavros_msgs/ESCStatus";
        public override string RosMessageName => k_RosMessageName;

        //  ESCStatus.msg
        // 
        // 
        //  See mavlink message documentation here:
        //  https://mavlink.io/en/messages/common.html#ESC_STATUS
        public Std.HeaderMsg header;
        public ESCStatusItemMsg[] esc_status;

        public ESCStatusMsg()
        {
            this.header = new Std.HeaderMsg();
            this.esc_status = new ESCStatusItemMsg[0];
        }

        public ESCStatusMsg(Std.HeaderMsg header, ESCStatusItemMsg[] esc_status)
        {
            this.header = header;
            this.esc_status = esc_status;
        }

        public static ESCStatusMsg Deserialize(MessageDeserializer deserializer) => new ESCStatusMsg(deserializer);

        private ESCStatusMsg(MessageDeserializer deserializer)
        {
            this.header = Std.HeaderMsg.Deserialize(deserializer);
            deserializer.Read(out this.esc_status, ESCStatusItemMsg.Deserialize, deserializer.ReadLength());
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.header);
            serializer.WriteLength(this.esc_status);
            serializer.Write(this.esc_status);
        }

        public override string ToString()
        {
            return "ESCStatusMsg: " +
            "\nheader: " + header.ToString() +
            "\nesc_status: " + System.String.Join(", ", esc_status.ToList());
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
