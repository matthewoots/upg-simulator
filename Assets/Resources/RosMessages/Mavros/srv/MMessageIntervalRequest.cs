//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Mavros
{
    public class MMessageIntervalRequest : Message
    {
        public const string RosMessageName = "mavros_msgs/MessageInterval";

        //  sets message interval
        //  See MAV_CMD_SET_MESSAGE_INTERVAL
        public uint message_id;
        public float message_rate;

        public MMessageIntervalRequest()
        {
            this.message_id = 0;
            this.message_rate = 0.0f;
        }

        public MMessageIntervalRequest(uint message_id, float message_rate)
        {
            this.message_id = message_id;
            this.message_rate = message_rate;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.Add(BitConverter.GetBytes(this.message_id));
            listOfSerializations.Add(BitConverter.GetBytes(this.message_rate));

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            this.message_id = BitConverter.ToUInt32(data, offset);
            offset += 4;
            this.message_rate = BitConverter.ToSingle(data, offset);
            offset += 4;

            return offset;
        }

        public override string ToString()
        {
            return "MMessageIntervalRequest: " +
            "\nmessage_id: " + message_id.ToString() +
            "\nmessage_rate: " + message_rate.ToString();
        }
    }
}