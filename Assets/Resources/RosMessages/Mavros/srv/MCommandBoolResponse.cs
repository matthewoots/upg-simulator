//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Mavros
{
    public class MCommandBoolResponse : Message
    {
        public const string RosMessageName = "mavros_msgs/CommandBool";

        public bool success;
        public byte result;

        public MCommandBoolResponse()
        {
            this.success = false;
            this.result = 0;
        }

        public MCommandBoolResponse(bool success, byte result)
        {
            this.success = success;
            this.result = result;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.Add(BitConverter.GetBytes(this.success));
            listOfSerializations.Add(new[]{this.result});

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            this.success = BitConverter.ToBoolean(data, offset);
            offset += 1;
            this.result = data[offset];;
            offset += 1;

            return offset;
        }

        public override string ToString()
        {
            return "MCommandBoolResponse: " +
            "\nsuccess: " + success.ToString() +
            "\nresult: " + result.ToString();
        }
    }
}
