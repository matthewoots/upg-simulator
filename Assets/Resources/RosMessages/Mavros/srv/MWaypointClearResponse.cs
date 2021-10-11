//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Mavros
{
    public class MWaypointClearResponse : Message
    {
        public const string RosMessageName = "mavros_msgs/WaypointClear";

        public bool success;

        public MWaypointClearResponse()
        {
            this.success = false;
        }

        public MWaypointClearResponse(bool success)
        {
            this.success = success;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.Add(BitConverter.GetBytes(this.success));

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            this.success = BitConverter.ToBoolean(data, offset);
            offset += 1;

            return offset;
        }

        public override string ToString()
        {
            return "MWaypointClearResponse: " +
            "\nsuccess: " + success.ToString();
        }
    }
}
