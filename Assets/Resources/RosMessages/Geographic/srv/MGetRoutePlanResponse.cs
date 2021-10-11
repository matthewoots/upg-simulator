//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Geographic
{
    public class MGetRoutePlanResponse : Message
    {
        public const string RosMessageName = "geographic_msgs/GetRoutePlan";

        public bool success;
        //  true if the call succeeded
        public string status;
        //  more details
        public MRoutePath plan;
        //  path to follow

        public MGetRoutePlanResponse()
        {
            this.success = false;
            this.status = "";
            this.plan = new MRoutePath();
        }

        public MGetRoutePlanResponse(bool success, string status, MRoutePath plan)
        {
            this.success = success;
            this.status = status;
            this.plan = plan;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.Add(BitConverter.GetBytes(this.success));
            listOfSerializations.Add(SerializeString(this.status));
            listOfSerializations.AddRange(plan.SerializationStatements());

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            this.success = BitConverter.ToBoolean(data, offset);
            offset += 1;
            var statusStringBytesLength = DeserializeLength(data, offset);
            offset += 4;
            this.status = DeserializeString(data, offset, statusStringBytesLength);
            offset += statusStringBytesLength;
            offset = this.plan.Deserialize(data, offset);

            return offset;
        }

        public override string ToString()
        {
            return "MGetRoutePlanResponse: " +
            "\nsuccess: " + success.ToString() +
            "\nstatus: " + status.ToString() +
            "\nplan: " + plan.ToString();
        }
    }
}
