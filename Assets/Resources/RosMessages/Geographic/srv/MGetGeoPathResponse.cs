//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Geographic
{
    public class MGetGeoPathResponse : Message
    {
        public const string RosMessageName = "geographic_msgs/GetGeoPath";

        public bool success;
        //  true if the call succeeded
        public string status;
        //  more details
        public MGeoPath plan;
        //  path to follow
        public Uuid.MUniqueID network;
        //  the uuid of the RouteNetwork
        public Uuid.MUniqueID start_seg;
        //  the uuid of the starting RouteSegment
        public Uuid.MUniqueID goal_seg;
        //  the uuid of the ending RouteSegment
        public double distance;
        //  the length of the plan

        public MGetGeoPathResponse()
        {
            this.success = false;
            this.status = "";
            this.plan = new MGeoPath();
            this.network = new Uuid.MUniqueID();
            this.start_seg = new Uuid.MUniqueID();
            this.goal_seg = new Uuid.MUniqueID();
            this.distance = 0.0;
        }

        public MGetGeoPathResponse(bool success, string status, MGeoPath plan, Uuid.MUniqueID network, Uuid.MUniqueID start_seg, Uuid.MUniqueID goal_seg, double distance)
        {
            this.success = success;
            this.status = status;
            this.plan = plan;
            this.network = network;
            this.start_seg = start_seg;
            this.goal_seg = goal_seg;
            this.distance = distance;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.Add(BitConverter.GetBytes(this.success));
            listOfSerializations.Add(SerializeString(this.status));
            listOfSerializations.AddRange(plan.SerializationStatements());
            listOfSerializations.AddRange(network.SerializationStatements());
            listOfSerializations.AddRange(start_seg.SerializationStatements());
            listOfSerializations.AddRange(goal_seg.SerializationStatements());
            listOfSerializations.Add(BitConverter.GetBytes(this.distance));

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
            offset = this.network.Deserialize(data, offset);
            offset = this.start_seg.Deserialize(data, offset);
            offset = this.goal_seg.Deserialize(data, offset);
            this.distance = BitConverter.ToDouble(data, offset);
            offset += 8;

            return offset;
        }

        public override string ToString()
        {
            return "MGetGeoPathResponse: " +
            "\nsuccess: " + success.ToString() +
            "\nstatus: " + status.ToString() +
            "\nplan: " + plan.ToString() +
            "\nnetwork: " + network.ToString() +
            "\nstart_seg: " + start_seg.ToString() +
            "\ngoal_seg: " + goal_seg.ToString() +
            "\ndistance: " + distance.ToString();
        }
    }
}
