//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Geographic
{
    public class MGetGeoPathRequest : Message
    {
        public const string RosMessageName = "geographic_msgs/GetGeoPath";

        //  Searches for given start and goal the nearest route segments
        //  and determine the path through the RouteNetwork
        public MGeoPoint start;
        //  starting point
        public MGeoPoint goal;
        //  goal point

        public MGetGeoPathRequest()
        {
            this.start = new MGeoPoint();
            this.goal = new MGeoPoint();
        }

        public MGetGeoPathRequest(MGeoPoint start, MGeoPoint goal)
        {
            this.start = start;
            this.goal = goal;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(start.SerializationStatements());
            listOfSerializations.AddRange(goal.SerializationStatements());

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.start.Deserialize(data, offset);
            offset = this.goal.Deserialize(data, offset);

            return offset;
        }

        public override string ToString()
        {
            return "MGetGeoPathRequest: " +
            "\nstart: " + start.ToString() +
            "\ngoal: " + goal.ToString();
        }
    }
}
