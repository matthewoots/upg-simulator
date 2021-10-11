//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Geographic
{
    public class MGetGeographicMapResponse : Message
    {
        public const string RosMessageName = "geographic_msgs/GetGeographicMap";

        public bool success;
        //  true if the call succeeded
        public string status;
        //  more details
        //  The requested map, its bounds may differ from the requested bounds.
        public MGeographicMap map;

        public MGetGeographicMapResponse()
        {
            this.success = false;
            this.status = "";
            this.map = new MGeographicMap();
        }

        public MGetGeographicMapResponse(bool success, string status, MGeographicMap map)
        {
            this.success = success;
            this.status = status;
            this.map = map;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.Add(BitConverter.GetBytes(this.success));
            listOfSerializations.Add(SerializeString(this.status));
            listOfSerializations.AddRange(map.SerializationStatements());

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
            offset = this.map.Deserialize(data, offset);

            return offset;
        }

        public override string ToString()
        {
            return "MGetGeographicMapResponse: " +
            "\nsuccess: " + success.ToString() +
            "\nstatus: " + status.ToString() +
            "\nmap: " + map.ToString();
        }
    }
}