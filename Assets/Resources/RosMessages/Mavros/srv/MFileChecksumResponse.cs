//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Mavros
{
    public class MFileChecksumResponse : Message
    {
        public const string RosMessageName = "mavros_msgs/FileChecksum";

        public uint crc32;
        public bool success;
        public int r_errno;

        public MFileChecksumResponse()
        {
            this.crc32 = 0;
            this.success = false;
            this.r_errno = 0;
        }

        public MFileChecksumResponse(uint crc32, bool success, int r_errno)
        {
            this.crc32 = crc32;
            this.success = success;
            this.r_errno = r_errno;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.Add(BitConverter.GetBytes(this.crc32));
            listOfSerializations.Add(BitConverter.GetBytes(this.success));
            listOfSerializations.Add(BitConverter.GetBytes(this.r_errno));

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            this.crc32 = BitConverter.ToUInt32(data, offset);
            offset += 4;
            this.success = BitConverter.ToBoolean(data, offset);
            offset += 1;
            this.r_errno = BitConverter.ToInt32(data, offset);
            offset += 4;

            return offset;
        }

        public override string ToString()
        {
            return "MFileChecksumResponse: " +
            "\ncrc32: " + crc32.ToString() +
            "\nsuccess: " + success.ToString() +
            "\nr_errno: " + r_errno.ToString();
        }
    }
}
