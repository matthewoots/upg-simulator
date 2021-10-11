//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Mavros
{
    public class MFileListResponse : Message
    {
        public const string RosMessageName = "mavros_msgs/FileList";

        public MFileEntry[] list;
        public bool success;
        public int r_errno;

        public MFileListResponse()
        {
            this.list = new MFileEntry[0];
            this.success = false;
            this.r_errno = 0;
        }

        public MFileListResponse(MFileEntry[] list, bool success, int r_errno)
        {
            this.list = list;
            this.success = success;
            this.r_errno = r_errno;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            
            listOfSerializations.Add(BitConverter.GetBytes(list.Length));
            foreach(var entry in list)
                listOfSerializations.Add(entry.Serialize());
            listOfSerializations.Add(BitConverter.GetBytes(this.success));
            listOfSerializations.Add(BitConverter.GetBytes(this.r_errno));

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            
            var listArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.list= new MFileEntry[listArrayLength];
            for(var i = 0; i < listArrayLength; i++)
            {
                this.list[i] = new MFileEntry();
                offset = this.list[i].Deserialize(data, offset);
            }
            this.success = BitConverter.ToBoolean(data, offset);
            offset += 1;
            this.r_errno = BitConverter.ToInt32(data, offset);
            offset += 4;

            return offset;
        }

        public override string ToString()
        {
            return "MFileListResponse: " +
            "\nlist: " + System.String.Join(", ", list.ToList()) +
            "\nsuccess: " + success.ToString() +
            "\nr_errno: " + r_errno.ToString();
        }
    }
}