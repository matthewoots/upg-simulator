//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Mavros
{
    public class MOverrideRCIn : Message
    {
        public const string RosMessageName = "mavros_msgs/OverrideRCIn";

        //  Override RC Input
        //  Currently MAVLink defines override for 8 channel
        public const ushort CHAN_RELEASE = 0;
        public const ushort CHAN_NOCHANGE = 65535;
        public ushort[] channels;

        public MOverrideRCIn()
        {
            this.channels = new ushort[8];
        }

        public MOverrideRCIn(ushort[] channels)
        {
            this.channels = channels;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            
            Array.Resize(ref channels, 8);
            foreach(var entry in channels)
                listOfSerializations.Add(BitConverter.GetBytes(entry));

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            
            this.channels= new ushort[8];
            for(var i = 0; i < 8; i++)
            {
                this.channels[i] = BitConverter.ToUInt16(data, offset);
                offset += 2;
            }

            return offset;
        }

        public override string ToString()
        {
            return "MOverrideRCIn: " +
            "\nchannels: " + System.String.Join(", ", channels.ToList());
        }
    }
}