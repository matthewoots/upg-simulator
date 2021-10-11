//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Mavros
{
    public class MBatteryStatus : Message
    {
        public const string RosMessageName = "mavros_msgs/BatteryStatus";

        //  Represent battery status from SYSTEM_STATUS
        // 
        //  To be replaced when sensor_msgs/BatteryState PR will be merged
        //  https://github.com/ros/common_msgs/pull/74
        public Std.MHeader header;
        public float voltage;
        //  [V]
        public float current;
        //  [A]
        public float remaining;
        //  0..1

        public MBatteryStatus()
        {
            this.header = new Std.MHeader();
            this.voltage = 0.0f;
            this.current = 0.0f;
            this.remaining = 0.0f;
        }

        public MBatteryStatus(Std.MHeader header, float voltage, float current, float remaining)
        {
            this.header = header;
            this.voltage = voltage;
            this.current = current;
            this.remaining = remaining;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(header.SerializationStatements());
            listOfSerializations.Add(BitConverter.GetBytes(this.voltage));
            listOfSerializations.Add(BitConverter.GetBytes(this.current));
            listOfSerializations.Add(BitConverter.GetBytes(this.remaining));

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.header.Deserialize(data, offset);
            this.voltage = BitConverter.ToSingle(data, offset);
            offset += 4;
            this.current = BitConverter.ToSingle(data, offset);
            offset += 4;
            this.remaining = BitConverter.ToSingle(data, offset);
            offset += 4;

            return offset;
        }

        public override string ToString()
        {
            return "MBatteryStatus: " +
            "\nheader: " + header.ToString() +
            "\nvoltage: " + voltage.ToString() +
            "\ncurrent: " + current.ToString() +
            "\nremaining: " + remaining.ToString();
        }
    }
}
