//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.RoboticsDemo
{
    public class MPosRot : Message
    {
        public const string RosMessageName = "robotics_demo/PosRot";

        public float pos_x;
        public float pos_y;
        public float pos_z;
        public float rot_x;
        public float rot_y;
        public float rot_z;
        public float rot_w;

        public MPosRot()
        {
            this.pos_x = 0.0f;
            this.pos_y = 0.0f;
            this.pos_z = 0.0f;
            this.rot_x = 0.0f;
            this.rot_y = 0.0f;
            this.rot_z = 0.0f;
            this.rot_w = 0.0f;
        }

        public MPosRot(float pos_x, float pos_y, float pos_z, float rot_x, float rot_y, float rot_z, float rot_w)
        {
            this.pos_x = pos_x;
            this.pos_y = pos_y;
            this.pos_z = pos_z;
            this.rot_x = rot_x;
            this.rot_y = rot_y;
            this.rot_z = rot_z;
            this.rot_w = rot_w;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.Add(BitConverter.GetBytes(this.pos_x));
            listOfSerializations.Add(BitConverter.GetBytes(this.pos_y));
            listOfSerializations.Add(BitConverter.GetBytes(this.pos_z));
            listOfSerializations.Add(BitConverter.GetBytes(this.rot_x));
            listOfSerializations.Add(BitConverter.GetBytes(this.rot_y));
            listOfSerializations.Add(BitConverter.GetBytes(this.rot_z));
            listOfSerializations.Add(BitConverter.GetBytes(this.rot_w));

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            this.pos_x = BitConverter.ToSingle(data, offset);
            offset += 4;
            this.pos_y = BitConverter.ToSingle(data, offset);
            offset += 4;
            this.pos_z = BitConverter.ToSingle(data, offset);
            offset += 4;
            this.rot_x = BitConverter.ToSingle(data, offset);
            offset += 4;
            this.rot_y = BitConverter.ToSingle(data, offset);
            offset += 4;
            this.rot_z = BitConverter.ToSingle(data, offset);
            offset += 4;
            this.rot_w = BitConverter.ToSingle(data, offset);
            offset += 4;

            return offset;
        }

        public override string ToString()
        {
            return "MPosRot: " +
            "\npos_x: " + pos_x.ToString() +
            "\npos_y: " + pos_y.ToString() +
            "\npos_z: " + pos_z.ToString() +
            "\nrot_x: " + rot_x.ToString() +
            "\nrot_y: " + rot_y.ToString() +
            "\nrot_z: " + rot_z.ToString() +
            "\nrot_w: " + rot_w.ToString();
        }
    }
}
