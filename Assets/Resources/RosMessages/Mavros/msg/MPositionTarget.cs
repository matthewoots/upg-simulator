//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Mavros
{
    public class MPositionTarget : Message
    {
        public const string RosMessageName = "mavros_msgs/PositionTarget";

        //  Message for SET_POSITION_TARGET_LOCAL_NED
        // 
        //  Some complex system requires all feautures that mavlink
        //  message provide. See issue #402.
        public Std.MHeader header;
        public byte coordinate_frame;
        public const byte FRAME_LOCAL_NED = 1;
        public const byte FRAME_LOCAL_OFFSET_NED = 7;
        public const byte FRAME_BODY_NED = 8;
        public const byte FRAME_BODY_OFFSET_NED = 9;
        public ushort type_mask;
        public const ushort IGNORE_PX = 1; //  Position ignore flags
        public const ushort IGNORE_PY = 2;
        public const ushort IGNORE_PZ = 4;
        public const ushort IGNORE_VX = 8; //  Velocity vector ignore flags
        public const ushort IGNORE_VY = 16;
        public const ushort IGNORE_VZ = 32;
        public const ushort IGNORE_AFX = 64; //  Acceleration/Force vector ignore flags
        public const ushort IGNORE_AFY = 128;
        public const ushort IGNORE_AFZ = 256;
        public const ushort FORCE = 512; //  Force in af vector flag
        public const ushort IGNORE_YAW = 1024;
        public const ushort IGNORE_YAW_RATE = 2048;
        public Geometry.MPoint position;
        public Geometry.MVector3 velocity;
        public Geometry.MVector3 acceleration_or_force;
        public float yaw;
        public float yaw_rate;

        public MPositionTarget()
        {
            this.header = new Std.MHeader();
            this.coordinate_frame = 0;
            this.type_mask = 0;
            this.position = new Geometry.MPoint();
            this.velocity = new Geometry.MVector3();
            this.acceleration_or_force = new Geometry.MVector3();
            this.yaw = 0.0f;
            this.yaw_rate = 0.0f;
        }

        public MPositionTarget(Std.MHeader header, byte coordinate_frame, ushort type_mask, Geometry.MPoint position, Geometry.MVector3 velocity, Geometry.MVector3 acceleration_or_force, float yaw, float yaw_rate)
        {
            this.header = header;
            this.coordinate_frame = coordinate_frame;
            this.type_mask = type_mask;
            this.position = position;
            this.velocity = velocity;
            this.acceleration_or_force = acceleration_or_force;
            this.yaw = yaw;
            this.yaw_rate = yaw_rate;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(header.SerializationStatements());
            listOfSerializations.Add(new[]{this.coordinate_frame});
            listOfSerializations.Add(BitConverter.GetBytes(this.type_mask));
            listOfSerializations.AddRange(position.SerializationStatements());
            listOfSerializations.AddRange(velocity.SerializationStatements());
            listOfSerializations.AddRange(acceleration_or_force.SerializationStatements());
            listOfSerializations.Add(BitConverter.GetBytes(this.yaw));
            listOfSerializations.Add(BitConverter.GetBytes(this.yaw_rate));

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.header.Deserialize(data, offset);
            this.coordinate_frame = data[offset];;
            offset += 1;
            this.type_mask = BitConverter.ToUInt16(data, offset);
            offset += 2;
            offset = this.position.Deserialize(data, offset);
            offset = this.velocity.Deserialize(data, offset);
            offset = this.acceleration_or_force.Deserialize(data, offset);
            this.yaw = BitConverter.ToSingle(data, offset);
            offset += 4;
            this.yaw_rate = BitConverter.ToSingle(data, offset);
            offset += 4;

            return offset;
        }

        public override string ToString()
        {
            return "MPositionTarget: " +
            "\nheader: " + header.ToString() +
            "\ncoordinate_frame: " + coordinate_frame.ToString() +
            "\ntype_mask: " + type_mask.ToString() +
            "\nposition: " + position.ToString() +
            "\nvelocity: " + velocity.ToString() +
            "\nacceleration_or_force: " + acceleration_or_force.ToString() +
            "\nyaw: " + yaw.ToString() +
            "\nyaw_rate: " + yaw_rate.ToString();
        }
    }
}