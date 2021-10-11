//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Mavros
{
    public class MHilStateQuaternion : Message
    {
        public const string RosMessageName = "mavros_msgs/HilStateQuaternion";

        //  HilStateQuaternion.msg
        // 
        //  ROS representation of MAVLink HIL_STATE_QUATERNION
        //  See mavlink message documentation here:
        //  https://mavlink.io/en/messages/common.html#HIL_STATE_QUATERNION
        public Std.MHeader header;
        public Geometry.MQuaternion orientation;
        public Geometry.MVector3 angular_velocity;
        public Geometry.MVector3 linear_acceleration;
        public Geometry.MVector3 linear_velocity;
        public Geographic.MGeoPoint geo;
        public float ind_airspeed;
        public float true_airspeed;

        public MHilStateQuaternion()
        {
            this.header = new Std.MHeader();
            this.orientation = new Geometry.MQuaternion();
            this.angular_velocity = new Geometry.MVector3();
            this.linear_acceleration = new Geometry.MVector3();
            this.linear_velocity = new Geometry.MVector3();
            this.geo = new Geographic.MGeoPoint();
            this.ind_airspeed = 0.0f;
            this.true_airspeed = 0.0f;
        }

        public MHilStateQuaternion(Std.MHeader header, Geometry.MQuaternion orientation, Geometry.MVector3 angular_velocity, Geometry.MVector3 linear_acceleration, Geometry.MVector3 linear_velocity, Geographic.MGeoPoint geo, float ind_airspeed, float true_airspeed)
        {
            this.header = header;
            this.orientation = orientation;
            this.angular_velocity = angular_velocity;
            this.linear_acceleration = linear_acceleration;
            this.linear_velocity = linear_velocity;
            this.geo = geo;
            this.ind_airspeed = ind_airspeed;
            this.true_airspeed = true_airspeed;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(header.SerializationStatements());
            listOfSerializations.AddRange(orientation.SerializationStatements());
            listOfSerializations.AddRange(angular_velocity.SerializationStatements());
            listOfSerializations.AddRange(linear_acceleration.SerializationStatements());
            listOfSerializations.AddRange(linear_velocity.SerializationStatements());
            listOfSerializations.AddRange(geo.SerializationStatements());
            listOfSerializations.Add(BitConverter.GetBytes(this.ind_airspeed));
            listOfSerializations.Add(BitConverter.GetBytes(this.true_airspeed));

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.header.Deserialize(data, offset);
            offset = this.orientation.Deserialize(data, offset);
            offset = this.angular_velocity.Deserialize(data, offset);
            offset = this.linear_acceleration.Deserialize(data, offset);
            offset = this.linear_velocity.Deserialize(data, offset);
            offset = this.geo.Deserialize(data, offset);
            this.ind_airspeed = BitConverter.ToSingle(data, offset);
            offset += 4;
            this.true_airspeed = BitConverter.ToSingle(data, offset);
            offset += 4;

            return offset;
        }

        public override string ToString()
        {
            return "MHilStateQuaternion: " +
            "\nheader: " + header.ToString() +
            "\norientation: " + orientation.ToString() +
            "\nangular_velocity: " + angular_velocity.ToString() +
            "\nlinear_acceleration: " + linear_acceleration.ToString() +
            "\nlinear_velocity: " + linear_velocity.ToString() +
            "\ngeo: " + geo.ToString() +
            "\nind_airspeed: " + ind_airspeed.ToString() +
            "\ntrue_airspeed: " + true_airspeed.ToString();
        }
    }
}
