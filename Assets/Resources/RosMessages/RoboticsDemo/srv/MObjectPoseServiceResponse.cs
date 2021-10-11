//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.RoboticsDemo
{
    public class MObjectPoseServiceResponse : Message
    {
        public const string RosMessageName = "robotics_demo/ObjectPoseService";

        public Geometry.MPose object_pose;

        public MObjectPoseServiceResponse()
        {
            this.object_pose = new Geometry.MPose();
        }

        public MObjectPoseServiceResponse(Geometry.MPose object_pose)
        {
            this.object_pose = object_pose;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(object_pose.SerializationStatements());

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.object_pose.Deserialize(data, offset);

            return offset;
        }

        public override string ToString()
        {
            return "MObjectPoseServiceResponse: " +
            "\nobject_pose: " + object_pose.ToString();
        }
    }
}
