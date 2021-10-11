//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Mavros
{
    public class MCommandIntRequest : Message
    {
        public const string RosMessageName = "mavros_msgs/CommandInt";

        //  Generic COMMAND_INT
        public bool broadcast;
        //  send this command in broadcast mode
        public byte frame;
        public ushort command;
        public byte current;
        public byte autocontinue;
        public float param1;
        public float param2;
        public float param3;
        public float param4;
        public int x;
        //  latitude in deg * 1E7 or local x * 1E4 m
        public int y;
        //  longitude in deg * 1E7 or local y * 1E4 m
        public float z;
        //  altitude

        public MCommandIntRequest()
        {
            this.broadcast = false;
            this.frame = 0;
            this.command = 0;
            this.current = 0;
            this.autocontinue = 0;
            this.param1 = 0.0f;
            this.param2 = 0.0f;
            this.param3 = 0.0f;
            this.param4 = 0.0f;
            this.x = 0;
            this.y = 0;
            this.z = 0.0f;
        }

        public MCommandIntRequest(bool broadcast, byte frame, ushort command, byte current, byte autocontinue, float param1, float param2, float param3, float param4, int x, int y, float z)
        {
            this.broadcast = broadcast;
            this.frame = frame;
            this.command = command;
            this.current = current;
            this.autocontinue = autocontinue;
            this.param1 = param1;
            this.param2 = param2;
            this.param3 = param3;
            this.param4 = param4;
            this.x = x;
            this.y = y;
            this.z = z;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.Add(BitConverter.GetBytes(this.broadcast));
            listOfSerializations.Add(new[]{this.frame});
            listOfSerializations.Add(BitConverter.GetBytes(this.command));
            listOfSerializations.Add(new[]{this.current});
            listOfSerializations.Add(new[]{this.autocontinue});
            listOfSerializations.Add(BitConverter.GetBytes(this.param1));
            listOfSerializations.Add(BitConverter.GetBytes(this.param2));
            listOfSerializations.Add(BitConverter.GetBytes(this.param3));
            listOfSerializations.Add(BitConverter.GetBytes(this.param4));
            listOfSerializations.Add(BitConverter.GetBytes(this.x));
            listOfSerializations.Add(BitConverter.GetBytes(this.y));
            listOfSerializations.Add(BitConverter.GetBytes(this.z));

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            this.broadcast = BitConverter.ToBoolean(data, offset);
            offset += 1;
            this.frame = data[offset];;
            offset += 1;
            this.command = BitConverter.ToUInt16(data, offset);
            offset += 2;
            this.current = data[offset];;
            offset += 1;
            this.autocontinue = data[offset];;
            offset += 1;
            this.param1 = BitConverter.ToSingle(data, offset);
            offset += 4;
            this.param2 = BitConverter.ToSingle(data, offset);
            offset += 4;
            this.param3 = BitConverter.ToSingle(data, offset);
            offset += 4;
            this.param4 = BitConverter.ToSingle(data, offset);
            offset += 4;
            this.x = BitConverter.ToInt32(data, offset);
            offset += 4;
            this.y = BitConverter.ToInt32(data, offset);
            offset += 4;
            this.z = BitConverter.ToSingle(data, offset);
            offset += 4;

            return offset;
        }

        public override string ToString()
        {
            return "MCommandIntRequest: " +
            "\nbroadcast: " + broadcast.ToString() +
            "\nframe: " + frame.ToString() +
            "\ncommand: " + command.ToString() +
            "\ncurrent: " + current.ToString() +
            "\nautocontinue: " + autocontinue.ToString() +
            "\nparam1: " + param1.ToString() +
            "\nparam2: " + param2.ToString() +
            "\nparam3: " + param3.ToString() +
            "\nparam4: " + param4.ToString() +
            "\nx: " + x.ToString() +
            "\ny: " + y.ToString() +
            "\nz: " + z.ToString();
        }
    }
}
