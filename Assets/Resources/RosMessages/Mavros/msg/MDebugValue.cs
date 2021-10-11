//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Mavros
{
    public class MDebugValue : Message
    {
        public const string RosMessageName = "mavros_msgs/DebugValue";

        //  Msg for Debug MAVLink API
        // 
        //  Supported types:
        //  DEBUG			https://mavlink.io/en/messages/common.html#DEBUG
        //  DEBUG_VECTOR			https://mavlink.io/en/messages/common.html#DEBUG_VECT
        //  NAMED_VALUE_FLOAT		https://mavlink.io/en/messages/common.html#NAMED_VALUE_FLOAT
        //  NAMED_VALUE_INT		https://mavlink.io/en/messages/common.html#NAMED_VALUE_INT
        //  @TODO: add support for DEBUG_ARRAY (https://github.com/mavlink/mavlink/pull/734)
        public Std.MHeader header;
        public int index;
        //  index value of DEBUG value (-1 if not indexed)
        public string name;
        //  value name/key
        public float value_float;
        //  float value for NAMED_VALUE_FLOAT and DEBUG
        public int value_int;
        //  int value for NAMED_VALUE_INT
        public float[] data;
        //  DEBUG vector or array
        public byte type;
        public const byte TYPE_DEBUG = 0;
        public const byte TYPE_DEBUG_VECT = 1;
        public const byte TYPE_DEBUG_ARRAY = 2;
        public const byte TYPE_NAMED_VALUE_FLOAT = 3;
        public const byte TYPE_NAMED_VALUE_INT = 4;

        public MDebugValue()
        {
            this.header = new Std.MHeader();
            this.index = 0;
            this.name = "";
            this.value_float = 0.0f;
            this.value_int = 0;
            this.data = new float[0];
            this.type = 0;
        }

        public MDebugValue(Std.MHeader header, int index, string name, float value_float, int value_int, float[] data, byte type)
        {
            this.header = header;
            this.index = index;
            this.name = name;
            this.value_float = value_float;
            this.value_int = value_int;
            this.data = data;
            this.type = type;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(header.SerializationStatements());
            listOfSerializations.Add(BitConverter.GetBytes(this.index));
            listOfSerializations.Add(SerializeString(this.name));
            listOfSerializations.Add(BitConverter.GetBytes(this.value_float));
            listOfSerializations.Add(BitConverter.GetBytes(this.value_int));
            
            listOfSerializations.Add(BitConverter.GetBytes(data.Length));
            foreach(var entry in data)
                listOfSerializations.Add(BitConverter.GetBytes(entry));
            listOfSerializations.Add(new[]{this.type});

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.header.Deserialize(data, offset);
            this.index = BitConverter.ToInt32(data, offset);
            offset += 4;
            var nameStringBytesLength = DeserializeLength(data, offset);
            offset += 4;
            this.name = DeserializeString(data, offset, nameStringBytesLength);
            offset += nameStringBytesLength;
            this.value_float = BitConverter.ToSingle(data, offset);
            offset += 4;
            this.value_int = BitConverter.ToInt32(data, offset);
            offset += 4;
            
            var dataArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.data= new float[dataArrayLength];
            for(var i = 0; i < dataArrayLength; i++)
            {
                this.data[i] = BitConverter.ToSingle(data, offset);
                offset += 4;
            }
            this.type = data[offset];;
            offset += 1;

            return offset;
        }

        public override string ToString()
        {
            return "MDebugValue: " +
            "\nheader: " + header.ToString() +
            "\nindex: " + index.ToString() +
            "\nname: " + name.ToString() +
            "\nvalue_float: " + value_float.ToString() +
            "\nvalue_int: " + value_int.ToString() +
            "\ndata: " + System.String.Join(", ", data.ToList()) +
            "\ntype: " + type.ToString();
        }
    }
}
