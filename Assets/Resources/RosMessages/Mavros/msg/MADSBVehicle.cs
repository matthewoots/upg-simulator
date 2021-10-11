//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;

namespace RosMessageTypes.Mavros
{
    public class MADSBVehicle : Message
    {
        public const string RosMessageName = "mavros_msgs/ADSBVehicle";

        //  The location and information of an ADSB vehicle
        // 
        //  https://mavlink.io/en/messages/common.html#ADSB_VEHICLE
        //  [[[cog:
        //  from pymavlink.dialects.v20 import common
        // 
        //  def decl_enum(ename, pfx='', bsz=8):
        //      enum = sorted(common.enums[ename].items())
        //      enum.pop() # remove ENUM_END
        // 
        //      cog.outl("# " + ename)
        //      for k, e in enum:
        //          sn = e.name[len(ename) + 1:]
        //          l = "uint{bsz} {pfx}{sn} = {k}".format(**locals())
        //          if e.description:
        //              l += ' ' * (40 - len(l)) + ' # ' + e.description
        //          cog.outl(l)
        // 
        //  decl_enum('ADSB_ALTITUDE_TYPE', 'ALT_')
        //  decl_enum('ADSB_EMITTER_TYPE', 'EMITTER_')
        //  decl_enum('ADSB_FLAGS', 'FLAG_', 16)
        //  ]]]
        //  ADSB_ALTITUDE_TYPE
        public const byte ALT_PRESSURE_QNH = 0; //  Altitude reported from a Baro source using QNH reference
        public const byte ALT_GEOMETRIC = 1; //  Altitude reported from a GNSS source
        //  ADSB_EMITTER_TYPE
        public const byte EMITTER_NO_INFO = 0;
        public const byte EMITTER_LIGHT = 1;
        public const byte EMITTER_SMALL = 2;
        public const byte EMITTER_LARGE = 3;
        public const byte EMITTER_HIGH_VORTEX_LARGE = 4;
        public const byte EMITTER_HEAVY = 5;
        public const byte EMITTER_HIGHLY_MANUV = 6;
        public const byte EMITTER_ROTOCRAFT = 7;
        public const byte EMITTER_UNASSIGNED = 8;
        public const byte EMITTER_GLIDER = 9;
        public const byte EMITTER_LIGHTER_AIR = 10;
        public const byte EMITTER_PARACHUTE = 11;
        public const byte EMITTER_ULTRA_LIGHT = 12;
        public const byte EMITTER_UNASSIGNED2 = 13;
        public const byte EMITTER_UAV = 14;
        public const byte EMITTER_SPACE = 15;
        public const byte EMITTER_UNASSGINED3 = 16;
        public const byte EMITTER_EMERGENCY_SURFACE = 17;
        public const byte EMITTER_SERVICE_SURFACE = 18;
        public const byte EMITTER_POINT_OBSTACLE = 19;
        //  ADSB_FLAGS
        public const ushort FLAG_VALID_COORDS = 1;
        public const ushort FLAG_VALID_ALTITUDE = 2;
        public const ushort FLAG_VALID_HEADING = 4;
        public const ushort FLAG_VALID_VELOCITY = 8;
        public const ushort FLAG_VALID_CALLSIGN = 16;
        public const ushort FLAG_VALID_SQUAWK = 32;
        public const ushort FLAG_SIMULATED = 64;
        public const ushort FLAG_VERTICAL_VELOCITY_VALID = 128;
        public const ushort FLAG_BARO_VALID = 256;
        public const ushort FLAG_SOURCE_UAT = 32768;
        //  [[[end]]] (checksum: a34f2a081739921b6e3e443ed0516d8d)
        public Std.MHeader header;
        public uint ICAO_address;
        public string callsign;
        public double latitude;
        public double longitude;
        public float altitude;
        //  AMSL
        public float heading;
        //  deg [0..360)
        public float hor_velocity;
        //  m/s
        public float ver_velocity;
        //  m/s
        public byte altitude_type;
        //  Type from ADSB_ALTITUDE_TYPE enum
        public byte emitter_type;
        //  Type from ADSB_EMITTER_TYPE enum
        public MDuration tslc;
        //  Duration from last communication, seconds [0..255]
        public ushort flags;
        //  ADSB_FLAGS bit field
        public ushort squawk;
        //  Squawk code

        public MADSBVehicle()
        {
            this.header = new Std.MHeader();
            this.ICAO_address = 0;
            this.callsign = "";
            this.latitude = 0.0;
            this.longitude = 0.0;
            this.altitude = 0.0f;
            this.heading = 0.0f;
            this.hor_velocity = 0.0f;
            this.ver_velocity = 0.0f;
            this.altitude_type = 0;
            this.emitter_type = 0;
            this.tslc = new MDuration();
            this.flags = 0;
            this.squawk = 0;
        }

        public MADSBVehicle(Std.MHeader header, uint ICAO_address, string callsign, double latitude, double longitude, float altitude, float heading, float hor_velocity, float ver_velocity, byte altitude_type, byte emitter_type, MDuration tslc, ushort flags, ushort squawk)
        {
            this.header = header;
            this.ICAO_address = ICAO_address;
            this.callsign = callsign;
            this.latitude = latitude;
            this.longitude = longitude;
            this.altitude = altitude;
            this.heading = heading;
            this.hor_velocity = hor_velocity;
            this.ver_velocity = ver_velocity;
            this.altitude_type = altitude_type;
            this.emitter_type = emitter_type;
            this.tslc = tslc;
            this.flags = flags;
            this.squawk = squawk;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(header.SerializationStatements());
            listOfSerializations.Add(BitConverter.GetBytes(this.ICAO_address));
            listOfSerializations.Add(SerializeString(this.callsign));
            listOfSerializations.Add(BitConverter.GetBytes(this.latitude));
            listOfSerializations.Add(BitConverter.GetBytes(this.longitude));
            listOfSerializations.Add(BitConverter.GetBytes(this.altitude));
            listOfSerializations.Add(BitConverter.GetBytes(this.heading));
            listOfSerializations.Add(BitConverter.GetBytes(this.hor_velocity));
            listOfSerializations.Add(BitConverter.GetBytes(this.ver_velocity));
            listOfSerializations.Add(new[]{this.altitude_type});
            listOfSerializations.Add(new[]{this.emitter_type});
            listOfSerializations.AddRange(tslc.SerializationStatements());
            listOfSerializations.Add(BitConverter.GetBytes(this.flags));
            listOfSerializations.Add(BitConverter.GetBytes(this.squawk));

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.header.Deserialize(data, offset);
            this.ICAO_address = BitConverter.ToUInt32(data, offset);
            offset += 4;
            var callsignStringBytesLength = DeserializeLength(data, offset);
            offset += 4;
            this.callsign = DeserializeString(data, offset, callsignStringBytesLength);
            offset += callsignStringBytesLength;
            this.latitude = BitConverter.ToDouble(data, offset);
            offset += 8;
            this.longitude = BitConverter.ToDouble(data, offset);
            offset += 8;
            this.altitude = BitConverter.ToSingle(data, offset);
            offset += 4;
            this.heading = BitConverter.ToSingle(data, offset);
            offset += 4;
            this.hor_velocity = BitConverter.ToSingle(data, offset);
            offset += 4;
            this.ver_velocity = BitConverter.ToSingle(data, offset);
            offset += 4;
            this.altitude_type = data[offset];;
            offset += 1;
            this.emitter_type = data[offset];;
            offset += 1;
            offset = this.tslc.Deserialize(data, offset);
            this.flags = BitConverter.ToUInt16(data, offset);
            offset += 2;
            this.squawk = BitConverter.ToUInt16(data, offset);
            offset += 2;

            return offset;
        }

        public override string ToString()
        {
            return "MADSBVehicle: " +
            "\nheader: " + header.ToString() +
            "\nICAO_address: " + ICAO_address.ToString() +
            "\ncallsign: " + callsign.ToString() +
            "\nlatitude: " + latitude.ToString() +
            "\nlongitude: " + longitude.ToString() +
            "\naltitude: " + altitude.ToString() +
            "\nheading: " + heading.ToString() +
            "\nhor_velocity: " + hor_velocity.ToString() +
            "\nver_velocity: " + ver_velocity.ToString() +
            "\naltitude_type: " + altitude_type.ToString() +
            "\nemitter_type: " + emitter_type.ToString() +
            "\ntslc: " + tslc.ToString() +
            "\nflags: " + flags.ToString() +
            "\nsquawk: " + squawk.ToString();
        }
    }
}
