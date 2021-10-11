//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Mavros
{
    public class MGPSRAW : Message
    {
        public const string RosMessageName = "mavros_msgs/GPSRAW";

        //  FCU GPS RAW message for the gps_status plugin
        //  A merge of <a href="https://mavlink.io/en/messages/common.html#GPS_RAW_INT">mavlink GPS_RAW_INT</a> and 
        //  <a href="https://mavlink.io/en/messages/common.html#GPS2_RAW">mavlink GPS2_RAW</a> messages.
        public Std.MHeader header;
        // # GPS_FIX_TYPE enum
        public const byte GPS_FIX_TYPE_NO_GPS = 0; //  No GPS connected
        public const byte GPS_FIX_TYPE_NO_FIX = 1; //  No position information, GPS is connected
        public const byte GPS_FIX_TYPE_2D_FIX = 2; //  2D position
        public const byte GPS_FIX_TYPE_3D_FIX = 3; //  3D position
        public const byte GPS_FIX_TYPE_DGPS = 4; //  DGPS/SBAS aided 3D position
        public const byte GPS_FIX_TYPE_RTK_FLOATR = 5; //  TK float, 3D position
        public const byte GPS_FIX_TYPE_RTK_FIXEDR = 6; //  TK Fixed, 3D position
        public const byte GPS_FIX_TYPE_STATIC = 7; //  Static fixed, typically used for base stations
        public const byte GPS_FIX_TYPE_PPP = 8; //  PPP, 3D position
        public byte fix_type;
        //  [GPS_FIX_TYPE] GPS fix type
        public int lat;
        //  [degE7] Latitude (WGS84, EGM96 ellipsoid)
        public int lon;
        //  [degE7] Longitude (WGS84, EGM96 ellipsoid)
        public int alt;
        //  [mm] Altitude (MSL). Positive for up. Note that virtually all GPS modules provide the MSL altitude in addition to the WGS84 altitude.
        public ushort eph;
        //  GPS HDOP horizontal dilution of position (unitless). If unknown, set to: UINT16_MAX
        public ushort epv;
        //  GPS VDOP vertical dilution of position (unitless). If unknown, set to: UINT16_MAX
        public ushort vel;
        //  [cm/s] GPS ground speed. If unknown, set to: UINT16_MAX
        public ushort cog;
        //  [cdeg] Course over ground (NOT heading, but direction of movement), 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
        public byte satellites_visible;
        //  Number of satellites visible. If unknown, set to 255
        //  -*- only available with MAVLink v2.0 and GPS_RAW_INT messages -*-
        public int alt_ellipsoid;
        //  [mm] Altitude (above WGS84, EGM96 ellipsoid). Positive for up.
        public uint h_acc;
        //  [mm] Position uncertainty. Positive for up.
        public uint v_acc;
        //  [mm] Altitude uncertainty. Positive for up.
        public uint vel_acc;
        //  [mm] Speed uncertainty. Positive for up.
        public int hdg_acc;
        //  [degE5] Heading / track uncertainty
        //  -*- only available with MAVLink v2.0 and GPS2_RAW messages -*-
        public byte dgps_numch;
        //  Number of DGPS satellites
        public uint dgps_age;
        //  [ms] Age of DGPS info

        public MGPSRAW()
        {
            this.header = new Std.MHeader();
            this.fix_type = 0;
            this.lat = 0;
            this.lon = 0;
            this.alt = 0;
            this.eph = 0;
            this.epv = 0;
            this.vel = 0;
            this.cog = 0;
            this.satellites_visible = 0;
            this.alt_ellipsoid = 0;
            this.h_acc = 0;
            this.v_acc = 0;
            this.vel_acc = 0;
            this.hdg_acc = 0;
            this.dgps_numch = 0;
            this.dgps_age = 0;
        }

        public MGPSRAW(Std.MHeader header, byte fix_type, int lat, int lon, int alt, ushort eph, ushort epv, ushort vel, ushort cog, byte satellites_visible, int alt_ellipsoid, uint h_acc, uint v_acc, uint vel_acc, int hdg_acc, byte dgps_numch, uint dgps_age)
        {
            this.header = header;
            this.fix_type = fix_type;
            this.lat = lat;
            this.lon = lon;
            this.alt = alt;
            this.eph = eph;
            this.epv = epv;
            this.vel = vel;
            this.cog = cog;
            this.satellites_visible = satellites_visible;
            this.alt_ellipsoid = alt_ellipsoid;
            this.h_acc = h_acc;
            this.v_acc = v_acc;
            this.vel_acc = vel_acc;
            this.hdg_acc = hdg_acc;
            this.dgps_numch = dgps_numch;
            this.dgps_age = dgps_age;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(header.SerializationStatements());
            listOfSerializations.Add(new[]{this.fix_type});
            listOfSerializations.Add(BitConverter.GetBytes(this.lat));
            listOfSerializations.Add(BitConverter.GetBytes(this.lon));
            listOfSerializations.Add(BitConverter.GetBytes(this.alt));
            listOfSerializations.Add(BitConverter.GetBytes(this.eph));
            listOfSerializations.Add(BitConverter.GetBytes(this.epv));
            listOfSerializations.Add(BitConverter.GetBytes(this.vel));
            listOfSerializations.Add(BitConverter.GetBytes(this.cog));
            listOfSerializations.Add(new[]{this.satellites_visible});
            listOfSerializations.Add(BitConverter.GetBytes(this.alt_ellipsoid));
            listOfSerializations.Add(BitConverter.GetBytes(this.h_acc));
            listOfSerializations.Add(BitConverter.GetBytes(this.v_acc));
            listOfSerializations.Add(BitConverter.GetBytes(this.vel_acc));
            listOfSerializations.Add(BitConverter.GetBytes(this.hdg_acc));
            listOfSerializations.Add(new[]{this.dgps_numch});
            listOfSerializations.Add(BitConverter.GetBytes(this.dgps_age));

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.header.Deserialize(data, offset);
            this.fix_type = data[offset];;
            offset += 1;
            this.lat = BitConverter.ToInt32(data, offset);
            offset += 4;
            this.lon = BitConverter.ToInt32(data, offset);
            offset += 4;
            this.alt = BitConverter.ToInt32(data, offset);
            offset += 4;
            this.eph = BitConverter.ToUInt16(data, offset);
            offset += 2;
            this.epv = BitConverter.ToUInt16(data, offset);
            offset += 2;
            this.vel = BitConverter.ToUInt16(data, offset);
            offset += 2;
            this.cog = BitConverter.ToUInt16(data, offset);
            offset += 2;
            this.satellites_visible = data[offset];;
            offset += 1;
            this.alt_ellipsoid = BitConverter.ToInt32(data, offset);
            offset += 4;
            this.h_acc = BitConverter.ToUInt32(data, offset);
            offset += 4;
            this.v_acc = BitConverter.ToUInt32(data, offset);
            offset += 4;
            this.vel_acc = BitConverter.ToUInt32(data, offset);
            offset += 4;
            this.hdg_acc = BitConverter.ToInt32(data, offset);
            offset += 4;
            this.dgps_numch = data[offset];;
            offset += 1;
            this.dgps_age = BitConverter.ToUInt32(data, offset);
            offset += 4;

            return offset;
        }

        public override string ToString()
        {
            return "MGPSRAW: " +
            "\nheader: " + header.ToString() +
            "\nfix_type: " + fix_type.ToString() +
            "\nlat: " + lat.ToString() +
            "\nlon: " + lon.ToString() +
            "\nalt: " + alt.ToString() +
            "\neph: " + eph.ToString() +
            "\nepv: " + epv.ToString() +
            "\nvel: " + vel.ToString() +
            "\ncog: " + cog.ToString() +
            "\nsatellites_visible: " + satellites_visible.ToString() +
            "\nalt_ellipsoid: " + alt_ellipsoid.ToString() +
            "\nh_acc: " + h_acc.ToString() +
            "\nv_acc: " + v_acc.ToString() +
            "\nvel_acc: " + vel_acc.ToString() +
            "\nhdg_acc: " + hdg_acc.ToString() +
            "\ndgps_numch: " + dgps_numch.ToString() +
            "\ndgps_age: " + dgps_age.ToString();
        }
    }
}