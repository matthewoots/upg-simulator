using System;
using System.Collections.Generic;
using UnityEngine;
using sensors_suite;


namespace RapidGUI.Example
{
    /// <summary>
    ///  RGUI.Field() examples part1
    /// </summary>
    public class rapid_gui_field_px4 : ExampleBase
    {
        public GameObject Object;

        // public string stringVal;
        // public bool boolVal;
        // public int intVal;
        // public float floatVal;
        // public Color colorVal;

        // public Vector2 vector2Val;
        // public Vector3 vector3Val;
        // public Vector4 vector4Val;
        // public Vector2Int vector2IntVal;
        // public Vector3Int vector3IntVal;
        // public Rect rectVal;
        // public RectInt rectIntVal;
        // public RectOffset rectOffsetVal;
        // public Bounds boundsVal;
        // public BoundsInt boundsIntVal;        

        // public float[] arrayVal;
        // public List<int> listVal;

        protected override string title => "RGUI.PX4Field()s";

        public override void DoGUI()
        {
            tcp_px4_unity_server tcp = Object.GetComponent<tcp_px4_unity_server>();
            MAVLink.mavlink_hil_state_quaternion_t _state = tcp.state_data;
            MAVLink.mavlink_hil_sensor_t _sensor = tcp.sensors_data;
            MAVLink.mavlink_hil_gps_t _gps = tcp.gps_data;

            Vector3 accel = new Vector3(_sensor.xacc, _sensor.yacc, _sensor.zacc);
            Vector3 accelVal = RGUI.Field(return_in_dp(accel, 2), "Accel [m/s/s]");
            
            Vector3 gyro = new Vector3(_sensor.xgyro, _sensor.ygyro, _sensor.zgyro);
            Vector3 gyroVal = RGUI.Field(return_in_dp(gyro, 2), "Gyro [rad/s]");

            float temperatureVal = RGUI.Field(_sensor.temperature, "Temperature [degC]");
            float abs_pressureVal = RGUI.Field(_sensor.abs_pressure, "Abs pressure [hPa]");
            float pressure_altVal = RGUI.Field(_sensor.pressure_alt, "Altitude Pa");

            Vector3 mag = new Vector3(_sensor.xmag, _sensor.ymag, _sensor.zmag);
            Vector3 magVal = RGUI.Field(return_in_dp(mag, 3), "Mag [gauss]");

            int fields_updatedVal = RGUI.Field((int)_sensor.fields_updated, "Sensor Fields Updated");

            // float rollspeedVal = RGUI.Field(_state.rollspeed, "Rollspeed [rad/s]");
            // float pitchspeedVal = RGUI.Field(_state.pitchspeed, "Pitchspeed [rad/s]");
            // float yawspeedVal = RGUI.Field(_state.yawspeed, "Yawspeed [rad/s]");
        
            // float[] attitudeVal = RGUI.Field(_state.attitude_quaternion, "Quarternion");

            Vector2 gps_latlon = new Vector2(_gps.lat, _gps.lon);
            Vector2 gpsVal =  RGUI.Field(gps_latlon, "GPS");

            int gps_alt = _gps.alt;
            int gps_altVal =  RGUI.Field(gps_alt, "GPS alt");

            // stringVal = RGUI.Field(stringVal, "string");
            // boolVal = RGUI.Field(boolVal, "bool");
            // intVal = RGUI.Field(intVal, "int");
            // floatVal = RGUI.Field(floatVal, "float");
            // colorVal = RGUI.Field(colorVal, "color");

            // vector2Val = RGUI.Field(vector2Val, "vector2");
            // vector3Val = RGUI.Field(vector3Val, "vector3");
            // vector4Val = RGUI.Field(vector4Val, "vector4");
            // vector2IntVal = RGUI.Field(vector2IntVal, "vector2Int");
            // vector3IntVal = RGUI.Field(vector3IntVal, "vector3Int");
            // rectVal = RGUI.Field(rectVal, "rect");
            // rectIntVal = RGUI.Field(rectIntVal, "rectInt");
            // rectOffsetVal = RGUI.Field(rectOffsetVal, "rectOffset");
            // boundsVal = RGUI.Field(boundsVal, "bounds");
            // boundsIntVal = RGUI.Field(boundsIntVal, "boundsInt");
            // arrayVal = RGUI.Field(arrayVal, "array");
            // listVal = RGUI.Field(listVal, "list");

            // listVal = RGUI.ListField(listVal, "list with custom element GUI", (list, idx, label) =>
            // {
            //     using (new GUILayout.HorizontalScope())
            //     {
            //         var v = list[idx];
            //         v = RGUI.Slider(v, 100, label);
            //         if (GUILayout.Button("+")) v++;
            //         if (GUILayout.Button("-")) v--;

            //         return v;
            //     }
            // });
        }

        public static Vector3 return_in_dp(Vector3 vector3, int decimalPlaces)
        {

            float multiplier = 1;
            for (int i = 0; i < decimalPlaces; i++)
            {
                multiplier *= 10f;
            }

            return new Vector3(
                Mathf.Round(vector3.x * multiplier) / multiplier,
                Mathf.Round(vector3.y * multiplier) / multiplier,
                Mathf.Round(vector3.z * multiplier) / multiplier);
        }
     
    }
}