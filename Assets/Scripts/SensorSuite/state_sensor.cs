/**
* @author Matthew Woo
* @contact matthewoots@gmail.com
* @year 2022
**/

using System;
using System.IO;

using UnityEngine;
using Unity.Robotics.ROSTCPConnector;

namespace sensors_suite {
    public class state_sensor : MonoBehaviour
    {
        public GameObject gps;
        public string topic = "gps";
        public Vector2 base_gps_rad = new Vector2((float)1.299851, (float)103.772243);
        public Vector2 current_gps_rad = new Vector2();

        private Vector3 gravity = new Vector3(0.0f, 0.0f, -9.81f);
        private const double M_PI = 3.141926535;
        private const double rad2deg = 180 / M_PI;
        private const double deg2rad = M_PI / 180;
        private const double CONSTANTS_RADIUS_OF_EARTH = 6371000;					// meters (m)
        private const float  CONSTANTS_RADIUS_OF_EARTH_F = (float)CONSTANTS_RADIUS_OF_EARTH;		// meters (m)
        private const float CONSTANTS_EARTH_SPIN_RATE = 7.2921150e-5f;
        private static Quaternion q_NWU_to_NED = new Quaternion(1.0f, 0.0f, 0.0f, 0.0f);


        private void Update()
        {
            /* EUN to NED */
            Vector3 ned_pos = new Vector3(gps.transform.position.z, gps.transform.position.x, - gps.transform.position.y);
            current_gps_rad = gps_pose_to_coord(ned_pos, base_gps_rad);
            
        }

        public MAVLink.mavlink_hil_state_quaternion_t px4_state_sensor_data()
        {
            MAVLink.mavlink_hil_state_quaternion_t _state_data = new MAVLink.mavlink_hil_state_quaternion_t();
            

            return _state_data;
        }

        Vector2 gps_pose_to_coord(Vector3 pos, Vector2 gps_home_rad)
        {
            Vector2 home_rad = new Vector2(gps_home_rad.x * (float)deg2rad, gps_home_rad.y * (float)deg2rad);
            float lat_home = home_rad.x; float lon_home = home_rad.y;
            // reproject local position to gps coordinates
            float x_rad = pos.y / CONSTANTS_RADIUS_OF_EARTH_F;    // north
            float y_rad = pos.x / CONSTANTS_RADIUS_OF_EARTH_F;    // east
            float c = Mathf.Sqrt(x_rad * x_rad + y_rad * y_rad);
            float sin_c = Mathf.Sin(c);
            float cos_c = Mathf.Cos(c);

            float lat_rad, lon_rad;

            if (c != 0.0) {
                lat_rad = Mathf.Asin(cos_c * Mathf.Sin(lat_home) + (x_rad * sin_c * Mathf.Cos(lat_home)) / c);
                lon_rad = (lon_home + Mathf.Atan2(y_rad * sin_c, c * Mathf.Cos(lat_home) * cos_c - x_rad * Mathf.Sin(lat_home) * sin_c));
            } else {
                lat_rad = lat_home;
                lon_rad = lon_home;
            }

            return new Vector2(lat_rad, lon_rad);
        }

        private Quaternion quaternion_eun_lh_to_nwu_rh(Quaternion Q)
        {
            float tmp_quat_angle;
            Vector3 quat_axis;

            Q.ToAngleAxis(out tmp_quat_angle, out quat_axis);
            float quat_angle = tmp_quat_angle / 180.0f * (float)Math.PI;
            quat_axis.Normalize();
            quat_angle *= -1;
            quat_axis = new Vector3(quat_axis.z, -quat_axis.x, quat_axis.y);

            /* How to rotate quaternion */
            /* http://www.euclideanspace.com/maths/geometry/rotations/conversions/angleToQuaternion/ */
            float s = (float)Math.Sin(quat_angle / 2);
            float quatX = quat_axis.x * s;
            float quatY = quat_axis.y * s;
            float quatZ = quat_axis.z * s;
            float quatW = (float)Math.Cos(quat_angle / 2);

            return new Quaternion(quatX, quatY, quatZ, quatW);
        }

    }
}