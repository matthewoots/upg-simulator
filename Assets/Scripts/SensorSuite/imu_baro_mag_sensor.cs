/**
* @author Matthew Woo
* @contact matthewoots@gmail.com
* @year 2022
**/

using System;
using System.IO;
using System.Collections;
using System.Collections.Generic;

using UnityEngine;

using ros_geometry_point = RosMessageTypes.Geometry.Point32Msg;
using ros_geometry_pose = RosMessageTypes.Geometry.PoseStampedMsg;

namespace sensors_suite {
    public class imu_baro_mag_sensor : MonoBehaviour
    {
        public GameObject sensor;
        public string frame_id = "/map";
        public string ros_coordinate_frame = "ned";
        public float acc_noise = 0.005f, gyro_noise = 0.005f;
        public float baro_alt_noise = 0.005f, baro_pressure_noise = 0.0005f;
        public float mag_noise = 0.005f;

        public gps_sensor gps;

        private Transform current_transform;
        private Vector3 current_position, previous_position;
        private Vector3 local_vel, local_acc, local_ang_vel;
        private Vector3 prev_lvel, prev_lacc;
        private Vector2 tmp_gps;
        private static Quaternion q_NWU_to_NED = new Quaternion(1.0f, 0.0f, 0.0f, 0.0f);
        private static Quaternion q_ENU_to_NED = new Quaternion(0.70711f, 0.70711f, 0.0f, 0.0f);
        private Vector3 gravity = new Vector3(0.0f, 0.0f, -9.81f);
        private const double M_PI = 3.141926535;
        private const double rad2deg = 180 / M_PI;
        private const double deg2rad = M_PI / 180;
        private const float SAMPLING_RES = 10.0f;
        private const float SAMPLING_MIN_LAT = -60.0f;
        private const float SAMPLING_MAX_LAT = 60.0f;
        private const float SAMPLING_MIN_LON = -180.0f;
        private const float SAMPLING_MAX_LON = 180.0f;
        private SensorSource ss;
        enum SensorSource {
            ACCEL = 0b111,
            GYRO = 0b111000,
            MAG	= 0b111000000,
            BARO = 0b1101000000000,
            DIFF_PRESS = 0b10000000000,
        };

        private void Update()
        {
            current_transform = sensor.transform;
            current_position = sensor.transform.position;
            Vector3 global_vel = current_position - previous_position;
            local_vel = get_local_vel(global_vel);
            // Debug.Log("local_velocity " + local_vel);
            local_acc = get_local_acc(local_vel, prev_lvel);
            // Debug.Log("local_acceleration " + local_acc);
            prev_lvel = local_vel;
            local_ang_vel = get_local_ang_vel(sensor.GetComponent<Rigidbody>());
            Vector2 tmp_gps = gps.current_gps_rad;
            previous_position = current_position;
        }
        
        public void RosDataHandler()
        {          
            
        }

        public ros_geometry_pose RosPoseStamped()
        {
            /* @brief UNITY is in EUN LH while ROS is in NWU RH */
            ros_geometry_pose msg = new ros_geometry_pose(); /* Message is in NWU */
            msg.pose.position.x = current_position.z; 
            msg.pose.position.y = - current_position.x;
            msg.pose.position.z = current_position.y;

            Quaternion q = new Quaternion();
     
            if (ros_coordinate_frame == "nwu")
            {
                /* Rotate to nwu */
                Quaternion q_nwu_rh = quaternion_eun_lh_to_nwu_rh(current_transform.rotation);
                q = q_nwu_rh;
            }

            if (ros_coordinate_frame == "ned")
            {
                /* Rotate to ned */
                Quaternion q_nwu_rh = quaternion_eun_lh_to_nwu_rh(current_transform.rotation);
                Quaternion q_ned_rh = new Quaternion();
                q_ned_rh = q_nwu_rh * q_NWU_to_NED;
                q = q_ned_rh;
            }

            if (ros_coordinate_frame == "enu")
            {
                /* Rotate to enu */
                Quaternion q_nwu_rh = quaternion_eun_lh_to_nwu_rh(current_transform.rotation);
                Quaternion q_ned_rh = new Quaternion();
                q_ned_rh = q_nwu_rh * q_NWU_to_NED;
                Quaternion q_enu_rh = new Quaternion();
                q_enu_rh = q_ned_rh * Quaternion.Inverse(q_ENU_to_NED);
                q = q_enu_rh;
            }

            msg.pose.orientation.x = q.x;
            msg.pose.orientation.y = q.y;
            msg.pose.orientation.z = q.z;
            msg.pose.orientation.w = q.w;

            msg.header.frame_id = frame_id;
            
            return msg;
        }

        public MAVLink.mavlink_hil_sensor_t px4_full_sensor_data(bool imu, bool baro, bool mag)
        {
            MAVLink.mavlink_hil_sensor_t _sensor_data = new MAVLink.mavlink_hil_sensor_t();
            ulong ut = (ulong)(Time.time * 1e6);
            _sensor_data.time_usec = ut;
            _sensor_data = px4_imu_sensor_update(_sensor_data);
            _sensor_data = px4_baro_sensor_update(_sensor_data);
            _sensor_data = px4_mag_sensor_update(_sensor_data);

            return _sensor_data;
        }

        private MAVLink.mavlink_hil_sensor_t px4_imu_sensor_update(MAVLink.mavlink_hil_sensor_t _sensor_data)
        {
            /* Rotate to ned */
            Quaternion q_nwu_rh = quaternion_eun_lh_to_nwu_rh(new Quaternion());
            Quaternion q_ned_rh = new Quaternion();
            q_ned_rh = q_nwu_rh * q_NWU_to_NED;

            Vector3 ned_local_acc = q_ned_rh * local_acc;
            Vector3 ned_local_ang_vel = q_ned_rh * local_ang_vel;
            

            _sensor_data.xacc = ned_local_acc.x + rng(0, acc_noise);
            _sensor_data.yacc = ned_local_acc.y + rng(0, acc_noise);
            _sensor_data.zacc = ned_local_acc.z;

            _sensor_data.xgyro = ned_local_ang_vel.x + rng(0, gyro_noise);
            _sensor_data.ygyro = ned_local_ang_vel.y + rng(0, gyro_noise);
            _sensor_data.zgyro = ned_local_ang_vel.z + rng(0, gyro_noise);

            _sensor_data.fields_updated = _sensor_data.fields_updated | (uint)SensorSource.ACCEL | (uint)SensorSource.GYRO;
            
            return _sensor_data;
        }

        private MAVLink.mavlink_hil_sensor_t px4_baro_sensor_update(MAVLink.mavlink_hil_sensor_t _sensor_data)
        {
            float alt = current_transform.position.y;
            _sensor_data.temperature = get_temp_local(alt) + rng(0, baro_alt_noise);
            _sensor_data.abs_pressure = get_absolute_pressure(alt) + rng(0, baro_pressure_noise);
            _sensor_data.pressure_alt = get_pressure_altitude(alt) + rng(0, baro_pressure_noise);

            _sensor_data.fields_updated = _sensor_data.fields_updated | (uint)SensorSource.BARO;
            
            return _sensor_data;
        }

        private MAVLink.mavlink_hil_sensor_t px4_mag_sensor_update(MAVLink.mavlink_hil_sensor_t _sensor_data)
        {
            Vector2 gps_coord = new Vector2(tmp_gps.x * (float)(180 / MathUtils.M_PI), tmp_gps.y * (float)(180 / MathUtils.M_PI));
            Vector3 mag = get_mag(gps_coord);
            _sensor_data.xmag = mag.x + rng(0, mag_noise);
            _sensor_data.ymag = mag.y + rng(0, mag_noise);
            _sensor_data.zmag = mag.z + rng(0, mag_noise);

            _sensor_data.fields_updated = _sensor_data.fields_updated | (uint)SensorSource.MAG;
            
            return _sensor_data;
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
        
        private Vector3 get_local_acc(Vector3 local_vel, Vector3 prev_local_vel)
        {
            return (local_vel - prev_local_vel)/Time.deltaTime;
        }

        private Vector3 get_local_ang_vel(Rigidbody rb)
        {
            return transform.InverseTransformDirection(rb.angularVelocity);
        }

        private Vector3 get_local_vel(Vector3 vel)
        {
            // return transform.InverseTransformDirection(vel);
            return Quaternion.Inverse(current_transform.rotation) * vel;
        }

        private float rng(float mean, float max)
        {
            System.Random random = new System.Random();
            // Returns a random number bw -1.0 and 1.0
            return ((float)(random.NextDouble()*2) - 1.0f) * max + mean;
        }

        float get_absolute_pressure(float pose_n_z)
        {
            // calculate abs_pressure using an ISA model for the tropsphere (valid up to 11km above MSL)
            float lapse_rate = 0.0065f; // reduction in temperature with altitude (Kelvin/m)
            float temperature_msl = 288.0f; // temperature at MSL (Kelvin)
            float alt_msl = pose_n_z - 0;
            float temperature_local = temperature_msl - lapse_rate * alt_msl;
            float pressure_ratio = Mathf.Pow(temperature_msl / temperature_local, 5.256f);
            float pressure_msl = 101325.0f; // pressure at MSL
            float absolute_pressure = pressure_msl / pressure_ratio;
            absolute_pressure *=  0.01f;
            return absolute_pressure;
        }
        
        float get_pressure_altitude(float pose_n_z)
        {
            float lapse_rate = 0.0065f; // reduction in temperature with altitude (Kelvin/m)
            float temperature_msl = 288.0f; // temperature at MSL (Kelvin)
            float alt_msl = pose_n_z - 0;
            float temperature_local = temperature_msl - lapse_rate * alt_msl;
            float density_ratio = Mathf.Pow(temperature_msl / temperature_local, 4.256f);
            float rho = 1.225f / density_ratio;

            return alt_msl / (gravity.magnitude * rho);
            // can add noise abs_pressure_noise + baro_drift_pa_
            // gravity_W_.Length() = 3 since Vector3
        }

        float get_temp_local(float pose_n_z)
        {
            float lapse_rate = 0.0065f; // reduction in temperature with altitude (Kelvin/m)
            float temperature_msl = 288.0f; // temperature at MSL (Kelvin)
            float alt_msl = pose_n_z - 0;
            float temperature_local = temperature_msl - lapse_rate * alt_msl;
            return temperature_local - 273.0f;
        }

        private Vector3 get_mag(Vector2 gps)
        {
            Vector2 home_rad = new Vector2(gps.x * (float)MathUtils.deg2rad, gps.y * (float)MathUtils.deg2rad);
            float groundtruth_lat_rad_ = home_rad.x; float groundtruth_lon_rad_ = home_rad.y;

            // Magnetic declination and inclination (radians)
            float declination_rad = get_mag_declination(groundtruth_lat_rad_ * (float)MathUtils.rad2deg, groundtruth_lon_rad_ * (float)rad2deg) * (float)deg2rad;
            float inclination_rad = get_mag_inclination(groundtruth_lat_rad_ * (float)MathUtils.rad2deg, groundtruth_lon_rad_ * (float)rad2deg) * (float)deg2rad;

            // Magnetic strength (10^5xnanoTesla)
            float strength_ga = 0.01f * get_mag_strength(groundtruth_lat_rad_ * (float)MathUtils.rad2deg, groundtruth_lon_rad_ * (float)rad2deg);

            // Magnetic filed components are calculated by http://geomag.nrcan.gc.ca/mag_fld/comp-en.php
            float H = strength_ga * Mathf.Cos(inclination_rad);
            float Z = Mathf.Tan(inclination_rad) * H;
            float X = H * Mathf.Cos(declination_rad);
            float Y = H * Mathf.Sin(declination_rad);

            return new Vector3(X, Y, Z);
        }

        private static float get_table_data(float lat, float lon, int[,] table)
        {
            /*
            * If the values exceed valid ranges, return zero as default
            * as we have no way of knowing what the closest real value
            * would be.
            */
            if (lat < -90.0f || lat > 90.0f ||
                lon < -180.0f || lon > 180.0f) {
                return 0.0f;
            }

            /* round down to nearest sampling resolution */
            float min_lat = (int)(lat / SAMPLING_RES) * SAMPLING_RES;
            float min_lon = (int)(lon / SAMPLING_RES) * SAMPLING_RES;

            /* find index of nearest low sampling point */
            uint min_lat_index = get_lookup_table_index(min_lat, SAMPLING_MIN_LAT, SAMPLING_MAX_LAT);
            uint min_lon_index = get_lookup_table_index(min_lon, SAMPLING_MIN_LON, SAMPLING_MAX_LON);

            float data_sw = table[min_lat_index, min_lon_index];
            float data_se = table[min_lat_index, min_lon_index + 1];
            float data_ne = table[min_lat_index + 1, min_lon_index + 1];
            float data_nw = table[min_lat_index + 1, min_lon_index];

            /* perform bilinear interpolation on the four grid corners */
            float lat_scale = Mathf.Clamp((lat - min_lat) / SAMPLING_RES, 0.0f, 1.0f);
            float lon_scale = Mathf.Clamp((lon - min_lon) / SAMPLING_RES, 0.0f, 1.0f);

            float data_min = lon_scale * (data_se - data_sw) + data_sw;
            float data_max = lon_scale * (data_ne - data_nw) + data_nw;

            return lat_scale * (data_max - data_min) + data_min;
        }

        static int[,] declination_table = new int[13,37] {
            { 47,46,45,43,42,41,39,37,33,29,23,16,10,4,-1,-6,-10,-15,-20,-27,-34,-42,-49,-56,-62,-67,-72,-74,-75,-73,-61,-22,26,42,47,48,47 },
            { 31,31,31,30,30,30,30,29,27,24,18,11,3,-4,-9,-13,-15,-18,-21,-27,-33,-40,-47,-52,-56,-57,-56,-52,-44,-30,-14,2,14,22,27,30,31 },
            { 22,23,23,23,22,22,22,23,22,19,13,5,-4,-12,-17,-20,-22,-22,-23,-25,-30,-36,-41,-45,-46,-44,-39,-31,-21,-11,-3,4,10,15,19,21,22 },
            { 17,17,17,18,17,17,17,17,16,13,8,-1,-10,-18,-22,-25,-26,-25,-22,-20,-21,-25,-29,-32,-31,-28,-23,-16,-9,-3,0,4,7,11,14,16,17 },
            { 13,13,14,14,14,13,13,12,11,9,3,-5,-14,-20,-24,-25,-24,-21,-17,-12,-9,-11,-14,-17,-18,-16,-12,-8,-3,-0,1,3,6,8,11,12,13 },
            { 11,11,11,11,11,10,10,10,9,6,-0,-8,-15,-21,-23,-22,-19,-15,-10,-5,-2,-2,-4,-7,-9,-8,-7,-4,-1,1,1,2,4,7,9,10,11 },
            { 10,9,9,9,9,9,9,8,7,3,-3,-10,-16,-20,-20,-18,-14,-9,-5,-2,1,2,0,-2,-4,-4,-3,-2,-0,0,0,1,3,5,7,9,10 },
            { 9,9,9,9,9,9,9,8,6,1,-4,-11,-16,-18,-17,-14,-10,-5,-2,-0,2,3,2,0,-1,-2,-2,-1,-0,-1,-1,-1,1,3,6,8,9 },
            { 8,9,9,10,10,10,10,8,5,0,-6,-12,-15,-16,-15,-11,-7,-4,-1,1,3,4,3,2,1,0,-0,-0,-1,-2,-3,-4,-2,0,3,6,8 },
            { 7,9,10,11,12,12,12,9,5,-1,-7,-13,-15,-15,-13,-10,-6,-3,0,2,3,4,4,4,3,2,1,0,-1,-3,-5,-6,-6,-3,0,4,7 },
            { 5,8,11,13,14,15,14,11,5,-2,-9,-15,-17,-16,-13,-10,-6,-3,0,3,4,5,6,6,6,5,4,2,-1,-5,-8,-9,-9,-6,-3,1,5 },
            { 3,8,11,15,17,17,16,12,5,-4,-12,-18,-19,-18,-16,-12,-8,-4,-0,3,5,7,9,10,10,9,7,4,-1,-6,-10,-12,-12,-9,-5,-1,3 },
            { 3,8,12,16,19,20,18,13,4,-8,-18,-24,-25,-23,-20,-16,-11,-6,-1,3,7,11,14,16,17,17,14,8,-0,-8,-13,-15,-14,-11,-7,-2,3 },	
        };

        // inclination data in degrees
        static int[,] inclination_table = new int[13,37] {
            { -78,-76,-74,-72,-70,-68,-65,-63,-60,-57,-55,-54,-54,-55,-56,-57,-58,-59,-59,-59,-59,-60,-61,-63,-66,-69,-73,-76,-79,-83,-86,-87,-86,-84,-82,-80,-78 },
            { -72,-70,-68,-66,-64,-62,-60,-57,-54,-51,-49,-48,-49,-51,-55,-58,-60,-61,-61,-61,-60,-60,-61,-63,-66,-69,-72,-76,-78,-80,-81,-80,-79,-77,-76,-74,-72 },
            { -64,-62,-60,-59,-57,-55,-53,-50,-47,-44,-41,-41,-43,-47,-53,-58,-62,-65,-66,-65,-63,-62,-61,-63,-65,-68,-71,-73,-74,-74,-73,-72,-71,-70,-68,-66,-64 },
            { -55,-53,-51,-49,-46,-44,-42,-40,-37,-33,-30,-30,-34,-41,-48,-55,-60,-65,-67,-68,-66,-63,-61,-61,-62,-64,-65,-66,-66,-65,-64,-63,-62,-61,-59,-57,-55 },
            { -42,-40,-37,-35,-33,-30,-28,-25,-22,-18,-15,-16,-22,-31,-40,-48,-55,-59,-62,-63,-61,-58,-55,-53,-53,-54,-55,-55,-54,-53,-51,-51,-50,-49,-47,-45,-42 },
            { -25,-22,-20,-17,-15,-12,-10,-7,-3,1,3,2,-5,-16,-27,-37,-44,-48,-50,-50,-48,-44,-41,-38,-38,-38,-39,-39,-38,-37,-36,-35,-35,-34,-31,-28,-25 },
            { -5,-2,1,3,5,8,10,13,16,20,21,19,12,2,-10,-20,-27,-30,-30,-29,-27,-23,-19,-17,-17,-17,-18,-18,-17,-16,-16,-16,-16,-15,-12,-9,-5 },
            { 15,18,21,22,24,26,29,31,34,36,37,34,28,20,10,2,-3,-5,-5,-4,-2,2,5,7,8,7,7,6,7,7,7,6,5,6,8,11,15 },
            { 31,34,36,38,39,41,43,46,48,49,49,46,42,36,29,24,20,19,20,21,23,25,28,30,30,30,29,29,29,29,28,27,25,25,26,28,31 },
            { 43,45,47,49,51,53,55,57,58,59,59,56,53,49,45,42,40,40,40,41,43,44,46,47,47,47,47,47,47,47,46,44,42,41,40,42,43 },
            { 53,54,56,57,59,61,64,66,67,68,67,65,62,60,57,55,55,54,55,56,57,58,59,59,60,60,60,60,60,60,59,57,55,53,52,52,53 },
            { 62,63,64,65,67,69,71,73,75,75,74,73,70,68,67,66,65,65,65,66,66,67,68,68,69,70,70,71,71,70,69,67,65,63,62,62,62 },
            { 71,71,72,73,75,77,78,80,81,81,80,79,77,76,74,73,73,73,73,73,73,74,74,75,76,77,78,78,78,78,77,75,73,72,71,71,71 },
        };

        // strength data in centi-Tesla
        static int[,] strength_table = new int[13,37] {
            { 62,60,58,56,54,52,49,46,43,41,38,36,34,32,31,31,30,30,30,31,33,35,38,42,46,51,55,59,62,64,66,67,67,66,65,64,62 },
            { 59,56,54,52,50,47,44,41,38,35,32,29,28,27,26,26,26,25,25,26,28,30,34,39,44,49,54,58,61,64,65,66,65,64,63,61,59 },
            { 54,52,49,47,45,42,40,37,34,30,27,25,24,24,24,24,24,24,24,24,25,28,32,37,42,48,52,56,59,61,62,62,62,60,59,56,54 },
            { 49,47,44,42,40,37,35,33,30,28,25,23,22,23,23,24,25,25,26,26,26,28,31,36,41,46,51,54,56,57,57,57,56,55,53,51,49 },
            { 43,41,39,37,35,33,32,30,28,26,25,23,23,23,24,25,26,28,29,29,29,30,32,36,40,44,48,51,52,52,51,51,50,49,47,45,43 },
            { 38,36,35,33,32,31,30,29,28,27,26,25,24,24,25,26,28,30,31,32,32,32,33,35,38,42,44,46,47,46,45,45,44,43,41,40,38 },
            { 34,33,32,32,31,31,31,30,30,30,29,28,27,27,27,28,29,31,32,33,33,33,34,35,37,39,41,42,43,42,41,40,39,38,36,35,34 },
            { 33,33,32,32,33,33,34,34,35,35,34,33,32,31,30,30,31,32,33,34,35,35,36,37,38,40,41,42,42,41,40,39,37,36,34,33,33 },
            { 34,34,34,35,36,37,39,40,41,41,40,39,37,35,35,34,35,35,36,37,38,39,40,41,42,43,44,45,45,45,43,41,39,37,35,34,34 },
            { 37,37,38,39,41,42,44,46,47,47,46,45,43,41,40,39,39,40,41,41,42,43,45,46,47,48,49,50,50,50,48,46,43,41,39,38,37 },
            { 42,42,43,44,46,48,50,52,53,53,52,51,49,47,45,45,44,44,45,46,46,47,48,50,51,53,54,55,56,55,54,52,49,46,44,43,42 },
            { 48,48,49,50,52,53,55,56,57,57,56,55,53,51,50,49,48,48,48,49,49,50,51,53,55,56,58,59,60,60,58,56,54,52,50,49,48 },
            { 54,54,54,55,56,57,58,58,59,58,58,57,56,54,53,52,51,51,51,51,52,53,54,55,57,58,60,61,62,61,61,59,58,56,55,54,54 },
        };

        public static uint get_lookup_table_index(float val, float min, float max)
        {
            /* for the rare case of hitting the bounds exactly
            * the rounding logic wouldn't fit, so enforce it.
            */

            /* limit to table bounds - required for maxima even when table spans full globe range */
            /* limit to (table bounds - 1) because bilinear interpolation requires checking (index + 1) */
            val = Mathf.Clamp(val, min, max - SAMPLING_RES);

            return (uint)((-(min) + val) / SAMPLING_RES);
        }

        private static float get_mag_declination(float lat, float lon)
        {
            return get_table_data(lat, lon, declination_table);
        }

        private static float get_mag_inclination(float lat, float lon)
        {
            return get_table_data(lat, lon, inclination_table);
        }

        private static float get_mag_strength(float lat, float lon)
        {
            return get_table_data(lat, lon, strength_table);
        }
    
    }
}
