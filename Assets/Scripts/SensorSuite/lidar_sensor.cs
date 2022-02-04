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

using ros_sensor_pcl = RosMessageTypes.Sensor.PointCloudMsg;
using ros_geometry_point = RosMessageTypes.Geometry.Point32Msg;
using ros_geometry_pose = RosMessageTypes.Geometry.PoseStampedMsg;


namespace sensors_suite {
    public class lidar_sensor : MonoBehaviour
    {
        public GameObject obj;
        public string frame_id = "/map";
        
        [Header("Scan Topic Parameters")]
        public float scan_publish_message_frequency = 15.0f;
        public float scan_time_elapsed = 0.0f;
        public string scan_topic = "lidar/scan";

        [Header("Pose Topic Parameters")]
        public float pose_publish_message_frequency = 50.0f;
        public float pose_time_elapsed = 0.0f;
        public string pose_topic = "lidar/pose";

        [Header("Ouster Specific Parameters")]
        public int rotation_rate = 10; /* @brief OS1-16 10 or 20 Hz */
        public double total_vertical_fov_deg = 32.2; /* @brief OS1-16 32.2 degrees */
        public double total_horizontal_fov_deg = 360.0; /* @brief OS1-16 360 degrees */
        public double scan_range = 40.0; /* @brief OS1-16 maximum scan range 40.0m */
        public double resolution = 0.012; /* @brief OS1-16 range resolution 1.20cm */
        public int vertical_scan_lines = 16; /* @brief OS1-16 16 lines for vertical scan */
        public int horizontal_scan_lines = 512; /* @brief OS1-16 512, 1024, or 2048 */

        [Header("Private Parameters")]
        private int total_ray_count;
        private Vector3[] scan_vector_array;
        private Transform current_transform;
        private Vector3 forward_vector, current_position;
        private Vector3 global_forward_vector = Vector3.forward;

        public static Quaternion q_NWU_to_NED = new Quaternion(1.0f, 0.0f, 0.0f, 0.0f);

        private void Update()
        {
            current_transform = obj.transform;
            forward_vector = Vector3.forward;

            current_position = obj.transform.position;
            Debug.DrawRay(current_position, forward_vector * 2, Color.red);
            // for (int k = 0; k < total_ray_count; k++)
            // {
            //     Debug.DrawRay(current_position, scan_vector_array[k], Color.green);
            // }
        }

        public void initialize()
        {
            total_ray_count = horizontal_scan_lines * vertical_scan_lines;
            double vertical_division = total_vertical_fov_deg / vertical_scan_lines;
            double horizontal_division = total_horizontal_fov_deg / horizontal_scan_lines;
            double base_vertical_deg = -(total_vertical_fov_deg / 2);
            scan_vector_array = new Vector3[total_ray_count];
            
            for (int i = 0; i < vertical_scan_lines; i++)
            {
                double vertical_deg = vertical_division * i + base_vertical_deg;

                for (int j = 0; j < horizontal_scan_lines; j++)
                {
                    double horizontal_deg = horizontal_division * j;
                    Vector3 relative_vector = forward_vector;
                    scan_vector_array[i*horizontal_scan_lines + j] = Quaternion.Euler
                        (-(float)vertical_deg, (float)horizontal_deg, 0) * relative_vector;
                    scan_vector_array[i*horizontal_scan_lines + j] = transform.rotation * 
                        scan_vector_array[i*horizontal_scan_lines + j];
                }
            }

        }

        public ros_sensor_pcl RosDataHandler()
        {            
            ros_sensor_pcl pcl = new ros_sensor_pcl();
            List<Vector3> vector_points = new List<Vector3>();

            int pcl_count = 0;
            for (int k = 0; k < total_ray_count; k++)
            {
                RaycastHit hit;
                
                if (Physics.Raycast(current_position, scan_vector_array[k], out hit, Mathf.Round((float)scan_range), 1))
                {
                    vector_points.Add(new Vector3(hit.point.x, hit.point.y, hit.point.z));
                    pcl_count++;
                }
                else
                    continue;
            }
            // Debug.Log("Point Cloud Size @ " + pcl_count);
            // Debug.Log("Scan Range @ " + Mathf.Round((float)scan_range));
            ros_geometry_point[] data = new ros_geometry_point[pcl_count];

            for (int k = 0; k < pcl_count; k++)
            {
                data[k] = new ros_geometry_point();

                /* @brief UNITY is in EUN LH to ROS in NWU RH */
                data[k].x = vector_points[k].z - current_position.z;
                data[k].y = - (vector_points[k].x - current_position.x);
                data[k].z = vector_points[k].y - current_position.y;
            }
            pcl.header.frame_id = frame_id;
            pcl.points = data;

            return pcl;
        }

        public ros_geometry_pose RosPoseStamped()
        {
            /* @brief UNITY is in EUN LH while ROS is in NWU RH */
            ros_geometry_pose msg = new ros_geometry_pose(); /* Message is in NWU */
            msg.pose.position.x = current_position.z; 
            msg.pose.position.y = - current_position.x;
            msg.pose.position.z = current_position.y;

            /* Rotate to nwu */
            Quaternion q_nwu_rh = quaternion_eun_lh_to_nwu_rh(current_transform.rotation);

            // Quaternion ned_rotation = new Quaternion();
            // ned_rotation = q_ned_rh * q_NWU_to_NED;

            msg.pose.orientation.x = q_nwu_rh.x;
            msg.pose.orientation.y = q_nwu_rh.y;
            msg.pose.orientation.z = q_nwu_rh.z;
            msg.pose.orientation.w = q_nwu_rh.w;

            msg.header.frame_id = frame_id;
            
            return msg;
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