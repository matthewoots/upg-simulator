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
    public class global_pcl_sensor : MonoBehaviour
    {
        public string frame_id = "/map";
        public string scan_topic = "global_pcl";
        public Vector3 scan_range = new Vector3(10.0f, 10.0f, 8.0f); 
        public Vector3 scan_origin = Vector3.zero; 
        public float vertical_resolution = 0.1f; /* @brief Step for 1.0m in the Z plane */
        public float horizontal_resolution = 0.05f; /* @brief Step for 1.0m in the XY plane */
        public GameObject[] obj; /* @brief Objects to omit */
        
        public ros_sensor_pcl RosDataHandler()
        {          
            ros_sensor_pcl pcl = new ros_sensor_pcl();
            List<Vector3> pcl_list = PclExtraction();
            ros_geometry_point[] data = new ros_geometry_point[pcl_list.Count];
            for (int k = 0; k < pcl_list.Count; k++)
            {
                data[k] = new ros_geometry_point();

                /* @brief UNITY is in EUN LH to ROS in NWU RH */
                data[k].x = pcl_list[k].z;
                data[k].y = - pcl_list[k].x;
                data[k].z = pcl_list[k].y;
            }

            pcl.header.frame_id = frame_id;
            pcl.points = data;

            return pcl;
        }
        
        public List<Vector3> PclExtraction()
        { 
            int x_size = (int)Math.Ceiling((scan_range.x / horizontal_resolution));
            int y_size = (int)Math.Ceiling((scan_range.y / horizontal_resolution));
            int z_size = (int)Math.Ceiling((scan_range.z / vertical_resolution));

            float sphere_resolution = horizontal_resolution/2;

            List<Vector3> points = new List<Vector3>(); 

            /* Z axis will be the last */
            for (int i = z_size; i >= 0; i--)
            {
                for (int j = x_size; j >= 0; j--)
                {
                    for (int k = y_size; k >= 0; k--)
                    {                    
                        float x = k * horizontal_resolution - (scan_range.y/2) + scan_origin.y; /* Unity X is ROS Y axis */
                        float y = i * vertical_resolution;
                        float z = j * horizontal_resolution - (scan_range.x/2) + scan_origin.x; /* Unity Z is ROS X axis */
                        
                        Vector3 p = new Vector3(x,y,z);
                        Collider[] hitColliders = Physics.OverlapSphere(p, sphere_resolution - 0.0001f);
                        if (hitColliders.Length == 0)
                            continue;

                        int skipcollider = hitColliders.Length;
                        if (skipcollider == 0)
                            continue;
                        for (int n = 0; n < hitColliders.Length; n++)
                        {
                            for (int m = 0; m < obj.Length; m++)
                            if (hitColliders[n].gameObject.GetInstanceID() == obj[m].GetInstanceID())
                            {
                                skipcollider--;
                                continue;
                            }
                        }

                        points.Add(new Vector3(x,y,z));
                    }                                 
                }
            }
            return points; 
        }
    
    }
}
