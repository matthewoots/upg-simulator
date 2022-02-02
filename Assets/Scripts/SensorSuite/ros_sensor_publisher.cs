using System;
using System.IO;
using System.Collections;
using System.Collections.Generic;

using UnityEngine;
using Unity.Robotics.ROSTCPConnector;

using ros_sensor_pcl = RosMessageTypes.Sensor.PointCloudMsg;
using ros_geometry_point = RosMessageTypes.Geometry.Point32Msg;
using ros_sensor_image = RosMessageTypes.Sensor.ImageMsg;
using ros_geometry_pose = RosMessageTypes.Geometry.PoseStampedMsg;

using sensors_suite;

public class ros_sensor_publisher : MonoBehaviour
{
    public ROSConnection ros;
    public bool send_lidar;
    public bool send_image;
    public sensors_suite.lidar_sensor lidar;
    public sensors_suite.image_sensor image;
    public sensors_suite.global_pcl_sensor global_pcl;
    private bool initialized;

    private void initialization()
    {
        if (Time.time < 2.0f)
            return;

        if (lidar != null)
        {
            sensors_suite.lidar_sensor lidar_tmp = lidar;
            ros.RegisterPublisher<ros_sensor_pcl>(lidar_tmp.scan_topic);
            ros.RegisterPublisher<ros_geometry_pose>(lidar_tmp.pose_topic);
        }

        if (image != null)
        {
            sensors_suite.global_pcl_sensor global_pcl_tmp = global_pcl;
            ros.RegisterPublisher<ros_sensor_pcl>(global_pcl_tmp.scan_topic);
        }

        if (global_pcl != null)
        {
            sensors_suite.image_sensor image_tmp = image;
            ros.RegisterPublisher<ros_sensor_image>(image_tmp.topic);
        }

        initialized = true;
    }

    private void lidar_handler()
    {
        lidar.scan_time_elapsed += Time.deltaTime;
        lidar.pose_time_elapsed += Time.deltaTime;
        if (lidar.scan_time_elapsed > 1 / lidar.scan_publish_message_frequency)
        {
            lidar.scan_time_elapsed = 0;
            lidar.initialize();
            ros_sensor_pcl pcl = lidar.RosDataHandler();
            ros.Send(lidar.scan_topic, pcl);
        }

        if (lidar.pose_time_elapsed > 1 / lidar.pose_publish_message_frequency)
        {
            lidar.pose_time_elapsed = 0;
            ros_geometry_pose pose = lidar.RosPoseStamped();
            ros.Send(lidar.pose_topic, pose);
        }
        // Debug.Log("Sent Point Cloud @" + Time.time);
    }

    private void image_handler()
    {

    }

    private void global_pcl_handler()
    {

    }


    private void Update()
    {
        if (!initialized)
        {
            initialization();
            return;
        }
        if (lidar != null)
            lidar_handler();

        if (image != null)
            image_handler();

        if (global_pcl != null)
            global_pcl_handler();
        
    }

}
