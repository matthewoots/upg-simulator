using System;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosPCL = RosMessageTypes.Sensor.PointCloudMsg;
using RosPoint32 = RosMessageTypes.Geometry.Point32Msg;
using System.IO;
using System.Collections;
using System.Collections.Generic;

public class RosPointCloudPublisher : MonoBehaviour
{
    public ROSConnection ros;
    float timeElapsed = 0;
    // public RaycastExample raycast;
    public PointCloudGenerator raycast;
    public string topic = "pcl";
    public float publishMessageFrequency = 0.5f;
    public string frameid = "/map";

    void Start()
    {
        // Start the ROS connection
        ros.RegisterPublisher<RosPCL>(topic);
    }

    private void Update()
    {
        timeElapsed += Time.deltaTime;

        if (timeElapsed > 1/publishMessageFrequency)
            timeElapsed = 0;
        else
            return;
        
        RosPCL pcl = new RosPCL();
        List<Vector3> tmp = raycast.points;
        RosPoint32[] data = new RosPoint32[tmp.Count];
        // Debug.Log(data.Length);
        for (int k = 0; k < tmp.Count; k++)
        {
            data[k] = new RosPoint32();
            // Debug.Log(tmp[k].x);
            // Debug.Log(data[k].x);
            // RUF to RFU(ENU)
            data[k].x = tmp[k].x;
            data[k].y = tmp[k].z;
            data[k].z = tmp[k].y;
        }
        pcl.header.frame_id = frameid;
        pcl.points = data;

        ros.Send(topic, pcl);
    }

}
