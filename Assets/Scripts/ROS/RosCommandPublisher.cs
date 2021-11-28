using System;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosCmd = RosMessageTypes.Std.ByteMsg;
using System.IO;
using System.Collections;
using System.Collections.Generic;
using RapidGUI.Example;

public class RosCommandPublisher : MonoBehaviour
{
    public ROSConnection ros;
    public RguiCommand rgui;
    public string topic = "cmd";
    sbyte previousCmd;
    float starttimer = 0;

    void Start()
    {
        // Start the ROS connection
        ros.RegisterPublisher<RosCmd>(topic);
    }

    private void Update()
    {
        // Give 2 secs to initialize
        starttimer += Time.deltaTime;

        if (starttimer <= 2)
            return;
        // This would prevent object not set to an instance since cmd gui would have set up by them

        RosCmd d = new RosCmd();
        sbyte cmd = new sbyte{};
        cmd = rgui.cmdbyte;
        if (cmd != previousCmd)
        {
            d.data = cmd;
            ros.Send(topic, d);
            previousCmd = cmd;
        }    
    }

}