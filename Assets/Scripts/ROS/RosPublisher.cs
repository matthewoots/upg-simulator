using System;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosImage = RosMessageTypes.Sensor.MImage;
using System.Runtime.InteropServices;
using System.Runtime.Serialization.Formatters.Binary;
using System.IO;

// using RosColor = RosMessageTypes.RoboticsDemo.MUnityColor;

[StructLayout(LayoutKind.Sequential, Pack = 1)] 

public class RosPublisher : MonoBehaviour
{
    ROSConnection ros;
    public GameObject uav;
    public bool isStereo;
    public Camera cameraLeft;
    public Camera cameraRight;
    public int cameraWidth = 1440;
    public int cameraHeight = 1080;
    public float publishMessageFrequency = 15f;
    private float timeElapsed;
    private Vector3 origin;

    private uint ReturnImageWidth(Camera cam) { return (uint)cam.pixelWidth;}
    private uint ReturnImageHeight(Camera cam) { return (uint)cam.pixelHeight;}

    void Start()
    {
        // start the ROS connection
        ros = ROSConnection.instance;
    }

    private void Update()
    {
        timeElapsed += Time.deltaTime;

        if (isStereo)
        {
            // Stereo Image Format
            Texture2D imageLeft = ReturnRGBImage(cameraLeft,cameraWidth,cameraHeight);
            Texture2D imageRight = ReturnRGBImage(cameraRight,cameraWidth,cameraHeight);
            RosImage leftImage = CopyData(cameraLeft, imageLeft.GetRawTextureData(),"rgb8", true); // Use GetRawTextureData since EncodeToPNG or JPG does not import the whole size of the data
            RosImage rightImage = CopyData(cameraRight, imageRight.GetRawTextureData(),"rgb8", true); // Use GetRawTextureData since EncodeToPNG or JPG does not import the whole size of the data
            ros.Send("Image_Left", leftImage);
            ros.Send("Image_Right", rightImage);
            // Debug.Log("Image datasize = " + rightImage.data.Length * sizeof(byte) / 10e6 + "mb");
        }

    }

    private Texture2D ReturnRGBImage(Camera cam, int imageWidth, int imageHeight)
    {
        RenderTexture rt = new RenderTexture(imageWidth, imageHeight, 0);
        cam.targetTexture = rt;
        RenderTexture currentRT = RenderTexture.active;
        RenderTexture.active = cam.targetTexture;
        cam.Render();

        Texture2D image = new Texture2D(cam.targetTexture.width, cam.targetTexture.height, TextureFormat.RGB24, false);
        image.ReadPixels(new Rect(0, 0, cam.targetTexture.width, cam.targetTexture.height), 0, 0);
        image.Apply();
        // Debug.Log("Intemediate RGB datasize = " + bytes.Length + " : " + ReturnImageWidth(cam) * 3 * ReturnImageHeight(cam));
        RenderTexture.active = currentRT;
        Destroy(rt);
        Destroy(image);
        return image;
    }

    private RosImage CopyData(Camera cam, byte[] Data, string format, bool isrgb)
    {
        byte[] tmp = new byte[Data.Length];
        RosImage ImageData = new RosImage();
        ImageData.height = ReturnImageHeight(cam);
        ImageData.width = ReturnImageWidth(cam);
        ImageData.encoding = format;
        ImageData.is_bigendian = 0;
        if (isrgb) ImageData.step = ReturnImageWidth(cam) * 3;
        else ImageData.step = ReturnImageWidth(cam);
        ImageData.data = Data;
        // Debug.Log("datasize: " + Data.Length + " width: "+ ImageData.width + " height: " + ImageData.height);
        return ImageData;      
    }

    private void restrictMsgRate(RosImage leftImage, RosImage rightImage)
    {
        if (timeElapsed > 1/publishMessageFrequency)
        {
            // Finally send the message to server_endpoint.py running in ROS
            ros.Send("Image_Left", leftImage);
            ros.Send("Image_Right", rightImage);

            timeElapsed = 0;
        }
    }
}
