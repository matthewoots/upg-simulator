using System;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosImage = RosMessageTypes.Sensor.ImageMsg;
using System.IO;

public class RosImagePublisher : MonoBehaviour
{
    ROSConnection ros;
    float timeElapsed = 0;
    public Camera cam;
    public string topic = "image";
    public int cameraWidth = 1440;
    public int cameraHeight = 1080;
    public float publishMessageFrequency = 15f;
    
    private uint ReturnImageWidth(Camera cam) { return (uint)cam.pixelWidth;}
    private uint ReturnImageHeight(Camera cam) { return (uint)cam.pixelHeight;}

    void Start()
    {
        // Start the ROS connection
        ros = ROSConnection.instance;
    }

    private void Update()
    {
        timeElapsed += Time.deltaTime;

        if (timeElapsed > 1/publishMessageFrequency)
            timeElapsed = 0;
        else
            return;

        // Image Format
        Texture2D imageTexture = ReturnRGBImage(cam,cameraWidth,cameraHeight);
        // Use GetRawTextureData since EncodeToPNG or JPG does not import the whole size of the data
        RosImage imageRos = CopyData(cam, imageTexture.GetRawTextureData(),"rgb8", true); 
        ros.Send(topic, imageRos);
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

}
