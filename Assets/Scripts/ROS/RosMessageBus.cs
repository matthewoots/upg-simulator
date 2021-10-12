using UnityEngine;
using System.Collections;

public class RosMessageBus : MonoBehaviour
{
    public float voltage;
    public float current;
    public float percentage;
    public UnityEngine.Vector3 local_pos;
    public UnityEngine.Vector3 local_rot;
    public UnityEngine.Vector3 local_vel;
    public bool connected;
    public bool armed;
    public string mode;
    public double latitude;
    public double longitude;
    public double altitude;

    void Start()
    {
        
    }
    void Update()
    {
        Debug.Log("Position: [" + local_pos + "]"); 
        Debug.Log("Global Pos: [" + latitude + " " + longitude + "]");
        Debug.Log("Rotation: [" + local_rot + "]");
    }


}