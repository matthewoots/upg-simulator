using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using System;

public class PointCloudGenerator : MonoBehaviour
{
    public Vector3 range = new Vector3(10.0f, 10.0f, 8.0f); 
    public Vector3 origin = Vector3.zero;
    public List<Vector3> points;
    public float resolution = 0.05f; // Step for 1m
    public GameObject[] obj;
    public Button button;
    public bool debug;
    
    void Start()
    {
        Button btn = button.GetComponent<Button>();
		btn.onClick.AddListener(TaskOnClick);
    }
    
    void TaskOnClick()
    {

        int x_size = (int)Math.Ceiling((range.x / resolution));
        int y_size = (int)Math.Ceiling((range.y / resolution));
        int z_size = (int)Math.Ceiling((range.z / resolution));

        float sphere_reso = resolution/2;

        points = new List<Vector3>(); 

        // z axis
	    for (int i = z_size; i >= 0; i--)
        {
            for (int j = x_size; j >= 0; j--)
            {
                for (int k = y_size; k >= 0; k--)
                {                    
                    float x = k * resolution - (range.y/2) + origin.y;
                    float y = i * resolution;
                    float z = j * resolution - (range.x/2) + origin.x;
                    Vector3 p = new Vector3(x,y,z);
                    Collider[] hitColliders = Physics.OverlapSphere(p, sphere_reso - 0.0001f);
                    if (hitColliders.Length == 0)
                        continue;

                    int skipcollider = hitColliders.Length;
                    for (int n = 0; n < hitColliders.Length; n++)
                    {
                        for (int m = 0; m < obj.Length; m++)
                        if (hitColliders[n].gameObject.GetInstanceID() == obj[m].GetInstanceID())
                        {
                            skipcollider--;
                            continue;
                        }
                    }
                    if (skipcollider == 0)
                        continue;

                    points.Add(new Vector3(x,y,z));
                }                
            }
        }
        
        if (debug) {Debug.Log("[size] " + points.Count);}
        for (int k = 0; k < points.Count; k++)
        {
            if (debug) {Debug.Log("[point] " + k + " [Vector] " + points[k]);} 
        }          
    }
  
}
