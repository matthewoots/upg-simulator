/**
* @author Matthew Woo
* @contact matthewoots@gmail.com
* @year 2022
**/

using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class PointCloudGenerator : MonoBehaviour
{
    public Vector3 range = new Vector3(10.0f, 10.0f, 8.0f); 
    public Vector3 origin = Vector3.zero; 
    public List<Vector3> points; /* @brief Pointcloud points */
    public float vertical_resolution = 0.08f; /* @brief Step for 1.0m in the Z plane */
    public float horizontal_resolution = 0.025f; /* @brief Step for 1.0m in the XY plane */
    public GameObject[] obj; /* @brief Objects to omit */
    public Button button; /* @brief Button to start the scan */
    public bool debug = false; /* @brief Only for debugging purposes */
    
    /* @brief Start button callback upon start */
    void Start()
    {
        Button btn = button.GetComponent<Button>();
		btn.onClick.AddListener(TaskOnClick);
    }
    
    /* @brief We will scan only with TaskOnClick instance */
    void TaskOnClick()
    {

        int x_size = (int)Math.Ceiling((range.x / horizontal_resolution));
        int y_size = (int)Math.Ceiling((range.y / horizontal_resolution));
        int z_size = (int)Math.Ceiling((range.z / vertical_resolution));

        float sphere_reso = horizontal_resolution/2;

        points = new List<Vector3>(); 

        /* Z axis will be the last */
	    for (int i = z_size; i >= 0; i--)
        {
            for (int j = x_size; j >= 0; j--)
            {
                for (int k = y_size; k >= 0; k--)
                {                    
                    float x = k * horizontal_resolution - (range.y/2) + origin.y; /* Unity X is ROS Y axis */
                    float y = i * vertical_resolution;
                    float z = j * horizontal_resolution - (range.x/2) + origin.x; /* Unity Z is ROS X axis */
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
        
        if (debug) 
        {
            Debug.Log("[Point Cloud Size] " + points.Count);
            for (int k = 0; k < points.Count; k++)
            {
                if (debug) {Debug.Log("[Point Cloud Point] " + k + " [Point Cloud Vector] " + points[k]);} 
            }      
        }    
    }
  
}
