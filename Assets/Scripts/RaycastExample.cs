using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RaycastExample : MonoBehaviour
{
     bool logged;
     public int size_xy = 2;
     public int size_z = 1;
     public int step = 2; // Step for 1m
     
 
     void Update()
     {
         float interval = 1/(float)step;
         int xy_int = 2 * size_xy * step;
         int z_int = size_z * step;
	 for (int k = z_int; k >= 0; k--)
         {
            // We can scan forward to back, since the scan covers left to right
	    for (int j = xy_int; j >= 0; j--)
	    {
                Vector3 origin = new Vector3(size_xy, interval * k, size_xy - interval * j);
	        // Container for hit data
	        RaycastHit[] hits;
	        float dist = size_xy * 2;
                hits = Physics.RaycastAll(origin, Vector3.left, dist);
                Debug.DrawRay(origin, Vector3.left * dist, Color.green);

                for (int i = 0; i < hits.Length; i++)
                {
                   RaycastHit hit = hits[i];
                   Debug.Log("Raycasting idx " + i + " point " + hit.point);
	        }
	    }
         }

     }
 }
