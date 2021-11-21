using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

public class RaycastExample : MonoBehaviour
{
    float timeElapsed = 0;
    Vector3[] array;
    public List<Vector3> points;
    public float dt = 2f;
    public int size_xy = 20;
    public int size_z = 15;
    public int step = 4; // Step for 1m
    public GameObject[] obj;
    public bool debug;
    

    void Update()
    {
        timeElapsed += Time.deltaTime;

        if (timeElapsed > dt)
            timeElapsed = 0;
        else
            return;

        float interval = 1/(float)step;
        int xy_int = 2 * size_xy * step;
        int z_int = size_z * step;

        points = new List<Vector3>(); 

        // z axis
	    for (int k = z_int; k >= 0; k--)
        {
            // We can scan forward to back, since the scan covers left to right
            for (int j = xy_int; j >= 0; j--)
            {
                // List<Tuple<int, Vector3>> data = new List<Tuple<int, Vector3>>{};
                Vector3 origin0 = new Vector3(size_xy, interval * k, size_xy - interval * j);
                Vector3 origin1 = new Vector3(-size_xy, interval * k, size_xy - interval * j);
                // Container for hit data
                RaycastHit[] hits0; // Forward ray (Direction towards -X)
                RaycastHit[] hits1; // Reverse ray to close up the contacts (Direction towards X)
                // Currently without reverse, it is in order
                
                float dist = size_xy * 2;
                hits0 = Physics.RaycastAll(origin0, Vector3.left, dist);
                hits1 = Physics.RaycastAll(origin1, Vector3.right, dist);
                // System.Array.Reverse(hits1);

                // Debug
                Debug.DrawRay(origin0, Vector3.left * dist, Color.green);
                Debug.DrawRay(origin1, Vector3.right * dist, Color.red);
  
                bool skipcollider;

                // Check if instance id match
                for (int i = 0; i < hits0.Length; i++)
                {
                    skipcollider = false;
                    for (int n = 0; n < obj.Length; n++)
                    {
                        if (hits0[i].collider.gameObject.GetInstanceID() == obj[n].GetInstanceID())
                            skipcollider = true;
                    }
                    if (skipcollider)
                        continue;

                    for (int l = 0; l < hits1.Length; l++)
                    {
                        skipcollider = false;
                        for (int n = 0; n < obj.Length; n++)
                        {
                            if (hits1[l].collider.gameObject.GetInstanceID() == obj[n].GetInstanceID())
                                skipcollider = true;
                        }
                        if (skipcollider)
                            continue;
                        if (hits0[i].collider.gameObject.GetInstanceID() !=
                            hits1[l].collider.gameObject.GetInstanceID())
                            continue;
                        
                        RaycastHit hit_rev = hits1[l]; RaycastHit hit_fwd = hits0[i];
                        float diff = hit_fwd.point.x - hit_rev.point.x;
                        
                        if (debug) {Debug.Log("[diff] " + diff);}

                        int sizeofpoints = (int)System.Math.Ceiling(diff/interval);
                        for (int m = 0; m < sizeofpoints; m++)
                        {
                            points.Add(new Vector3(hit_rev.point.x + interval*(m-1), interval * k, size_xy - interval * j));
                        }
                    }
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
