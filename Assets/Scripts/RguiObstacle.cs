using System;
using UnityEngine;
using System.IO;
using System.Collections;
using System.Collections.Generic;

namespace RapidGUI.Example
{
    public class RguiObstacle : MonoBehaviour
    {
        public float width = 400;
        public float height = 40;
        public float extra = 40;
        public List<GameObject> listVal;

        // public int range;

        public enum ObsEnum
        {
            BEAM,
            MAZE0,
            MAZE1
        };
        public ObsEnum obsEnum;
        public RguiCommand command;
        Rect rect;
        float h = Screen.height; float w = Screen.width;

        private void Start()
        {
            Rect rect_tmp = command.rect;
            float width_tmp = command.width;
            float height_tmp = command.height;
            // Top right hand corner below command
            rect = new Rect(w-width, 0+height_tmp+extra, width, height);            
        }

        private void OnGUI()
        {
            rect = RGUI.ResizableWindow(GetHashCode(), rect,
                (id) =>
                {
                    // string n1 = listVal[range].name;
                    // int ne1 = RGUI.Field(range, n1);
                    
                    obsEnum = RGUI.Field(obsEnum, "Choose Obs");
                    for (int i = 0; i < listVal.Count; i++)
                    {
                        if ((int)obsEnum == i)
                            listVal[i].SetActive(true);
                        else
                            listVal[i].SetActive(false);
                    }
                    // GUILayout.Label("ResizableWindow");
                    GUI.DragWindow();
                    listVal = RGUI.Field(listVal, "list of Obstacles");
                },
                "Obstacle Course");
            // CommandEnum = RGUI.Field(CommandEnum, "enumFlags");
        }
    }
}