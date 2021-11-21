using System;
using UnityEngine;
using System.IO;
using System.Collections;
using System.Collections.Generic;

namespace RapidGUI.Example
{
    public class RguiCommand : MonoBehaviour
    {
        public float width = 400;
        public float height = 40;
        public enum CommandEnum
        {
            IDLE,
            TAKEOFF,
            HOVER,
            MISSION,
            HOME,
            LAND
        };
        public CommandEnum enumCmd;
        public sbyte cmdbyte;
        public Rect rect;
        float h = Screen.height; float w = Screen.width;

        private void Start()
        {
            // Top right hand corner
            rect = new Rect(w-width, 0, width, height);
            cmdbyte = new sbyte{};
        }

        private void OnGUI()
        {
            rect = RGUI.ResizableWindow(GetHashCode(), rect,
                (id) =>
                {
                    // GUILayout.Label("ResizableWindow");
                    GUI.DragWindow();
                    enumCmd = RGUI.Field(enumCmd, "Command");
                    cmdbyte = (sbyte)enumCmd;
                },
                "User Command");
            // CommandEnum = RGUI.Field(CommandEnum, "enumFlags");
        }
    }
}