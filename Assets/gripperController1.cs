// Original Author: Long Qian
// Email: lqian8@jhu.edu
// Modified by Shelly Bagchi

using System;
using System.Linq;
using UnityEngine;
using UnityEngine.UI;
//using UnityStandardAssets.CrossPlatformInput;

// Needed //////////////////////////////////////////////////
//using HoloLensXboxController;
///////////////////////////////////////////////////////////

public class gripperController1 : MonoBehaviour
{

    public GameObject RobotBase;
    private float[] jointValues = new float[7];
    private GameObject[] jointList = new GameObject[9]; // new gripper joint
    //private float[] upperLimit_r = { 180f, 180f, 180f, 180f, 180f, 180f };
    //private float[] lowerLimit_r = { -180f, -180f, -180f, -180f, -180f, -180f };

    private float[] upperLimit_r = { 175f, 90f, 160f, 175f, 175f, 175f, 20f};
    private float[] lowerLimit_r = { -175f, -90f, -160f, -175f, -175f, -175f, -20f};
    // Using actual joint limits, -5 degrees to avoid stops
    // 0 == base
    //private float[] upperLimit_r = { 355f, 0f, 155f, 345f, 345f, 345f };
    //private float[] lowerLimit_r = { -355f, -175f, -155f, -345f, -345f, -345f };

    // With offsets
    private float[] upperLimit_s = { 400f, 90f, 155f, 435f, 345f, 345f, 180f };
    private float[] lowerLimit_s = { -310f, -85f, -155f, -255f, -345f, -345f, -180f };

    public GameObject CanvasObj;
    private Slider[] sliderList = new Slider[7];

    public InputField TextControl;
    public Toggle TextToggle;

    public float[] getJointValues()
    {
        return jointValues;
    }

    private void setJointValues(float[] input)
    {
        jointValues = input;
    }

    public GameObject[] getJointList()
    {
        return jointList;
    }

    public Slider[] getSliderList()
    {
        return sliderList;
    }

    public void setSliderList(float[] values)
    {
        for (int i = 0; i < 7; i++)
        {
            sliderList[i].value = values[i];
        }
    }

    // Needed //////////////////////////////////////////////////
    //private ControllerInput controllerInput;
    ///////////////////////////////////////////////////////////

    // Use this for initialization
    void Start()
    {
        initializeJoints(jointList);
        initializeSliders(sliderList);

        TextControl.text = "(0,0,0,0,0,0,0)";
        //define the initial pose
        for (int i = 0; i < 9; i++)
        {
            Vector3 currentRotation = jointList[i].transform.localEulerAngles;

            //Debug.Log(currentRotation);

            //difine the rotation direction
            //Quaternion currentRotation = jointList[i].transform.localRotation;
            switch (i)
            {
                case 0:
                    currentRotation.y = jointValues[i];
                    break;
                case 1:
                    currentRotation.x = jointValues[i]; //the link 2 have some prolem, so i add more constrain
                    currentRotation.y = 0;
                    currentRotation.z = -90f;
                    break;
                case 2:
                    currentRotation.y = jointValues[i];
                    break;
                case 3:
                    currentRotation.y = jointValues[i];
                    break;
                case 4:
                    currentRotation.x = jointValues[i];
                    break;
                case 5:
                    currentRotation.x = jointValues[i];
                    break;
                case 6:
                    currentRotation.x = jointValues[i];
                    break;
                case 7:
                    currentRotation.x = jointValues[6];
                    break;
                case 8:
                    currentRotation.x = jointValues[6];
                    break;
            }
            //jointList[i].transform.localEulerAngles = currentRotation;   //perform the rotation 
            //jointList[i].transform.localRotation = currentRotation;
        }

        // Needed //////////////////////////////////////////////////
        //controllerInput = new ControllerInput(0, 0.19f);
        // First parameter is the number, starting at zero, of the controller you want to follow.
        // Second parameter is the default “dead” value; meaning all stick readings less than this value will be set to 0.0.
        ///////////////////////////////////////////////////////////

    }

    // Update is called once per frame
    void Update()
    {
        //TextControl.text = string.Format("({0:0.0}, {1:0.0}, {2:0.0}, {3:0.0}, {4:0.0}, {5:0.0})",
        //    jointValues[5], jointValues[4], jointValues[3],
        //    jointValues[2], jointValues[1], jointValues[0]);

        // Needed //////////////////////////////////////////////////
        //controllerInput.Update();
        ///////////////////////////////////////////////////////////
    }

    // the classic way to perform rotation of joints
    private void FixedUpdate()
    {
        /*
        for (int i = 0; i < 7; i++)
        {
            Vector3 currentRotation = jointList[i].transform.localEulerAngles;

            //Debug.Log(currentRotation);

            //difine the rotation direction
            //Quaternion currentRotation = jointList[i].transform.localRotation;
            switch (i)
            {
                case 0:
                    currentRotation.y = jointValues[i];
                    break;
                case 1:
                    currentRotation.x = jointValues[i];
                    break;
                case 2:
                    currentRotation.y = jointValues[i];
                    break;
                case 3:
                    currentRotation.y = jointValues[i];
                    break;
                case 4:
                    currentRotation.x = jointValues[i];
                    break;
                case 5:
                    currentRotation.x = jointValues[i];
                    break;
            }
            //jointList[i].transform.localEulerAngles = currentRotation;  //perform the rotation 
            //jointList[i].transform.localRotation = currentRotation;
        }
        */
        for (int i = 7; i < 9; i++) // only for gripper, others performed in ROS.
        {
            Vector3 currentRotation = jointList[i].transform.localEulerAngles;
            currentRotation.x = -sliderList[6].value;
            if (i == 7)
            {
                currentRotation.x = -currentRotation.x;
            }
            jointList[i].transform.localEulerAngles = currentRotation;  //perform the rotation 
        }
    }

    // the only important part of this script
    void OnGUI()
    {
        //sliderList = getSliderList();
        for (int i = 0; i < sliderList.Length; i++)
        {
            jointValues[i] = sliderList[i].value;
        }
        /*
        if (TextToggle.isOn)
        {
            float[] offsetvalues = offsetslidervalues(sliderlist);
            /*var temp = "";
            offsetjointvalues(offsetvalues).tolist().foreach(i => temp+=", "+i.tostring());
            debug.log("re-offset slider values for checking (order not reversed): " +  temp );

            textcontrol.text = string.format("({0:0.0}, {1:0.0}, {2:0.0}, {3:0.0}, {4:0.0}, {5:0.0})",
                offsetvalues[5], offsetvalues[4], offsetvalues[3],
                offsetvalues[2], offsetvalues[1], offsetvalues[0]);

        }
        else
        {
            TextControl.text = string.Format("({0:0.0}, {1:0.0}, {2:0.0}, {3:0.0}, {4:0.0}, {5:0.0})",
                jointValues[5], jointValues[4], jointValues[3],
                jointValues[2], jointValues[1], jointValues[0]);
        }
        */
        
        TextControl.text = string.Format("({0:0.0}, {1:0.0}, {2:0.0}, {3:0.0}, {4:0.0}, {5:0.0}, {6:0.0})",
                jointValues[5], jointValues[4], jointValues[3],
                jointValues[2], jointValues[1], jointValues[0], jointValues[6]);
        
    }




    /*
    // Get sliders & offset to joint values for robot
    public float[] offsetSliderValues(Slider[] sliderList_)
    {

        float[] axes = new float[7];

        float tempVal = 0.0f;
        for (int i = 0; i < 7; i++)
        {
            // Offsets - should be opposite of set
            switch (i)
            {
                case 0:
                    tempVal = -1f * (sliderList_[i].value + 45f);
                    break;
                case 1:
                    tempVal = sliderList_[i].value - 90f;
                    break;
                case 3:
                    tempVal = sliderList_[i].value - 90f;
                    break;
                case 4:
                    tempVal = -1f * sliderList_[i].value;
                    break;
                default:
                    tempVal = sliderList_[i].value;
                    break;
            }

            // Check if out of bounds and loop around
            if (tempVal > upperLimit_r[i])
                tempVal -= 360f;

            else if (tempVal < lowerLimit_r[i])
                tempVal += 360f;

            // Save modified slider value
            axes[i] = tempVal;
        }

        return axes;
    }



    // Get joint values & offset to slider space
    public float[] offsetJointValues(float[] axes)
    {

        float[] sliderVals = new float[7];

        float tempVal = 0.0f;
        for (int i = 0; i < 7; i++)
        {
            // Should be opposite of offsets
            switch (i)
            {
                case 0:
                    tempVal = (-1f * axes[i]) - 45f;
                    break;
                case 1:
                    tempVal = axes[i] + 90f;
                    break;
                case 3:
                    tempVal = axes[i] + 90f;
                    break;
                case 4:
                    tempVal = -1f * axes[i];
                    break;
                default:
                    tempVal = axes[i];
                    break;
            }

            // Check if out of bounds and loop around
            if (tempVal > upperLimit_s[i])
                tempVal -= 360f;

            else if (tempVal < lowerLimit_s[i])
                tempVal += 360f;

            // Save modified joint value
            sliderVals[i] = tempVal;
        }

        return sliderVals;
    }
    */


            // Create the list of GameObjects that represent each joint of the robot
public void initializeJoints(GameObject[] jointList_)
    {
        var RobotChildren = RobotBase.GetComponentsInChildren<Rigidbody>();
        for (int i = 0; i < RobotChildren.Length; i++)
        {
            if (RobotChildren[i].name == "joint1")
            {
                jointList_[0] = RobotChildren[i].gameObject;
            }
            else if (RobotChildren[i].name == "joint2")
            {
                jointList_[1] = RobotChildren[i].gameObject;
            }
            else if (RobotChildren[i].name == "joint3")
            {
                jointList_[2] = RobotChildren[i].gameObject;
            }
            else if (RobotChildren[i].name == "joint4")
            {
                jointList_[3] = RobotChildren[i].gameObject;
            }
            else if (RobotChildren[i].name == "joint5")
            {
                jointList_[4] = RobotChildren[i].gameObject;
            }
            else if (RobotChildren[i].name == "joint6_flange")
            {
                jointList_[5] = RobotChildren[i].gameObject;
            }
            else if (RobotChildren[i].name == "newgripper280")
            {
                jointList_[6] = RobotChildren[i].gameObject;
            }
            
        }
        // gripper
        jointList_[7] = GameObject.Find("leftgripper");
        jointList_[8] = GameObject.Find("rightGripper");


    }

    // Create the list of GameObjects that represent each slider in the canvas
    public void initializeSliders(Slider[] sliderList_)
    {
        var CanvasChildren = CanvasObj.GetComponentsInChildren<Slider>();

        for (int i = 0; i < CanvasChildren.Length; i++)
        {
            if (CanvasChildren[i].name == "Slider0")
            {
                sliderList_[0] = CanvasChildren[i];

                sliderList_[0].minValue = lowerLimit_r[0];
                sliderList_[0].maxValue = upperLimit_r[0];
                sliderList_[0].value = 0;
            }
            else if (CanvasChildren[i].name == "Slider1")
            {
                sliderList_[1] = CanvasChildren[i];

                sliderList_[1].minValue = lowerLimit_r[1];
                sliderList_[1].maxValue = upperLimit_r[1];
                sliderList_[1].value = 0;
            }
            else if (CanvasChildren[i].name == "Slider2")
            {
                sliderList_[2] = CanvasChildren[i];

                sliderList_[2].minValue = lowerLimit_r[2];
                sliderList_[2].maxValue = upperLimit_r[2];
                sliderList_[2].value = 0;
            }
            else if (CanvasChildren[i].name == "Slider3")
            {
                sliderList_[3] = CanvasChildren[i];

                sliderList_[3].minValue = lowerLimit_r[3];
                sliderList_[3].maxValue = upperLimit_r[3];
                sliderList_[3].value = 0;
            }
            else if (CanvasChildren[i].name == "Slider4")
            {
                sliderList_[4] = CanvasChildren[i];

                sliderList_[4].minValue = lowerLimit_r[4];
                sliderList_[4].maxValue = upperLimit_r[4];
                sliderList_[4].value = 0;
            }
            else if (CanvasChildren[i].name == "Slider5")
            {
                sliderList_[5] = CanvasChildren[i];

                sliderList_[5].minValue = lowerLimit_r[5];
                sliderList_[5].maxValue = upperLimit_r[5];
                sliderList_[5].value = 0;
            }
            else if (CanvasChildren[i].name == "Slider6")
            {
                sliderList_[6] = CanvasChildren[i];

                sliderList_[6].minValue = lowerLimit_r[6];
                sliderList_[6].maxValue = upperLimit_r[6];
                sliderList_[6].value = 0;
            }
        }
    }
}
