using UnityEngine;
using UnityEngine.UI;
using Microsoft;
using Microsoft.MixedReality.Toolkit.Utilities;
using Microsoft.MixedReality.Toolkit.Input;
using UnityEngine.EventSystems;
using Microsoft.MixedReality.Toolkit.UI;


public class HandGestureSlider : MonoBehaviour
{
    public Slider[] sliders;
    public Toggle toggle;
    private int i = 0;

    private Vector3 lastPosition;
    private Vector3 handMovement;
    GameObject thumbObject;
    MixedRealityPose pose;
    public GameObject sphereMarker;
    private bool sliderControlSwitch;
    private GameObject selectedObject;
    //private bool isPointerDown;

    //public ManipulationEventData eventData;
    //private CursorStateEnum cursorState;
    //public IMixedRealityPointer mixedRealityPointer;
    // Get the PointerStatus component


    private void Start()
    {
        Debug.Log("hand tracjing script started");
        //Debug.Log("cursor=" + cursor.IsPointerDown);
        //Debug.Log("genericPointer=" + genericPointer.IsFocusLocked);
        //Debug.Log("mixedRealityPointer=" + mixedRealityPointer.IsFocusLocked.ToString());
        lastPosition = Vector3.zero;
        handMovement = Vector3.zero;
        sliderControlSwitch = true;
        //isPointerDown = false;

        thumbObject = Instantiate(sphereMarker, this.transform);

        if (HandJointUtils.TryGetJointPose(TrackedHandJoint.ThumbTip, Handedness.Right, out pose))
        {
            //thumbObject.GetComponent<Renderer>().enabled = true;
            lastPosition = pose.Position;
        }


        //ObjectManipulator objectManipulator = FindObjectOfType<ObjectManipulator>();

        //Debug.Log("selescted=" + ObjectManipulator.isPointerDown.ToString());

        Debug.Log("selescted=" + toggle.isOn.ToString());
    }

    private void FixedUpdate()
    {
        //Debug.Log("PlayerMovement script fixupdate started");
        //Debug.Log("pointer=" + cursorState);
        //Debug.Log("mixedRealityPointer=" + mixedRealityPointer.IsFocusLocked.ToString());
        //Debug.Log("mixedRealityPointer=" + mixedRealityPointer.IsTargetPositionLockedOnFocusLock.ToString());
        if (ObjectManipulatedDetect.isPointerDown)
        {
            //Debug.Log("selescted=" + ObjectManipulator.isPointerDown.ToString());//without that, report wrong, need to debug in future

                                                                                 //Debug.Log("selescted-----test=" + ObjectManipulator.isOneHand.ToString());//without that, report wrong, need to debug in future
                                                                                 //Debug.Log("sliderControlSwitch=" + sliderControlSwitch.ToString());


            if (HandJointUtils.TryGetJointPose(TrackedHandJoint.ThumbTip, Handedness.Right, out pose))
            {
                thumbObject.transform.position = pose.Position;

                Vector3 currentPosition = thumbObject.transform.position;
                handMovement = currentPosition - lastPosition;
                lastPosition = currentPosition;

                //Debug.Log("value= " + handMovement.x.ToString());

                if (sliderControlSwitch && toggle.isOn)
                {
                    sliderControl();
                }
            }
        }
        
    }

    public void sliderControl()
    {
        //indexSelected();
        //Debug.Log("selected=" + isSelected);
        

        if (handMovement.x > 0.00001f)
        {
            sliders[i].value += 1f;
        }
        else if (handMovement.x < -0.00001f)
        {
            sliders[i].value -= 1f;
        }

        //Debug.Log("i= " + i+ "; slideri's value="+ sliders[i].value);
    }

    
    // open or close the slider control
    public void SliderSwitchOn()
    {
        sliderControlSwitch = true;
    }

    public void SliderSwitchOff()
    {
        sliderControlSwitch = false;
    }


    /*
    // open or close the slider control using toggle
    public void toggleSwitchOn()
    {
        toggle.value = true;
    }

    public void toggleSwitchOff()
    {
        toggle.value = false;
    }
    */

    // choose the slider
    public void SliderChoose0()
    {
        i = 0;
    }

    public void SliderChoose1()
    {
        i = 1;
    }

    public void SliderChoose2()
    {
        i = 2;
    }

    public void SliderChoose3()
    {
        i = 3;
    }

    public void SliderChoose4()
    {
        i = 4;
    }

    public void SliderChoose5()
    {
        i = 5;
    }

    public void SliderChoose6()
    {
        i = 6;
    }

    public void Output()
    {
        Debug.Log("outputdebug");
    }

    /*
    public void OnPointerDown(MixedRealityPointerEventData eventData)
    {
        Debug.Log("Object selected!");
        isSelected = true;
    }

    public void OnPointerDragged(MixedRealityPointerEventData eventData)
    {
        Debug.Log("Object dragged!");
    }

    public void OnPointerUp(MixedRealityPointerEventData eventData)
    {
        Debug.Log("No object selected.");
        isSelected = false;
    }

    public void OnPointerClicked(MixedRealityPointerEventData eventData)
    {
        // Handle pointer clicked event here
        Debug.Log("click.");
    }
    */
}



/*
public void indexSelected()
{
    // 获取选中的物体
    selectedObject = eventData.ManipulationSource;

    // 检查是否选中了物体
    if (selectedObject == null)
    {
        isSelected = false;
        Debug.Log("No object selected.");
    }
    else
    {
        isSelected = true;
        Debug.Log("Object selected!");
    }
}
*/




