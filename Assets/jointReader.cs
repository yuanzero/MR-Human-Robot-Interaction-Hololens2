using RosSharp.Urdf;
using UnityEngine;
using Joint = UnityEngine.Joint;
using UnityEngine.UI;

public class jointReader : MonoBehaviour
{

    private Slider jointSlider1;
    
    // Start is called before the first frame update
    void Start()
    {

        jointSlider1 = GameObject.Find("Slider1").GetComponent<Slider>();

        jointSlider1.onValueChanged.AddListener(jointSliderUpdate);
    }

    void jointSliderUpdate(float value)
        {
        Debug.Log(value);
        }

}
