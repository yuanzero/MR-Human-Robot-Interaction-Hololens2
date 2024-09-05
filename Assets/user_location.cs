using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Microsoft.MixedReality.Toolkit;
using Microsoft.MixedReality.Toolkit.Utilities;

public class user_location : MonoBehaviour
{
    public GameObject seudo_human_model;
    GameObject human_model;
    // Start is called before the first frame update
    void Start()
    {
        human_model = Instantiate(seudo_human_model);
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        Vector3 userPosition = CameraCache.Main.transform.position;
        human_model.transform.position = userPosition;
        Debug.Log("human_model" + human_model.transform.position.ToString("F2"));
    }
}
