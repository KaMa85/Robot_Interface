using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Microsoft.MixedReality.Toolkit.UI;
using System;
using Microsoft.MixedReality.Toolkit;
using Microsoft.MixedReality.Toolkit.SpatialAwareness;
using TMPro;

public class Menu_New : MonoBehaviour
{
    private bool visibleMeshOn = true;

    public CrackManager crackManager;
    public string keyboardText;
    public KeyboardMode keyboardMode = KeyboardMode.None;
    public TouchScreenKeyboard keyboard;
    private string dataBaseIP = StaticVars.DataBaseIp;

    //Inputs
    public ButtonConfigHelper btnModeManipulation;
    public ButtonConfigHelper btnModeCrackPlacement;
    public ButtonConfigHelper btnToggleMesh;
    public ButtonConfigHelper btnCrackColor;
    public ButtonConfigHelper btnClearCracks;
    public ButtonConfigHelper btnVideoUpload;
    public ButtonConfigHelper btnIp;
    public PinchSlider recordSlider;

    //Otherstuff
    public PointerHandler pointerHandler;
    private IMixedRealitySpatialAwarenessMeshObserver observer = CoreServices.GetSpatialAwarenessSystemDataProvider<IMixedRealitySpatialAwarenessMeshObserver>();

    VideoUpload videoUpload;
    VideoUpload0 videoUpload0;
    VideoUpload1 videoUpload1;
    VideoUpload3 videoUpload3;
    VideoUpload4 videoUpload4;
    public Recivecoordinates Recivecoordinates;


    public enum KeyboardMode
    {
        None,
        ServerIP
    }

    void Start()
    {
        videoUpload = GetComponent<VideoUpload>(); 
        videoUpload0 = GetComponent<VideoUpload0>();
        videoUpload1 = GetComponent<VideoUpload1>();
        videoUpload3 = GetComponent<VideoUpload3>();
        videoUpload4 = GetComponent<VideoUpload4>();
        Recivecoordinates = GetComponent<Recivecoordinates>();

        btnClearCracks.OnClick.AddListener(ReciveCoordinates);
        btnModeManipulation.OnClick.AddListener(OnModeManipulation);
        btnModeCrackPlacement.OnClick.AddListener(OnModeCrackPlacement);
        btnToggleMesh.OnClick.AddListener(OnToggleMesh);
        btnCrackColor.OnClick.AddListener(OnCrackColorButton);
        btnVideoUpload.OnClick.AddListener(OnVideoUpload);
        btnIp.OnClick.AddListener(OnIPPressed);
        btnIp.GetComponent<ButtonConfigHelper>().MainLabelText = StaticVars.DataBaseIp;
        recordSlider.OnValueUpdated.AddListener(OnRecordSliderUpdate);
    }



    void Update()
    {
        if (Input.GetKeyDown(KeyCode.Space))
        {
            print("space key was pressed");
            OnIPPressed();
        }
        if (keyboard != null)
        {
            keyboardText = keyboard.text;
            switch (keyboardMode)
            {
                case KeyboardMode.ServerIP:
                    StaticVars.DataBaseIp = keyboardText;
                    btnIp.GetComponent<ButtonConfigHelper>().MainLabelText = StaticVars.DataBaseIp;
                    break;
                default:
                    break;
            }
        }
        else if (keyboard == null && keyboardMode == KeyboardMode.ServerIP)
        {
            keyboardMode = KeyboardMode.None;
        }
    }

    public void OnIPPressed()
    {

        keyboardMode = KeyboardMode.ServerIP;
        print(keyboardMode.ToString());
        OpenSystemKeyboard();
    }

    public void OpenSystemKeyboard()
    {
        keyboard = TouchScreenKeyboard.Open("", TouchScreenKeyboardType.Default, false, false, false, false);
        keyboard.text = StaticVars.DataBaseIp;
        print(keyboard.text);
        print("keyboard open");
    }

    private void ReciveCoordinates()
    {
        Debug.Log("CLicked");
        Recivecoordinates.RequestCoords(dataBaseIP +  "/CoordsToHololens.php");
    }
    private void OnVideoUpload()
    {
        videoUpload.OnStoppedRecordingVideo();
    }

    public void OnToggleMesh()
    {
        videoUpload1.OnStoppedRecordingVideo();
    }

    public void OnModeCrackPlacement()
    {
        videoUpload3.OnStoppedRecordingVideo();
    }


    public void OnModeManipulation()
    {
        videoUpload0.OnStoppedRecordingVideo();
    }


    private void ClearModeButtonsColor()
    {
        //print(btnModeCrackPlacement.transform.Find("IconAndText").Find("UIButtonSquareIcon").GetComponent<Renderer>().material.color);
        btnModeCrackPlacement.transform.Find("IconAndText").Find("UIButtonSquareIcon").GetComponent<Renderer>().material.color = Color.white;
        btnModeManipulation.transform.Find("IconAndText").Find("UIButtonSquareIcon").GetComponent<Renderer>().material.color = Color.white;
    }

    private void SetCrackManipulation(bool enabled)
    {
        crackManager.CrackManipultion(enabled);
    }

    public void OnCrackColorButton()
    {
        videoUpload4.OnStoppedRecordingVideo();
    }


    private void OnRecordSliderUpdate(SliderEventData sliderEvent)
    {
        videoUpload.seconds = sliderEvent.NewValue * 10;
        videoUpload1.seconds = sliderEvent.NewValue * 10;
        videoUpload0.seconds = sliderEvent.NewValue * 10;
        videoUpload3.seconds = sliderEvent.NewValue * 10;
        videoUpload4.seconds = sliderEvent.NewValue * 10;
    }

}