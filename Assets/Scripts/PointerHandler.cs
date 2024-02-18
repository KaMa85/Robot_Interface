using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Microsoft.MixedReality.Toolkit.Input;
using System;

public class PointerHandler: MonoBehaviour, IMixedRealityPointerHandler
{

    public InputMode inputMode = InputMode.placeCrack;
    public CrackManager crackManager;

    [Serializable]
    public enum InputMode
    {
        manipulation,
        placeCrack
    }



    #region MonoBehavior
    // Start is called before the first frame update
    void Start() { }

    // Update is called once per frame
    void Update() { }
    #endregion

    #region IMixedRealityPointerHandler
    public void OnPointerClicked(MixedRealityPointerEventData eventData)
    {
        print("OnPointerDown");
        IMixedRealityCursor cursor = eventData.Pointer.BaseCursor;
       
        //print(eventData.selectedObject.name);
        switch (inputMode)
        {
            case InputMode.manipulation:
                //TODO
                break;
            case InputMode.placeCrack:
                crackManager.AddCrack(cursor.Position, cursor.Rotation);
                break;
            default:
                break;
        }

    }

    //Unused, but required for IMixedRealityPointerHandler
    public void OnPointerDown(MixedRealityPointerEventData eventData) { }

    //Unused, but required for IMixedRealityPointerHandler
    public void OnPointerDragged(MixedRealityPointerEventData eventData) { }

    //Unused, but required for IMixedRealityPointerHandler
    public void OnPointerUp(MixedRealityPointerEventData eventData) { }
    #endregion
}
