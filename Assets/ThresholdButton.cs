using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ThresholdButton : MonoBehaviour
{
    public selectnewestprocessed newest;
    public int thresholdID = 0;

    public void OnPress()
    {
        print("Pressed! " + thresholdID);
        newest.ButtonPressed(thresholdID);
    }
}
