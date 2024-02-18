using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CrackManager : MonoBehaviour
{


    public int currentCrackIndex = 0;
    public GameObject[] crackPrefabs;

    public int currentCrackColorIndex = 0;
    public Color[] crackColors;

    public List<GameObject> cracks;


    public void AddCrack(Vector3 position, Quaternion rotation)
    {
        cracks.Add(Instantiate(crackPrefabs[currentCrackIndex], position, rotation));
        cracks[cracks.Count-1].GetComponent<Renderer>().material.SetColor("Color_", crackColors[currentCrackColorIndex]);
    }

    public void CrackManipultion(bool enabled)
    {
        foreach (GameObject crack in cracks)
            crack.GetComponent<Collider>().enabled = enabled;
    }

    public void ChangeAllCrackColorsTo(int crackColorIndex)
    {
        foreach (GameObject crack in cracks)
            crack.GetComponent<Renderer>().material.SetColor("Color_", crackColors[crackColorIndex]);
    }


    public void ClearCracks()
    {
        foreach (GameObject crack in cracks)
            Destroy(crack);
        cracks.Clear();
    }
}
