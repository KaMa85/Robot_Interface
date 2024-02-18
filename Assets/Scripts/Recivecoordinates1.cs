using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using UnityEngine;
using UnityEngine.Networking;
using UnityEngine.Windows.WebCam;
using TMPro;
using System.Globalization;


public class Recivecoordinates1 : MonoBehaviour
{
    public GameObject spherePrefab;

    public TMP_Text displayCoords;
    string coords;
  
    public void RequestCoords(string ip)
    {
        StartCoroutine(Coords(ip));
    }
    public IEnumerator Coords(string ip)
    {
        Debug.Log(ip);
        UnityWebRequest webRequest = UnityWebRequest.Get(ip);
        yield return webRequest.SendWebRequest();
        if (webRequest.isNetworkError || webRequest.isHttpError)
        {
            Debug.Log(webRequest.error);
        }
        else
        {
            coords = webRequest.downloadHandler.text;
            Debug.Log(coords);

            // Remove the square brackets at the beginning and end of the string
            coords = coords.Trim('[', ']');

            // Split the string by commas to get individual coordinate strings
            string[] coordinateStrings = coords.Split(',');

            // Create a 2D array to hold the extracted coordinates
            Vector3[] extractedCoordinates = new Vector3[coordinateStrings.Length / 3];

            // Extract X, Y, and Z coordinates and store them in the array
            for (int i = 0; i < coordinateStrings.Length; i += 3)
            {
                // Trim any whitespace and additional characters from the coordinate strings
                string xString = coordinateStrings[i].Trim('[', ']');
                string yString = coordinateStrings[i + 1].Trim('[', ']');
                string zString = coordinateStrings[i + 2].Trim('[', ']');

                // Parse the trimmed coordinate strings to floats
                float x = FloatParse(xString);
                float y = FloatParse(yString);
                float z = FloatParse(zString);

                extractedCoordinates[i / 3] = new Vector3(x, y, z);
            }

            foreach (Vector3 coordinate in extractedCoordinates)
            {
                CreateSphere(coordinate);
            }
        }
    }

    private void CreateSphere(Vector3 position)
    {
        // Instantiate a sphere GameObject
        Debug.Log(position);
        GameObject sphere = Instantiate(spherePrefab, position, Quaternion.identity);
        sphere.transform.localScale = new Vector3(0.015f, 0.015f, 0.015f);
        sphere.GetComponent<Renderer>().material.color = Color.red;
    }

    private float FloatParse(string input)
    {
        float value;
        float.TryParse(input, NumberStyles.Float, CultureInfo.InvariantCulture, out value);
        return value;
    }
}