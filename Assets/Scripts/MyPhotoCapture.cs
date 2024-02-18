using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.Windows.WebCam;

public class MyPhotoCapture : MonoBehaviour
{

    private void Start()
    {
        PhotoCapture.CreateAsync(false, OnPhotoCaptureCreated);
    }
    private PhotoCapture photoCaptureObject = null;

    void OnPhotoCaptureCreated(PhotoCapture captureObject)
    {
        photoCaptureObject = captureObject;

        Resolution cameraResolution = PhotoCapture.SupportedResolutions.OrderByDescending((res) => res.width * res.height).First();

        CameraParameters c = new CameraParameters();
        c.hologramOpacity = 0.0f;
        c.cameraResolutionWidth = cameraResolution.width;
        c.cameraResolutionHeight = cameraResolution.height;
        c.pixelFormat = CapturePixelFormat.BGRA32;

        captureObject.StartPhotoModeAsync(c, OnPhotoModeStarted);
    }
    void OnStoppedPhotoMode(PhotoCapture.PhotoCaptureResult result)
    {
        print("Photomode Finished");
        photoCaptureObject.Dispose();
        photoCaptureObject = null;
    }
    //private void OnPhotoModeStarted(PhotoCapture.PhotoCaptureResult result)
    //{
    //    if (result.success)
    //    {
    //        string filename = string.Format(@"CapturedImage{0}_n.jpg", Time.time);
    //        string filePath = System.IO.Path.Combine(Application.persistentDataPath, filename);

    //        photoCaptureObject.TakePhotoAsync(filePath, PhotoCaptureFileOutputFormat.JPG, OnCapturedPhotoToDisk);
    //    }
    //    else
    //    {
    //        Debug.LogError("Unable to start photo mode!");
    //    }
    //}
    void OnCapturedPhotoToDisk(PhotoCapture.PhotoCaptureResult result)
    {
        if (result.success)
        {
            Debug.Log("Saved Photo to disk!");
            photoCaptureObject.StopPhotoModeAsync(OnStoppedPhotoMode);
        }
        else
        {
            Debug.Log("Failed to save Photo to disk");
        }
    }
    private void OnPhotoModeStarted(PhotoCapture.PhotoCaptureResult result)
    {
        if (result.success)
        {
            photoCaptureObject.TakePhotoAsync(OnCapturedPhotoToMemory);
        }
        else
        {
            Debug.LogError("Unable to start photo mode!");
        }
    }
    void OnCapturedPhotoToMemory(PhotoCapture.PhotoCaptureResult result, PhotoCaptureFrame photoCaptureFrame)
    {
        if (result.success)
        {
            // Create our Texture2D for use and set the correct resolution
            Resolution cameraResolution = PhotoCapture.SupportedResolutions.OrderByDescending((res) => res.width * res.height).First();
            Texture2D targetTexture = new Texture2D(cameraResolution.width, cameraResolution.height);
            // Copy the raw image data into our target texture
            photoCaptureFrame.UploadImageDataToTexture(targetTexture);
            // Do as we wish with the texture such as apply it to a material, etc.
            if (photoCaptureFrame.hasLocationData)
            {
                photoCaptureFrame.TryGetCameraToWorldMatrix(out Matrix4x4 cameraToWorldMatrix);

                Vector3 position = cameraToWorldMatrix.GetColumn(3) - cameraToWorldMatrix.GetColumn(2);
                Quaternion rotation = Quaternion.LookRotation(-cameraToWorldMatrix.GetColumn(2), cameraToWorldMatrix.GetColumn(1));

                photoCaptureFrame.TryGetProjectionMatrix(Camera.main.nearClipPlane, Camera.main.farClipPlane, out Matrix4x4 projectionMatrix);

                //Instantiate quad with photo
                GameObject newObj = GameObject.CreatePrimitive(PrimitiveType.Cube);
                newObj.transform.position = Camera.main.transform.position;
                newObj.transform.localScale = Vector3.one * 0.1f;
                //newObj.transform.rotation = Camera.main.transform.rotation;
                //newObj.transform.localScale = new Vector3(cameraResolution.width, cameraResolution.height)/4000;
                //newObj.GetComponent<Renderer>().material.mainTexture = targetTexture;
                
            }
            else
            {
                Debug.LogError("no location data on photo");
            }
        }
        // Clean up
        photoCaptureObject.StopPhotoModeAsync(OnStoppedPhotoMode);
    }
    
    //void OnCapturedPhotoToMemory(PhotoCapture.PhotoCaptureResult result, PhotoCaptureFrame photoCaptureFrame)
    //{
    //    if (result.success)
    //    {
    //        List<byte> imageBufferList = new List<byte>();
    //        // Copy the raw IMFMediaBuffer data into our empty byte list.
    //        photoCaptureFrame.CopyRawImageDataIntoBuffer(imageBufferList);

    //        // In this example, we captured the image using the BGRA32 format.
    //        // So our stride will be 4 since we have a byte for each rgba channel.
    //        // The raw image data will also be flipped so we access our pixel data
    //        // in the reverse order.
    //        int stride = 4;
    //        float denominator = 1.0f / 255.0f;
    //        List<Color> colorArray = new List<Color>();
    //        for (int i = imageBufferList.Count - 1; i >= 0; i -= stride)
    //        {
    //            float a = (int)(imageBufferList[i - 0]) * denominator;
    //            float r = (int)(imageBufferList[i - 1]) * denominator;
    //            float g = (int)(imageBufferList[i - 2]) * denominator;
    //            float b = (int)(imageBufferList[i - 3]) * denominator;

    //            colorArray.Add(new Color(r, g, b, a));
    //        }
    //        // Now we could do something with the array such as texture.SetPixels() or run image processing on the list
    //    }
    //    photoCaptureObject.StopPhotoModeAsync(OnStoppedPhotoMode);
    //}

}
