using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using UnityEngine;
using UnityEngine.Networking;
using UnityEngine.Windows.WebCam;


public class VideoUpload4 : MonoBehaviour
{

    string videoName = "";
    string videoPath = "";
    private string DataBaseIp = StaticVars.DataBaseIp;
    string uploadUrl;
    string uploadPath = "/save_video.php";
    VideoCapture m_VideoCapture = null;
    public float seconds = 4;
    void Start()
    {
        uploadUrl = "http://" + DataBaseIp + uploadPath;
        //StartRecord();
    }



    public void OnStoppedRecordingVideo()
    {
        Debug.Log("Stopped Recording Video!");
        //Insert cam position
        Vector3 OriginPostion = GameObject.FindGameObjectWithTag("Origin_Orb").transform.position;
        Vector3 GoPostion = GameObject.FindGameObjectWithTag("Go_Orb").transform.position;
        Quaternion OriginRotation = GameObject.FindGameObjectWithTag("Origin_Orb").transform.rotation;
        Quaternion GoRotation = GameObject.FindGameObjectWithTag("Go_Orb").transform.rotation;
        Matrix4x4 m = Matrix4x4.Rotate(OriginRotation);
        Matrix4x4 n = Matrix4x4.Rotate(GoRotation);

        //Insert transform, then upload video
        StartCoroutine(InsertDecision("n"));
        //m_VideoCapture.StopVideoModeAsync(OnStoppedVideoCaptureMode);

    }

    void OnStoppedVideoCaptureMode(VideoCapture.VideoCaptureResult result)
    {
        m_VideoCapture.Dispose();
        m_VideoCapture = null;
    }

    public IEnumerator InsertDecision(string name)
    {
        //Upload the transform of main camera to data base, so the result can be placed in the correct position after processing is completed.
        //url for API call
        string uri = string.Format("{0}/{1}.php", DataBaseIp, nameof(InsertDecision));

        yield return Insert(
            uri,
             nameof(name), name
            );


    }



    /// <summary>
    /// 
    /// </summary>
    /// <param name="uri"></param>
    /// <param name="args">arg1Name, arg1Value</param>
    /// <returns></returns>
    public IEnumerator Insert(string uri, params object[] args)
    {
        //Create form
        WWWForm form = new WWWForm();

        //Add args to form
        for (int i = 0; i < args.Length - 1; i += 2)
        {
            string name = args[i].ToString();
            //If arg value is unassigned, use NULL
            string value = args[i + 1] == null ? "NULL" : args[i + 1].ToString();

            form.AddField(name, value);

            //print(string.Format("{0} form.Addfield({1}, {2})", uri, name, value));
        }

        //Send request
        yield return InsertRequest(uri, form);
    }

    public IEnumerator InsertRequest(string uri, WWWForm form)
    {
        using (UnityWebRequest www = UnityWebRequest.Post(uri, form))
        {

            yield return www.SendWebRequest();

            if (www.isNetworkError || www.isHttpError)
                print(www.error);
            else
                print(www.downloadHandler.text);
        }

        //Finished uploading transform data, now upload video
    }
}
