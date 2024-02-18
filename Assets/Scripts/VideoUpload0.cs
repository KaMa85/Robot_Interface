using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using UnityEngine;
using UnityEngine.Networking;
using UnityEngine.Windows.WebCam;


public class VideoUpload0 : MonoBehaviour
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
        Vector3 l_r = GameObject.FindGameObjectWithTag("l_r").transform.position;
        Vector3 l_l = GameObject.FindGameObjectWithTag("l_l").transform.position;
        Vector3 u_l = GameObject.FindGameObjectWithTag("u_l").transform.position;
        Vector3 u_r = GameObject.FindGameObjectWithTag("u_r").transform.position;

        //Insert transform, then upload video
        StartCoroutine(InsertTransform("no_video", OriginPostion.x, OriginPostion.y, OriginPostion.z, 
            OriginRotation.eulerAngles.x, OriginRotation.eulerAngles.y, OriginRotation.eulerAngles.z, GoPostion.x, GoPostion.y,
            GoPostion.z, GoRotation.eulerAngles.x, GoRotation.eulerAngles.y, GoRotation.eulerAngles.z,
            0,
            m[0,0], m[0, 1], m[0, 2], m[0, 3], m[1, 0], m[1, 1], m[1, 2], m[1, 3], m[2, 0], m[2, 1], m[2, 2], m[2, 3], m[3, 0], m[3, 1],
            m[3, 2], m[3, 3], n[0, 0], n[0, 1],n[0, 2], n[0, 3], n[1, 0], n[1, 1], n[1, 2], n[1, 3], n[2, 0], n[2, 1], n[2, 2], 
            n[2, 3], n[3, 0], n[3, 1],n[3, 2], n[3, 3], 
            l_r.x, l_r.y, l_r.z, l_l.x, l_l.y, l_l.z, u_r.x, u_r.y, u_r.z, u_l.x, u_l.y, u_l.z));        
    }

    void OnStoppedVideoCaptureMode(VideoCapture.VideoCaptureResult result)
    {
        m_VideoCapture.Dispose();
        m_VideoCapture = null;
    }

    public IEnumerator InsertTransform(string name, float posx, float posy, float posz, float rotx, float roty, float rotz, 
        float posx1, float posy1, float posz1, float rotx1, float roty1, float rotz1, int gripper, float or11, float or12,
        float or13, float or14, float or21, float or22, float or23, float or24, float or31, float or32, float or33,
        float or34, float or41, float or42, float or43, float or44, float go11, float go12,
        float go13, float go14, float go21, float go22, float go23, float go24, float go31, float go32, float go33,
        float go34, float go41, float go42, float go43, float go44, 
        float lrx, float lry, float lrz, float llx, float lly, float llz,
        float urx, float ury, float urz, float ulx, float uly, float ulz)
    {
        //Upload the transform of main camera to data base, so the result can be placed in the correct position after processing is completed.
        //url for API call
        string uri = string.Format("{0}/{1}.php", DataBaseIp, nameof(InsertTransform));

        yield return Insert(
            uri,
            nameof(name), name,
            nameof(posx), posx,
            nameof(posy), posy,
            nameof(posz), posz,
            nameof(rotx), rotx,
            nameof(roty), roty,
            nameof(rotz), rotz,
            nameof(posx1), posx1,
            nameof(posy1), posy1,
            nameof(posz1), posz1,
            nameof(rotx1), rotx1,
            nameof(roty1), roty1,
            nameof(rotz1), rotz1,
            nameof(gripper), gripper,         
            nameof(or11), or11,
            nameof(or12), or12,
            nameof(or13), or13,
            nameof(or14), or14,
            nameof(or21), or21,
            nameof(or22), or22,
            nameof(or23), or23,
            nameof(or24), or24,
            nameof(or31), or31,
            nameof(or32), or32,
            nameof(or33), or33,
            nameof(or34), or34,
            nameof(or41), or41,
            nameof(or42), or42,
            nameof(or43), or43,
            nameof(or44), or44,
            nameof(go11), go11,
            nameof(go12), go12,
            nameof(go13), go13,
            nameof(go14), go14,
            nameof(go21), go21,
            nameof(go22), go22,
            nameof(go23), go23,
            nameof(go24), go24,
            nameof(go31), go31,
            nameof(go32), go32,
            nameof(go33), go33,
            nameof(go34), go34,
            nameof(go41), go41,
            nameof(go42), go42,
            nameof(go43), go43,
            nameof(go44), go44,
            nameof(lrx), lrx,
            nameof(lry), lry,
            nameof(lrz), lrz,
            nameof(llx), llx,
            nameof(lly), lly,
            nameof(llz), llz,
            nameof(urx), urx,
            nameof(ury), ury,
            nameof(urz), urz,
            nameof(ulx), ulx,
            nameof(uly), uly,
            nameof(ulz), ulz
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
