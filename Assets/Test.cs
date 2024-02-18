using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Networking;

public class Test : MonoBehaviour
{
    string videoName = "Test2";
    string videoPath = "";
    private string DataBaseIp = StaticVars.DataBaseIp;
    string uploadUrl;
    string uploadPath = "/save_video.php";

    void Start()
    {
        uploadUrl = "http://" + DataBaseIp + uploadPath;


        //Insert cam position
        Vector3 camPostion = Camera.main.transform.position;
        Quaternion camRotation = Camera.main.transform.rotation;
        StartCoroutine(InsertTransform(videoName, camPostion.x, camPostion.y, camPostion.z, camRotation.x, camRotation.y, camRotation.z, camRotation.w));
    }




public IEnumerator InsertTransform(string name, float posx, float posy, float posz, float rotx, float roty, float rotz, float rotw)
{
    //Upload the transform of main camera to data base, so the result can be placed in the correct postiion after processing is completed.
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
        nameof(rotw), rotw
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
}

}
