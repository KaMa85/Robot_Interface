using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.Networking;
using Microsoft.MixedReality.Toolkit.UI;
using UnityEngine.Events;

public class selectnewestprocessed : MonoBehaviour
{
    public Texture2D downloadedTexture;
    private Material mat;
    public Material defaultMaterial;
    private string DataBaseIp = StaticVars.DataBaseIp;

    public GameObject buttonPrefab;
    public List<GameObject> buttons;
    public List<int> buttonIDs;
    public ButtonConfigHelper btnShowThresholds;
    public bool ShowThresholdButtons = true;
    public Gradient colorGradient;

    public GameObject markerPrefab;
    private List<GameObject> markers = new List<GameObject>();
    public Vector2 videoSize = new Vector2(2272, 1278);

    public GameObject resultQuad;
    public GameObject parentObject;

    public float widthMulti = 1;
    public float heightMulti = 1;

    public Vector3 headPos = new Vector3(-0.015f, -0.026f, 0);
    PostProcessedData postprocesseddata;

    public int activeInterestPointsID = 1;

    // Start is called before the first frame update
    void Start()
    {
        //mat = GetComponent<Renderer>().material;
        StartCoroutine(DataBaseObserver());
        btnShowThresholds.OnClick.AddListener(OnToggleThresholds);
    }

    private void OnToggleThresholds()
    {
        ShowThresholdButtons = !ShowThresholdButtons;
        foreach (GameObject button in buttons)
        {
            button.SetActive(ShowThresholdButtons);
        }
    }

    // Update is called once per frame
    void Update()
    {

    }
    private IEnumerator DataBaseObserver()
    {
        print("db Observer started");
        string uri = DataBaseIp + "/selectnewestprocessed.php";
        string commandUri = DataBaseIp + "/getCommand.php";
        int prevID = 0;
        int prevCommandID = 0;
        bool useAsFirstID = true;
        bool updateButtonNames = false;
        print(uri);
        while (true)
        {
            yield return GetRequest(uri, (string str) =>
            {
                if (str != "Failed to connect")
                {
                    //print(str);
                    postprocesseddata = JsonUtility.FromJson<PostProcessedData>(str);
                    if (postprocesseddata.id != prevID && postprocesseddata.status == "true")
                    {
                        prevID = postprocesseddata.id;
                        updateButtonNames = true;
                        //Clear buttons
                        foreach (GameObject button in buttons)
                        {
                            Destroy(button);
                        }
                        buttons.Clear();
                        buttonIDs.Clear();
                        buttons = new List<GameObject>();
                        buttonIDs = new List<int>();

                        //make buttons
                        InterestPointsIDList interestPointsIDList = new InterestPointsIDList();
                        interestPointsIDList.interestPoints = postprocesseddata.json_points;
                        interestPointsIDList.Points = interestPointsIDList.getPoints();
                        float thresholdValue = 0.0f;
                        int columnLength = 3;
                        for (int i = 0; i < interestPointsIDList.Points.Count; i++)
                        {
                            int id = interestPointsIDList.Points[i];
                            buttonIDs.Add(id);
                            buttons.Add(Instantiate<GameObject>(buttonPrefab, this.transform));
                            int column = (int)Mathf.Floor(i / columnLength);
                            int row = i - (column * columnLength);
                            buttons[i].transform.localPosition = new Vector3(0.1f + (0.03f * column), -0.03f * row, 0 );
                            buttons[i].name = "button_" + id;
                            //buttons[i].GetComponent<ButtonConfigHelper>().MainLabelText = String.Format("Threshold {0:F2}", thresholdValue); //TODO: replace name with actual threshold
                            thresholdValue += 0.1f;
                            buttons[i].GetComponent<ThresholdButton>().thresholdID = id;
                            buttons[i].GetComponent<ThresholdButton>().newest = this;
                            buttons[i].GetComponent<ButtonConfigHelper>().OnClick.AddListener(buttons[i].GetComponent<ThresholdButton>().OnPress);
                            buttons[i].SetActive(ShowThresholdButtons);
                            buttons[i].transform.Find("IconAndText").Find("UIButtonSquareIcon")
                                .GetComponent<Renderer>().material.color = colorGradient.Evaluate((float)i / interestPointsIDList.Points.Count);
                            print("button made!");

                            activeInterestPointsID = id;
                        }
                        //TempFunction();
                        //StartCoroutine(DownloadImage(postprocesseddata.image_location, postprocesseddata.transformName));
                    }
                }
                else
                {
                    Debug.LogError("Failed to connect to: " + uri);
                }
            });

            if (updateButtonNames)
            {
                yield return GetRequest(DataBaseIp + "/selectThresholdNamesFromID.php?id=" + prevID, (string str1) =>
                {
                    string[] thresholdNames = str1.Split('\t');

                    for (int i = 0; i < buttons.Count; i++)
                    {
                        buttons[i].GetComponent<ButtonConfigHelper>().MainLabelText = "Threshold " + thresholdNames[i];
                    }
                    
                });
                updateButtonNames = false;
            }

            //Check for database commands
            yield return GetRequest(commandUri, (string str) =>
            {
                if (str != "Failed to connect")
                {
                    //print(str);
                    CommandData commandData = JsonUtility.FromJson<CommandData>(str);
                    if (commandData.id != prevCommandID && commandData.status == "true")
                    {
                        prevCommandID = commandData.id;
                        if (useAsFirstID) { useAsFirstID = false; }
                        else
                        {
                            String[] commandSplit = commandData.command.Split(' ');
                            switch (commandSplit[0])
                            {
                                //Record
                                case "R":
                                    int duration = int.Parse(commandSplit[1]);
                                    //GameObject.Find("Menu_v2(Clone)").GetComponent<VideoUpload>().StartRecordTime(duration);
                                    break;
                                //Move head
                                case "H":
                                    float x = float.Parse(commandSplit[1]);
                                    float y = float.Parse(commandSplit[2]);
                                    float z = float.Parse(commandSplit[3]);
                                    headPos = new Vector3(x, y, z);
                                    StartCoroutine(DownloadImage(postprocesseddata.image_location, postprocesseddata.transformName, activeInterestPointsID));
                                    break;
                                //Resize points
                                case "P":
                                    break;
                                default:
                                    break;
                            }
                        }
                    }
                }
                else
                {
                    Debug.LogError("Failed to connect to: " + commandUri);
                }
            });

            yield return new WaitForSeconds(0.2f);
        }
    }

    public void ButtonPressed(int id)
    {
        print("Starting coroutine!");
        StartCoroutine(DownloadImage(postprocesseddata.image_location, postprocesseddata.transformName, id));
    }

    private IEnumerator DownloadImage(string imageName, string transformName, int interestPointsID)
    {


        //Get texture
        /*
        string url = DataBaseIp + "/postprocessed_images/" + imageName;
        print(url);
        UnityWebRequest request = UnityWebRequestTexture.GetTexture(url);
        yield return request.SendWebRequest();
        if (request.isNetworkError || request.isHttpError)
            Debug.Log(request.error);
        else
        {
            downloadedTexture = ((DownloadHandlerTexture)request.downloadHandler).texture;
            mat.mainTexture = downloadedTexture;
        }
        */


        //Get transform
        UnityWebRequest webRequest = UnityWebRequest.Get(DataBaseIp + String.Format("/selecttransform.php?name={0}", transformName));
        yield return webRequest.SendWebRequest();
        if (webRequest.isNetworkError || webRequest.isHttpError)
            Debug.Log(webRequest.error);
        else
        {
            print(webRequest.downloadHandler.text);
            TransformData transformData = JsonUtility.FromJson<TransformData>(webRequest.downloadHandler.text);

            if (transformData.status == "false")
            {
                //If no position data is in the database, project from the current hololens view.
                transformData.posx = Camera.main.transform.position.x;
                transformData.posy = Camera.main.transform.position.y;
                transformData.posz = Camera.main.transform.position.z;
                
                transformData.rotx = Camera.main.transform.rotation.x;
                transformData.roty = Camera.main.transform.rotation.y;
                transformData.rotz = Camera.main.transform.rotation.z;
                transformData.rotw = Camera.main.transform.rotation.w;

                transformData.status = "true";
            }

            if (transformData.status == "true")
            {
                Destroy(resultQuad);
                //Make object to attach result to
                resultQuad = GameObject.CreatePrimitive(PrimitiveType.Quad);
                resultQuad.transform.rotation = transformData.GetRot();
                resultQuad.transform.position = transformData.GetPos() + resultQuad.transform.forward * 0.2f; //TODO: push the image forward to match FoV
                resultQuad.transform.localScale = new Vector3(0.3408f * widthMulti, 0.1917f * heightMulti, 1);
                //resultQuad.GetComponent<Renderer>().destroy//material = defaultMaterial;
                Destroy(resultQuad.GetComponent<Renderer>());
                //resultQuad.GetComponent<Renderer>().material.mainTexture = downloadedTexture;


                //Get the interest points and project them from the transform quad
                print("Attaching to head");
                StartCoroutine(AttachPointsToHead(transformData, interestPointsID));
            }
        }


    }



    public IEnumerator AttachPointsToHead(TransformData transformData, int pointsID)
    {
        UnityWebRequest pointRequest = UnityWebRequest.Get(DataBaseIp + String.Format("/selectPointsFromID.php?id={0}", pointsID));
        yield return pointRequest.SendWebRequest();
        if (pointRequest.isNetworkError || pointRequest.isHttpError)
            Debug.Log(pointRequest.error);
        else
        {
            Destroy(parentObject);
            PointData pointData = JsonUtility.FromJson<PointData>(pointRequest.downloadHandler.text);
            List<Vector2> interestPoints = pointData.GetPoints();
            Debug.Log(interestPoints);
            //CastInterestPoints(interestPoints, resultQuad.transform, transformData.GetPos());

            //Create marker casting quad
            parentObject = new GameObject();
            parentObject.transform.rotation = transformData.GetRot();
            parentObject.transform.position = transformData.GetPos();
            resultQuad.transform.parent = parentObject.transform;
            resultQuad.AddComponent(typeof(ProjectPoints));
            resultQuad.GetComponent<ProjectPoints>().interestPoints = interestPoints;
            resultQuad.GetComponent<ProjectPoints>().markerPrefab = markerPrefab;
            //resultQuad.GetComponent<Renderer>().material.mainTextureScale = new Vector2(0, 0);
            resultQuad.GetComponent<ProjectPoints>().CreateHead(headPos);
            //resultQuad.SetActive(false);
            print("projecting points for db id " + pointsID);
            resultQuad.GetComponent<ProjectPoints>().MakePoints();

        }
    }




    private void CastInterestPoints(List<Vector2> interestPoints, Transform imageTransform, Vector3 cameraPosition)
    {
        //Remove old markers
        foreach (GameObject marker in markers)
        {
            GameObject.Destroy(marker);
        }
        markers.Clear();

        //Spawn new markers
        foreach (Vector2 interestPoint in interestPoints)
        {
            //Top left is 0,0, bottom right is 1,1;

            //TODO: simplify this
            float horizontalNormal = interestPoint.x / videoSize.x;
            float localX = (horizontalNormal * imageTransform.lossyScale.x) - (imageTransform.lossyScale.x / 2);
            float localY = (Mathf.Abs(interestPoint.y - videoSize.y) / videoSize.y * imageTransform.lossyScale.y) + (imageTransform.lossyScale.y / 2) - imageTransform.lossyScale.y;

            Vector3 interestPointGlobalPosition = imageTransform.position + (imageTransform.rotation * new Vector3(localX, localY, 0));

            // float verticalAngle = (verticalNormal - 0.5f) * FoV.y;
            // float horizontalAngle = (horizontalNormal - 0.5f) * FoV.x;
            // Quaternion rayQuat = transform.rotation * Quaternion.Euler(new Vector3(verticalAngle, horizontalAngle, 0));

            Ray ray = new Ray(interestPointGlobalPosition, interestPointGlobalPosition - cameraPosition);


            RaycastHit hit;
            if (Physics.Raycast(ray, out hit, 3, LayerMask.GetMask("Spatial Awareness")))
            {
                markers.Add(Instantiate(markerPrefab, hit.point, Quaternion.identity));
                Debug.DrawLine(interestPointGlobalPosition, hit.point, Color.green, 20f);
            }
        }
    }


    private IEnumerator GetRequest(string uri, System.Action<string> callback)
    {
        using (UnityWebRequest webRequest = UnityWebRequest.Get(uri))
        {
            // Request and wait for the desired page.
            yield return webRequest.SendWebRequest();

            //byte[] results = www.downloadHandler.data;
            string[] pages = uri.Split('/');
            int page = pages.Length - 1;

            if (webRequest.isNetworkError)
            {
                Debug.Log(pages[page] + ": Error: " + webRequest.error);
                callback("Failed to connect");
            }
            else
            {
                string data = webRequest.downloadHandler.text;
                callback(data);
            }
        }
    }
    [Serializable]
    public class PostProcessedData
    {
        public string status;
        public int id;
        public string transformName;
        public int prevideo_id;
        public string json_points;
        public string image_location;
        public string date;
    }

    public class CommandData
    {
        public string status;
        public int id;
        public string command;
    }

    public class TransformData
    {
        public string status;
        public int id;
        public string name;
        public float posx;
        public float posy;
        public float posz;
        public float rotx;
        public float roty;
        public float rotz;
        public float rotw;

        internal Vector3 GetPos()
        {
            return new Vector3(posx, posy, posz);
        }

        internal Quaternion GetRot()
        {
            return new Quaternion(rotx, roty, rotz, rotw);
        }
    }



    public class PointData
    {
        public string status;
        public string interestPoints;
        public List<Vector2> Points;

        internal List<Vector2> GetPoints()
        {
            Points = new List<Vector2>();

            float[][] pointArray = interestPoints.Split(new string[] { "[[", "],[", "]]" }, StringSplitOptions.RemoveEmptyEntries)
                .Select(x => Array.ConvertAll<string, float>(x.Split(','), float.Parse))
                .ToArray();

            foreach (float[] pair in pointArray)
            {
                Points.Add(new Vector2(pair[0], pair[1]));
            }

            return Points;

        }
    }

    public class InterestPointsIDList
    {
        public string status;
        public string interestPoints; // [1,2,3]
        public List<int> Points;

        internal List<int> getPoints()
        {
            Points = new List<int>();

            string[] nums = interestPoints.Split(new string[] { "[", ",", "]" }, StringSplitOptions.RemoveEmptyEntries);

            foreach(string num in nums)
            {
                try{ Points.Add(System.Convert.ToInt32(num)); }
                catch { print("nan"); }
            }

            return Points;
        }
    }





}
