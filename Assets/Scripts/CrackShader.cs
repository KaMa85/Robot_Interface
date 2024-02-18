using System.Collections;
using System.Collections.Generic;
using UnityEngine;





//[ExecuteInEditMode]
public class CrackShader: MonoBehaviour
{

    Material mat;
    public float minDistance = 1;
    public float maxDistance = 2;
    // Start is called before the first frame update
    void Start()
    {
        mat = GetComponent<Renderer>().material;
    }

    // Update is called once per frame
    void Update()
    {
        float range = maxDistance - minDistance;
        float distance = Vector3.Distance(this.transform.position, Camera.main.transform.position);
        float scaledDistance = (1 / range) * (distance - minDistance);
        float t = Mathf.Clamp(scaledDistance, 0, 1); // t= scaledDistance if 0<scaledDistance<1 otherwise t=0/1
        mat.SetFloat("Vector1_T", t);
    }
}
