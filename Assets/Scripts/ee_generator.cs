using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ee_generator : MonoBehaviour
{
    Mesh mesh;
    Vector3[] vertices;
    int[] EE;
    void Start()
    {
        mesh = new Mesh();
        GetComponent<MeshFilter>().mesh = mesh;
        CreateShape();
        UpdateMesh();
    }
    void CreateShape()
    {
        vertices = new Vector3[]
        {
            new Vector3 (0.05f,0.02f,0),
            new Vector3 (0.05f,-0.02f,0),
            new Vector3 (-0.05f,0.02f,0),
            new Vector3 (-0.05f,-0.02f,0),
            new Vector3 (0.05f,0.02f,0.1f),
            new Vector3 (0.05f,-0.02f,0.1f),
            new Vector3 (-0.05f,0.02f,0.1f),
            new Vector3 (-0.05f,-0.02f,0.1f),
            new Vector3 (0.04f,0.02f,0.1f),
            new Vector3 (0.04f,-0.02f,0.1f),
            new Vector3 (-0.04f,0.02f,0.1f),
            new Vector3 (-0.04f,-0.02f,0.1f),
            new Vector3 (0.04f,0.02f,0.02f),
            new Vector3 (0.04f,-0.02f,0.02f),
            new Vector3 (-0.04f,0.02f,0.02f),
            new Vector3 (-0.04f,-0.02f,0.02f),
        };
        EE = new int[]
        {
            1,0,3,
            0,2,3
        };
    }
    void UpdateMesh()
    {
        mesh.Clear();
        mesh.vertices = vertices;
        mesh.triangles = EE;
    }

}
