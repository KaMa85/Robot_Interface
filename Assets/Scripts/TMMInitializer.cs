using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TMMInitializer : MonoBehaviour
{
    //TODO: Save manager fix
    //TODO: Manipulation

    public GameObject MeshParentPrefab;
    public GameObject MenuPrefab;
    private GameObject menuObject;

    // Start is called before the first frame update
    void Start()
    {
        menuObject = Instantiate(MenuPrefab);
        menuObject.GetComponent<Menu_New>().crackManager = GetComponent<CrackManager>();
        menuObject.GetComponent<Menu_New>().pointerHandler = GetComponent<PointerHandler>();
        
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
