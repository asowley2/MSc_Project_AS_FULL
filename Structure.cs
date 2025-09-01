using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Structure : MonoBehaviour
{
    public bool touchingStructure;
    public Drone drone;

    void OnTriggerEnter(Collider collision)
    {
        // Structure hit by drone
        if (collision.gameObject.CompareTag("drone"))
        {
            drone = collision.gameObject.GetComponent<Drone>();
            touchingStructure = true;
            NotifyAgent();
        }
    }

    private void NotifyAgent()
    {
        drone.HitStructure();
    }
}
