using UnityEngine;

public class DroneContactSphere : MonoBehaviour
{
    private Drone parentDrone;

    private float penaltyCooldown = 0.5f; // Apply penalty every 0.5 seconds
    private float lastPenaltyTime = 0f;

    private void Awake()
    {
        // Get the parent drone component
        parentDrone = GetComponentInParent<Drone>();
        if (parentDrone == null)
        {
            Debug.LogError("[DroneContactSphere] Parent Drone component not found!");
        }
    }

    private void OnTriggerStay(Collider other)
    {
        // Check if the other object is a drone and not the parent drone
        if (other.CompareTag("drone") && other.gameObject != parentDrone.gameObject)
        {
            if (Time.time - lastPenaltyTime >= penaltyCooldown)
            {
                lastPenaltyTime = Time.time;

                // Penalty for being too close to another drone
                float proximityPenalty = -0.05f;
                // PROXIMITY REWARD (penalty)
                parentDrone.AddReward(proximityPenalty);

                if (parentDrone.debugRewards)
                {
                    Debug.Log($"[DroneContactSphere] Proximity penalty applied for being near another drone: {proximityPenalty}");
                }
            }
        }
    }
}