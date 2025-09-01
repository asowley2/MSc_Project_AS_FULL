using System.Collections;
using System.Collections.Generic;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;
using UnityEngine.Serialization;

public class Drone : Agent
{
    [SerializeField, Tooltip("Units per second for local-space movement")]
    [FormerlySerializedAs("moveSpeed")]
    private float movementSpeed = 3f;

    [Header("Spawn")]
    [SerializeField] private Vector3 spawnCenter = new Vector3(0f, 5f, 0f);
    [SerializeField, Tooltip("Minimum distance between drones at spawn")]
    private float minSeparation = 40f; 
    [SerializeField] private int maxSpawnAttempts = 100;
    [SerializeField] private bool randomizeYaw = true;

    [Header("ML-Agents Observations")]
    [SerializeField] private bool includeSensorObservations = true;
    [SerializeField] private bool includePositionObservations = true;
    [SerializeField] private bool includeVelocityObservations = true;
    [SerializeField] private bool includeExplorationProgress = true;

    [Header("Rewards")]
    [SerializeField] private float explorationReward = 2f;
    [SerializeField] private float collisionPenalty = -3.0f;
    [SerializeField] private bool debugLifecycle = true;
    [SerializeField] public bool debugRewards = false;

    private static readonly List<Drone> ActiveDrones = new List<Drone>();
    
    // Drone components
    private DroneSensor droneSensor;
    private Rigidbody droneRigidbody;
    private WorldOccupancyGrid worldGrid;
    
    // Tracking vars
    private float lastExplorationPercentage;
    private int stepCount;

    // Grace period vars
    private bool inGracePeriod = false;
    private float gracePeriodDuration = 2f; // 2 second grace period on spawn

    // Each drone will be programmed based off this class, however they will perform their own actions independently
    public void HitStructure()
    {
        if (inGracePeriod) 
        {
            if (debugLifecycle) Debug.Log("[Drone] Collision ignored during grace period.");
            return; 
        }

        Debug.Log($"[Drone] Structure collision! Penalty: {collisionPenalty}");
        
        AddReward(collisionPenalty);
        EndEpisode();
    }

    private void Awake() 
    { 
        if (debugLifecycle) Debug.Log("[Drone] Awake");
        
        // Get drones sensor and rigidbody
        droneSensor = GetComponent<DroneSensor>();
        droneRigidbody = GetComponent<Rigidbody>();
        
        if (droneSensor == null)
        {
            Debug.LogError("[Drone] DroneSensor component not found!");
        }
    }

    private void Start() 
    { 
        if (debugLifecycle) Debug.Log("[Drone] Start");

        // Ensure structures are only reset once
        if (!structuresInitialised)
        {
            StructureSpawner structureSpawner = FindFirstObjectByType<StructureSpawner>();
            if (structureSpawner != null)
            {
                structureSpawner.SpawnStructures();
                structuresInitialised = true; // Mark structures as init
                Debug.Log("[Drone] Structures initialised.");
            }
            else
            {
                Debug.LogWarning("[Drone] No StructureSpawner found in the scene!");
            }
        }

        // Find world grid
        worldGrid = FindFirstObjectByType<WorldOccupancyGrid>();
        if (worldGrid == null)
        {
            Debug.LogWarning("[Drone] No WorldOccupancyGrid found in scene!");
        }
    }

    private void Update()
    {
        // Skip sensor data collection during grace period
        if (inGracePeriod) return;
        
        // Track overall exploration progress for rewards (less frequent)
        if (worldGrid != null && includeExplorationProgress && Time.frameCount % 60 == 0) // Check every 60 frames (1 second at 60fps)
        {
            float currentExploration = worldGrid.GetExplorationPercentage();
            if (currentExploration > lastExplorationPercentage)
            {
                float explorationGain = currentExploration - lastExplorationPercentage;

                // Get the reward multiplier based on the current exploration percentage
                float rewardMultiplier = GetExplorationRewardMultiplier(currentExploration);

                // Calculate the reward
                float reward = explorationGain * explorationReward * rewardMultiplier;
                // JOINT EXPLORATION REWARD
                AddReward(reward);
                
                if (debugRewards && explorationGain > 0.1f)
                    Debug.Log($"[Drone] Exploration reward: {reward:F4} (Progress: {currentExploration:F1}%, Gain: +{explorationGain:F2}%, Multiplier: {rewardMultiplier:F1})");
                
                lastExplorationPercentage = currentExploration;
            }
        }
    }



    public override void OnEpisodeBegin()
    {
        if (debugLifecycle) Debug.Log("[Drone] OnEpisodeBegin");

        // Start the grace period before resetting the drone
        StartCoroutine(GracePeriod());

        // Reset the drone
        DroneReset();
    }

    private void DroneReset()
    {
        if (!ActiveDrones.Contains(this)) ActiveDrones.Add(this);

        // Disable sensors and mapping during spawn attempts
        // Stops drones mapping while finding valid spawn
        if (droneSensor != null)
        {
            droneSensor.enabled = false;
        }

        Vector3 spawnPos = Vector3.zero; // Default spawn pos
        bool placed = false;

        for (int spawnAttemptIndex = 0; spawnAttemptIndex < maxSpawnAttempts; spawnAttemptIndex++)
        {
            // Generate a random pos within the world bounds
            Vector3 candidate = new Vector3(
                Random.Range(-worldGrid.worldSize.x / 2f, worldGrid.worldSize.x / 2f),
                10f, // Start at 10m above the ground
                Random.Range(-worldGrid.worldSize.z / 2f, worldGrid.worldSize.z / 2f)
            );

            // Ensure the candidate position not too close to other drones
            bool tooClose = false;
            foreach (var other in ActiveDrones)
            {
                if (other == null || other == this) continue;

                if ((other.transform.position - candidate).sqrMagnitude < minSeparation * minSeparation) // Minimum separation
                {
                    tooClose = true;
                    break;
                }
            }

            // Ensure candidate position is not above a structure / colliding
            if (!tooClose && !worldGrid.IsVoxelOccupied(candidate))
            {
                // Perform raycast to ensure the position is at least 5m above ground
                if (Physics.Raycast(candidate, Vector3.down, out RaycastHit hit, 70f, LayerMask.GetMask("Terrain")))
                {
                    if (hit.distance >= 5f)
                    {
                        // Check for collisions using OverlapSphere
                        Collider[] colliders = Physics.OverlapSphere(candidate, 2f, LayerMask.GetMask("Structure"));
                        if (colliders.Length == 0) // No collisions, can place drone
                        {
                            spawnPos = candidate;
                            placed = true;
                            break;
                        }
                    }
                }
            }
        }

        // If no valid position is found, default to the center of the world
        spawnPos.y = 10f; // Ensure the drone is at least 10m above the ground
        transform.position = placed ? spawnPos : new Vector3(0f, 10f, 0f);

        // Random initial rotation
        if (randomizeYaw)
        {
            transform.rotation = Quaternion.Euler(0, Random.Range(0f, 360f), 0);
        }

        // Reset physics
        if (droneRigidbody != null)
        {
            droneRigidbody.velocity = Vector3.zero;
            droneRigidbody.angularVelocity = Vector3.zero;
        }

        // Reset tracking variables
        lastExplorationPercentage = worldGrid != null ? worldGrid.GetExplorationPercentage() : 0f;
        stepCount = 0;

        // Re enable sensors and mapping after spawning
        if (droneSensor != null)
        {
            droneSensor.enabled = true;
        }

        if (!placed)
        {
            Debug.LogWarning("[DroneReset] No valid spawn position found. Using fallback position.");
        }
        else
        {
            Debug.Log($"[DroneReset] Drone successfully reset to position {transform.position}");
        }
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        var discrete = actions.DiscreteActions;

        int horizontalIndex       = discrete[0]; // left/right
        int upDownIndex           = discrete[1]; // down/up
        int forwardBackwardIndex  = discrete[2]; // back/forward

        float horizontalDir      = ToSignedDirection(horizontalIndex);
        float upDownDir          = ToSignedDirection(upDownIndex);
        float forwardBackwardDir = ToSignedDirection(forwardBackwardIndex);

        Vector3 localDirection = new Vector3(horizontalDir, upDownDir, forwardBackwardDir);
        transform.Translate(localDirection * movementSpeed * Time.deltaTime, Space.Self);

        // MOVEMENT REWARD
        float movementMagnitude = localDirection.magnitude;
        if (movementMagnitude > 0.1f)
        {
            AddReward(0.001f * movementMagnitude); // Small positive reward for movement
        }
        // STATIONARY REWARD (penalty)
        else
        {
            AddReward(-0.01f); // Penalty for being stationary
        }

        stepCount++;

        // Reduced debug frequency to avoid spam
        if (stepCount % 1000 == 0)
        {
            Debug.Log($"[Drone] Step {stepCount}, Total Reward: {GetCumulativeReward():F3}");
        }

        // Make sure drone isnt outside world
        CheckWorldBounds();
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        // TESTING, manual control
        var discrete = actionsOut.DiscreteActions;

        float horizontalAxis = Input.GetAxisRaw("Horizontal"); // A/D  left/right
        float forwardBackwardAxis = Input.GetAxisRaw("Vertical"); // W/S  forward/back
        float upDownAxis = 0f; // E up, Q down
        if (Input.GetKey(KeyCode.E)) upDownAxis += 1f;
        if (Input.GetKey(KeyCode.Q)) upDownAxis -= 1f;

        discrete[0] = ToBranchIndex(horizontalAxis);
        discrete[1] = ToBranchIndex(upDownAxis);
        discrete[2] = ToBranchIndex(forwardBackwardAxis);
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // Check if sensor is null
        if (sensor == null)
        {
            Debug.LogError("[Drone] VectorSensor is null!");
            return;
        }
        
        try
        {
            // ######## DRONE STATE OBSERVATIONS ########
            if (includePositionObservations)
            {
                // norm position relative to spawn
                Vector3 currentPos = transform.position;
                Vector3 relativePosition = new Vector3(
                    (currentPos.x - spawnCenter.x) / worldGrid.worldSize.x,
                    (currentPos.y - spawnCenter.y) / worldGrid.worldSize.y,
                    (currentPos.z - spawnCenter.z) / worldGrid.worldSize.z
                );

                sensor.AddObservation(relativePosition.x);
                sensor.AddObservation(relativePosition.y);
                sensor.AddObservation(relativePosition.z);

                // Drone rotation (4 values - quaternion)
                Quaternion rot = transform.rotation;
                sensor.AddObservation(rot.x);
                sensor.AddObservation(rot.y);
                sensor.AddObservation(rot.z);
                sensor.AddObservation(rot.w);
            }

            if (includeVelocityObservations)
            {
                if (droneRigidbody != null)
                {
                    // div by 0 that took me years to fix :(
                    if (movementSpeed <= 0f)
                    {
                        Debug.LogError($"[Drone] Invalid movementSpeed: {movementSpeed}");
                        movementSpeed = 3f;
                    }
                
                    // Linear velocity (3 values)
                    Vector3 linVel = droneRigidbody.velocity / movementSpeed;
                    sensor.AddObservation(linVel.x);
                    sensor.AddObservation(linVel.y);
                    sensor.AddObservation(linVel.z);

                    // Angular velocity (3 values)
                    Vector3 angVel = droneRigidbody.angularVelocity;
                    sensor.AddObservation(angVel.x);
                    sensor.AddObservation(angVel.y);
                    sensor.AddObservation(angVel.z);
                }
                else
                {
                    // Add zero velocity if rigidbody is null
                    for (int i = 0; i < 6; i++)
                    {
                        sensor.AddObservation(0f);
                    }
                }
            }

            // ######## SENSOR OBSERVATIONS ########
            if (includeSensorObservations)
            {
                if (droneSensor != null)
                {
                    // Obstacle avoidance distances (norm) - 16 values
                    float[] avoidanceDistances = droneSensor.GetNormalisedAvoidanceDistances();
                    if (avoidanceDistances.Length != 16)
                    {
                        Debug.LogWarning($"[Drone] Unexpected avoidance distances length: {avoidanceDistances.Length}");
                        avoidanceDistances = new float[16]; // Default to 16 values
                    }
                
                    foreach (float distance in avoidanceDistances)
                    {
                        float safeDistance = float.IsNaN(distance) ? 1.0f : Mathf.Clamp01(distance);
                        sensor.AddObservation(safeDistance);
                    }

                    // Recent mapping data summary (5 values)
                    List<MappingData> mappingData = null;
                
                    try
                    {
                        mappingData = droneSensor.GetMappingData();
                    }
                    catch (System.Exception e)
                    {
                        Debug.LogError($"[Drone] Error getting mapping data: {e.Message}");
                    }
                
                    if (mappingData != null)
                    {
                        float mappingCountNorm = Mathf.Clamp01(mappingData.Count / 100f);
                        sensor.AddObservation(mappingCountNorm);

                        // Average distance to detected obstacles (4 dir)
                        float[] quadrantDistances = null;
                        
                        try
                        {
                            quadrantDistances = CalculateQuadrantAverages(mappingData);
                        }
                        catch (System.Exception e)
                        {
                            Debug.LogError($"[Drone] Error calculating quadrant averages: {e.Message}");
                        }
                        
                        if (quadrantDistances != null && quadrantDistances.Length == 4)
                        {
                            foreach (float distance in quadrantDistances)
                            {
                                float safeDistance = float.IsNaN(distance) ? 1.0f : Mathf.Clamp01(distance);
                                sensor.AddObservation(safeDistance);
                            }
                        }
                        else
                        {
                            // Add default quadrant values
                            for (int i = 0; i < 4; i++)
                            {
                                sensor.AddObservation(1.0f);
                            }
                        }
                    }
                    else
                    {
                        // No mapping data - add defaults
                        sensor.AddObservation(0f); // No mapping hits
                        
                        for (int i = 0; i < 4; i++) // Default quadrants
                        {
                            sensor.AddObservation(1.0f);
                        }
                    }
                }
                else
                {
                    // Sensor is null - add all default values
                    Debug.LogWarning($"[Drone] DroneSensor is null, adding default sensor observations");
                    
                    // Default avoidance distances (16 values)
                    for (int i = 0; i < 16; i++)
                    {
                        sensor.AddObservation(1.0f);
                    }
                    
                    // Default mapping data (5 values)
                    sensor.AddObservation(0f); // No mapping hits
                    
                    for (int i = 0; i < 4; i++) // Default quadrants
                    {
                        sensor.AddObservation(1.0f);
                    }
                }
            }

            // === EXPLORATION PROGRESS ===
            if (includeExplorationProgress)
            {
                if (worldGrid != null)
                {
                    // Global exploration progress (1 value)
                    float explorationPercent = 0f;
                    
                    try
                    {
                        explorationPercent = worldGrid.GetExplorationPercentage();
                    }
                    catch (System.Exception e)
                    {
                        Debug.LogError($"[Drone] Error getting exploration percentage: {e.Message}");
                    }
                    
                    float explorationNorm = Mathf.Clamp01(explorationPercent / 100f);
                    sensor.AddObservation(explorationNorm);

                    // Local area exploration (sample 3x3 grid around drone) (9 values)
                    float[] localExploration = null;
                    
                    try
                    {
                        localExploration = SampleLocalExploration();
                    }
                    catch (System.Exception e)
                    {
                        Debug.LogError($"[Drone] Error sampling local exploration: {e.Message}");
                    }
                    
                    if (localExploration != null && localExploration.Length == 9)
                    {
                        foreach (float exploration in localExploration)
                        {
                            float safeExploration = float.IsNaN(exploration) ? 0f : Mathf.Clamp01(exploration);
                            sensor.AddObservation(safeExploration);
                        }
                    }
                    else
                    {
                        // Default local exploration
                        for (int i = 0; i < 9; i++)
                        {
                            sensor.AddObservation(0f);
                        }
                    }
                }
                else
                {
                    // No world grid - add defaults
                    sensor.AddObservation(0f); // No global exploration
                    
                    for (int i = 0; i < 9; i++) // No local exploration
                    {
                        sensor.AddObservation(0f);
                    }
                }
            }

            // ######## MISSION STATE ########
            // Step count (normalised) (1 value)
            float stepNorm = Mathf.Clamp01(stepCount / 5000f); // Assuming max 5000 steps per episode
            sensor.AddObservation(stepNorm);

            // ######## WORLD BORDER OBSERVATIONS ########
            AddWorldBorderObservations(sensor);

            // ######## NEARBY DRONE OBSERVATIONS ########
            AddNearbyDroneObservations(sensor);

            // Add the distance to a random free voxel outside the range, kinda works
            if (worldGrid != null)
            {
                float distanceToFreeVoxel = worldGrid.GetDistanceToRandomFreeVoxelOutsideRange(transform.position, range: 5, sampleDistance: 6, maxDistance: 100f);
                sensor.AddObservation(distanceToFreeVoxel);

                if (debugRewards)
                {
                    Debug.Log($"[Drone] Distance to random free voxel: {distanceToFreeVoxel:F3}");
                }
            }
            else
            {
                // Add default value if the world grid is not available
                sensor.AddObservation(1.0f); // Max normalised distance
            }
        }
        catch (System.Exception e)
        {
            Debug.LogError($"[Drone] Exception in CollectObservations: {e.Message}\nStackTrace: {e.StackTrace}");
            
            // add minimum required observations
            int requiredObservations = GetObservationCount();
            Debug.LogWarning($"[Drone] Adding fallback observations to reach {requiredObservations} total");
            
            for (int i = 0; i < requiredObservations; i++)
            {
                sensor.AddObservation(0f);
            }
        }
    }

    private void AddWorldBorderObservations(VectorSensor sensor)
    {
        Vector3 position = transform.position;

        // Calc normalised distances to the borders (0 = at border, 1 = at center)
        float xMinDist = Mathf.Clamp01((position.x - worldGrid.worldOrigin.x) / worldGrid.worldSize.x);
        float xMaxDist = Mathf.Clamp01((worldGrid.worldOrigin.x + worldGrid.worldSize.x - position.x) / worldGrid.worldSize.x);
        float zMinDist = Mathf.Clamp01((position.z - worldGrid.worldOrigin.z) / worldGrid.worldSize.z);
        float zMaxDist = Mathf.Clamp01((worldGrid.worldOrigin.z + worldGrid.worldSize.z - position.z) / worldGrid.worldSize.z);

        // Add distances to the sensor
        sensor.AddObservation(xMinDist);
        sensor.AddObservation(xMaxDist);
        sensor.AddObservation(zMinDist);
        sensor.AddObservation(zMaxDist);
    }

    private void AddNearbyDroneObservations(VectorSensor sensor)
    {
        Collider[] nearbyDrones = Physics.OverlapSphere(transform.position, droneSensor.sensorRange * 2f, LayerMask.GetMask("Drone"));
        foreach (Collider droneCollider in nearbyDrones)
        {
            if (droneCollider.gameObject == this.gameObject) continue; // Skip self

            Vector3 relativePosition = (droneCollider.transform.position - transform.position) / (droneSensor.sensorRange * 2f);
            sensor.AddObservation(relativePosition.x);
            sensor.AddObservation(relativePosition.y);
            sensor.AddObservation(relativePosition.z);
        }

        // Add padding if fewer drones are detected than expected
        int maxNearbyDrones = 5; // Example: Assume a max of 5 nearby drones
        int detectedDrones = nearbyDrones.Length - 1; // Exclude self
        for (int i = detectedDrones; i < maxNearbyDrones; i++)
        {
            sensor.AddObservation(0f); // zeros for missing drones
            sensor.AddObservation(0f);
            sensor.AddObservation(0f);
        }
    }

    private float[] CalculateQuadrantAverages(List<MappingData> mappingData)
    {
        // Add comprehensive null checking
        if (mappingData == null || droneSensor == null)
        {
            Debug.LogWarning($"[Drone] CalculateQuadrantAverages: mappingData or droneSensor is null");
            return new float[] { 1.0f, 1.0f, 1.0f, 1.0f }; // No obstacles detected
        }

        float[] quadrants = new float[4]; // North, East, South, West
        int[] counts = new int[4];
        
        foreach (var data in mappingData)
        {
            if (data == null) continue; // Skip null entries
            
            float angle = data.horizontalAngle;
            int quadrant = GetQuadrant(angle);
            
            // Use sensorRange safely
            quadrants[quadrant] += data.distance / droneSensor.sensorRange;
            counts[quadrant]++;
        }
        
        // Average and handle empty quadrants
        for (int i = 0; i < 4; i++)
        {
            quadrants[i] = counts[i] > 0 ? quadrants[i] / counts[i] : 1.0f; // 1.0 = no obstacles
        }
        
        return quadrants;
    }

    private int GetQuadrant(float angle)
    {
        // Convert angle to 0-360 range
        angle = (angle + 360f) % 360f;
        
        if (angle >= 315f || angle < 45f) return 0; // North
        if (angle >= 45f && angle < 135f) return 1; // East  
        if (angle >= 135f && angle < 225f) return 2; // South
        return 3; // West
    }

    private float[] SampleLocalExploration()
    {
        float[] localGrid = new float[9];
        
        if (worldGrid == null) 
        {
            // Return all unknown if no grid
            for (int i = 0; i < 9; i++) localGrid[i] = 0f;
            return localGrid;
        }
        
        Vector3 dronePos = transform.position;
        float sampleRadius = 5f; // Sample 5m around drone
        
        int index = 0;
        for (int x = -1; x <= 1; x++)
        {
            for (int z = -1; z <= 1; z++)
            {
                Vector3 samplePos = dronePos + new Vector3(x * sampleRadius, 0, z * sampleRadius);
                
                if (worldGrid.IsVoxelOccupied(samplePos))
                    localGrid[index] = 1.0f; // Occupied
                else if (worldGrid.IsVoxelFree(samplePos))
                    localGrid[index] = 0.5f; // Free/explored
                else
                    localGrid[index] = 0.0f; // Unknown
                
                index++;
            }
        }
        
        return localGrid;
    }

    public int GetObservationCount()
    {
        int count = 0;
        
        if (includePositionObservations) count += 7; // Position(3) + Rotation(4)
        if (includeVelocityObservations) count += 6; // Linear(3) + Angular(3)
        if (includeSensorObservations && droneSensor != null) 
        {
            count += droneSensor.GetObservationCount(); // Avoidance rays
            count += 5; // Mapping summary
        }
        if (includeExplorationProgress) count += 10; // Global(1) + Local(9)
        count += 1; // Step count
        
        return count;
    }

    private static readonly float[] SignedDirections = { -1f, 0f, 1f };

    private static float ToSignedDirection(int branchIndex)
    {
        return SignedDirections[Mathf.Clamp(branchIndex, 0, 2)];
    }

    private static int ToBranchIndex(float axis)
    {
        if (axis > 0.1f) return 2;  // +1
        if (axis < -0.1f) return 0; // -1
        return 1;                   // 0
    }

    private void OnDestroy()
    {
        ActiveDrones.Remove(this);
    }

    private void CheckWorldBounds()
    {
        Vector3 position = transform.position;

        // Calculate the bounds based on a centered world
        float halfWorldSizeX = worldGrid.worldSize.x / 2f;
        float halfWorldSizeZ = worldGrid.worldSize.z / 2f;

        // Check if the drone is outside the bounds
        if (position.x < worldGrid.worldOrigin.x - halfWorldSizeX || position.x > worldGrid.worldOrigin.x + halfWorldSizeX ||
            position.y < worldGrid.worldOrigin.y || position.y > worldGrid.worldOrigin.y + worldGrid.worldSize.y || // Y bounds are typically not centered
            position.z < worldGrid.worldOrigin.z - halfWorldSizeZ || position.z > worldGrid.worldOrigin.z + halfWorldSizeZ)
        {
            Debug.Log($"[Drone] Out of bounds! Position: {position}, Penalty: {collisionPenalty}");

            // Apply penalty and end the episode
            // OUT OF BOUNDS REWARD (penalty)
            AddReward(collisionPenalty);
            EndEpisode();
        }
    }

    private IEnumerator GracePeriod()
    {
        inGracePeriod = true;
        Debug.Log("[Drone] Grace period started.");
        yield return new WaitForSeconds(gracePeriodDuration);
        inGracePeriod = false;
        Debug.Log("[Drone] Grace period ended.");
    }

    private static bool structuresInitialised = false;

    private float GetExplorationRewardMultiplier(float explorationPercentage)
    {
        // Divide the exploration percentage into 10% tiers
        float tier = explorationPercentage / 10f;
        return tier * tier * tier * 0.1f; // Reward multiplier increases by 0.1 * tier cubed per tier
    }
}

