using System.Collections.Generic;
using UnityEngine;

public class DroneSensor : MonoBehaviour
{
    [Header("Sensor Configuration")]
    [SerializeField] public float sensorRange = 20f;
    [SerializeField] private float mappingGridSize = 1f;
    [SerializeField] private bool autoCalculateRayCount = true;
    
    [Header("Manual Configuration (No Auto)")]
    [SerializeField] private int manualRayCount = 64;
    [SerializeField] private int manualVerticalRayCount = 16;
    
    [Header("Drone Purpose Settings")]
    [SerializeField] private bool enableMapping = true;
    [SerializeField] private bool enableObstacleAvoidance = true;
    
    [Header("Obstacle Avoidance (ML-Agents Observations)")]
    [SerializeField] private int avoidanceRayCount = 16;
    [SerializeField] public float avoidanceRange = 10f;
    
    [Header("Visualization")]
    [SerializeField] private bool showMappingRays = true;
    [SerializeField] private bool showAvoidanceRays = true;
    [SerializeField] private Color mappingHitColour = Color.blue;
    [SerializeField] private Color mappingMissColour = Color.cyan;
    [SerializeField] private Color avoidanceHitColor = Color.red;
    [SerializeField] private Color avoidanceMissColour = Color.green;
    
    [Header("Layer Masks")]
    [SerializeField] private LayerMask mappingLayers = -1;
    [SerializeField] private LayerMask avoidanceLayers = -1;
    
    [Header("Spherical Mapping ")]
    [SerializeField] private bool useSphericalMapping = true;
    [SerializeField] private float verticalAngleRange = 150f; // -75 to +75, can do 360 but more rays, unneccesary
    
    [Header("World Grid Integration")]
    [SerializeField] private bool contributeToWorldMap = true;
    
    // Spherical ray config
    private SphericalConfiguration sphericalConfig;
    private int calculatedMappingRayCount;
    private float[] mappingDistances;
    private List<MappingData> mappingHits = new List<MappingData>();
    
    // Obstacle avoidance data
    private float[] avoidanceDistances;
    
    // World grid reference
    private WorldOccupancyGrid worldGrid;
    
    void Start()
    {
        InitialiseSensor();
        
        if (contributeToWorldMap)
        {
            // Get World Grid (our global mapping grid)
            worldGrid = FindFirstObjectByType<WorldOccupancyGrid>();
            if (worldGrid == null)
            {
                Debug.LogWarning("[DroneSensor] No WorldOccupancyGrid found in scene!");
            }
        }
    }
    
    void FixedUpdate()
    {
        PerformSensorScan(); // Need to do on fixed update (observations need to be made on fixed update)
    }
    
    private void InitialiseSensor()
    {
        if (autoCalculateRayCount && enableMapping)
        {
            if (useSphericalMapping)
            {
                // Num rays to cover all voxels for sphere with rays
                sphericalConfig = SensorCalculations.CalculateSphericalConfiguration(
                    sensorRange, mappingGridSize, verticalAngleRange);
                calculatedMappingRayCount = sphericalConfig.totalRays;
                
                SensorCalculations.LogSphericalConfiguration(sphericalConfig, sensorRange, mappingGridSize);
            }
            else
            {
                // Num rays to accurately cover circle radius
                calculatedMappingRayCount = SensorCalculations.CalculateOptimalRayCount(sensorRange, mappingGridSize);
                SensorCalculations.LogSensorConfiguration(sensorRange, calculatedMappingRayCount, mappingGridSize);
            }
        }
        else
        {
            calculatedMappingRayCount = manualRayCount;
        }
        
        // init mapping and avoidNCE ray arrays
        if (enableMapping)
            mappingDistances = new float[calculatedMappingRayCount];
        
        if (enableObstacleAvoidance)
            avoidanceDistances = new float[avoidanceRayCount];
    }
    
    public void PerformSensorScan()
    {
        if (enableMapping)
            PerformMappingScan();
        
        if (enableObstacleAvoidance)
            PerformAvoidanceScan();
    }
    
    private void PerformMappingScan()
    {
        mappingHits.Clear();
        
        if (useSphericalMapping && sphericalConfig != null)
        {
            PerformSphericalScan();
        }
        else
        {
            PerformHorizontalScan();
        }
    }
    
    private void PerformSphericalScan()
    {
        float startAngle = -verticalAngleRange * 0.5f;
        float verticalStep = verticalAngleRange / (sphericalConfig.verticalLayers - 1);
        
        int rayIndex = 0;
        
        for (int layer = 0; layer < sphericalConfig.verticalLayers; layer++)
        {
            float verticalAngle = startAngle + (layer * verticalStep);
            int raysThisLayer = sphericalConfig.raysPerLayer[layer];
            float horizontalStep = 360f / raysThisLayer;
            
            for (int i = 0; i < raysThisLayer; i++)
            {
                if (rayIndex >= calculatedMappingRayCount) break;
                
                float horizontalAngle = i * horizontalStep;
                Vector3 direction = SphericalToCartesian(horizontalAngle, verticalAngle);
                
                PerformSingleRaycast(rayIndex, direction, horizontalAngle, verticalAngle);
                rayIndex++;
            }
        }
        
    }
    
    private void PerformHorizontalScan()
    {
        float angleStep = 360f / calculatedMappingRayCount;
        
        for (int i = 0; i < calculatedMappingRayCount; i++)
        {
            float angle = i * angleStep;
            Vector3 direction = Quaternion.Euler(0, angle, 0) * transform.forward;
            
            PerformSingleRaycast(i, direction, angle, 0f);
        }
    }
    
    private void PerformSingleRaycast(int rayIndex, Vector3 direction, float horizontalAngle, float verticalAngle)
    {
        Vector3 rayStart = transform.position;
        Vector3 rayEnd = rayStart + direction * sensorRange; // not needed now, cast ray x dist
        
        RaycastHit hit;
        bool hitSomething = Physics.Raycast(rayStart, direction, out hit, sensorRange, mappingLayers);
        
        if (hitSomething)
        {
            mappingDistances[rayIndex] = hit.distance;
            
            mappingHits.Add(new MappingData
            {
                worldPosition = hit.point,
                distance = hit.distance,
                horizontalAngle = horizontalAngle,
                verticalAngle = verticalAngle,
                objectTag = hit.collider.tag,
                normal = hit.normal
            });
            
            // Contribute to world grid
            if (contributeToWorldMap && worldGrid != null)
            {
                worldGrid.ProcessRaycastData(rayStart, hit.point, true, hit.point, hit.collider.tag, hit.collider.gameObject.layer);
            }
            
            if (showMappingRays)
                Debug.DrawRay(rayStart, direction * hit.distance, mappingHitColour);
        }
        else
        {
            mappingDistances[rayIndex] = sensorRange;
            
            // Contribute free space to world grid
            if (contributeToWorldMap && worldGrid != null)
            {
                worldGrid.ProcessRaycastData(rayStart, rayStart + direction * sensorRange, false, Vector3.zero);
            }
            
            if (showMappingRays)
                Debug.DrawRay(rayStart, direction * sensorRange, mappingMissColour);
        }
    }
    
    private Vector3 SphericalToCartesian(float horizontalAngle, float verticalAngle)
    {
        float radH = horizontalAngle * Mathf.Deg2Rad;
        float radV = verticalAngle * Mathf.Deg2Rad;
        
        float x = Mathf.Cos(radV) * Mathf.Sin(radH);
        float y = Mathf.Sin(radV);
        float z = Mathf.Cos(radV) * Mathf.Cos(radH);
        
        return new Vector3(x, y, z);
    }
    
    private void PerformAvoidanceScan()
    {
        float angleStep = 360f / avoidanceRayCount;
        
        for (int i = 0; i < avoidanceRayCount; i++)
        {
            float angle = i * angleStep;
            Vector3 direction = Quaternion.Euler(0, angle, 0) * transform.forward;
            
            RaycastHit hit;
            if (Physics.Raycast(transform.position, direction, out hit, avoidanceRange, avoidanceLayers))
            {
                avoidanceDistances[i] = hit.distance;
                
                if (showAvoidanceRays)
                    Debug.DrawRay(transform.position, direction * hit.distance, avoidanceHitColor);
            }
            else
            {
                avoidanceDistances[i] = avoidanceRange;
                
                if (showAvoidanceRays)
                    Debug.DrawRay(transform.position, direction * avoidanceRange, avoidanceMissColour);
            }
        }
    }

    public float[] GetNormalisedAvoidanceDistances()
    {
        if (!enableObstacleAvoidance || avoidanceDistances == null)
        {
            // Return default values if not ready
            float[] defaults = new float[avoidanceRayCount];
            for (int i = 0; i < defaults.Length; i++)
            {
                defaults[i] = 1.0f; // Max distance (no obstacles)
            }
            return defaults;
        }

        // Normalise distances to 0-1 range
        float[] normalised = new float[avoidanceDistances.Length];
        for (int i = 0; i < avoidanceDistances.Length; i++)
        {
            normalised[i] = avoidanceDistances[i] / avoidanceRange;
        }
        return normalised;
    }
    
    public List<MappingData> GetMappingData()
    {
        return mappingHits; // This should already exist
    }
    
    public int GetObservationCount()
    {
        return enableObstacleAvoidance ? avoidanceRayCount : 0;
    }
    
    public Vector3 GetDronePosition()
    {
        return transform.position;
    }
    
    public float GetDroneHeading()
    {
        return transform.eulerAngles.y;
    }
    
    private void OnDrawGizmos()
    {
        if (!Application.isPlaying) return;
        
        // Simple test sphere to verify gizmos work
        Gizmos.color = Color.yellow;
        Gizmos.DrawWireSphere(transform.position, 1f);
        
        // Draw actual spherical rays
        if (showMappingRays && enableMapping && useSphericalMapping && sphericalConfig != null)
        {
            DrawActualSphericalGizmos();
        }
        else if (showMappingRays && enableMapping && !useSphericalMapping)
        {
            DrawHorizontalGizmos();
        }
    }
    
    private void DrawActualSphericalGizmos()
    {
        // Draw the ACTUAL sphere that sensor uses
        float startAngle = -verticalAngleRange * 0.5f;
        float verticalStep = verticalAngleRange / (sphericalConfig.verticalLayers - 1);
        
        // Every 4th ray 
        int raySkip = Mathf.Max(1, sphericalConfig.totalRays / 200); // Max 200 rays for gizmos
        int drawnRays = 0;
        
        for (int layer = 0; layer < sphericalConfig.verticalLayers; layer++)
        {
            float verticalAngle = startAngle + (layer * verticalStep);
            int raysThisLayer = sphericalConfig.raysPerLayer[layer];
            float horizontalStep = 360f / raysThisLayer;
            
            // Colour code by layer for visualization
            float layerPercent = (float)layer / sphericalConfig.verticalLayers;
            Gizmos.color = Color.Lerp(Color.red, Color.blue, layerPercent);
            
            for (int i = 0; i < raysThisLayer; i += raySkip)
            {
                if (drawnRays > 200) break; // Performance limit
                
                float horizontalAngle = i * horizontalStep;
                Vector3 direction = SphericalToCartesian(horizontalAngle, verticalAngle);
                
                Gizmos.DrawLine(transform.position, transform.position + direction * sensorRange * 0.3f);
                drawnRays++;
            }
        }
        
        // Draw layer indicators
        Gizmos.color = Color.white;
        for (int layer = 0; layer < sphericalConfig.verticalLayers; layer += 2)
        {
            float verticalAngle = startAngle + (layer * verticalStep);
            float radius = sensorRange * 0.2f * Mathf.Cos(verticalAngle * Mathf.Deg2Rad);
            float height = sensorRange * 0.2f * Mathf.Sin(verticalAngle * Mathf.Deg2Rad);
            
            Vector3 center = transform.position + Vector3.up * height;
            
            // Draw circle for this layer
            DrawCircle(center, radius, 16);
        }
    }
    
    private void DrawHorizontalGizmos()
    {
        // Draw horizontal pattern for 2D mode
        float angleStep = 360f / calculatedMappingRayCount;
        int raySkip = Mathf.Max(1, calculatedMappingRayCount / 64);
        
        Gizmos.color = Color.cyan;
        for (int i = 0; i < calculatedMappingRayCount; i += raySkip)
        {
            float angle = i * angleStep;
            Vector3 direction = Quaternion.Euler(0, angle, 0) * transform.forward;
            Gizmos.DrawLine(transform.position, transform.position + direction * sensorRange * 0.5f);
        }
    }
    
    private void DrawCircle(Vector3 center, float radius, int segments)
    {
        if (radius <= 0) return;
        
        float angleStep = 360f / segments;
        Vector3 prevPoint = center + Vector3.forward * radius;
        
        for (int i = 1; i <= segments; i++)
        {
            float angle = i * angleStep;
            Vector3 newPoint = center + Quaternion.Euler(0, angle, 0) * Vector3.forward * radius;
            Gizmos.DrawLine(prevPoint, newPoint);
            prevPoint = newPoint;
        }
    }
}

// Enhanced MappingData for spherical mapping
[System.Serializable]
public class MappingData
{
    public Vector3 worldPosition;
    public float distance;
    public float horizontalAngle;
    public float verticalAngle; // New for spherical
    public string objectTag;
    public Vector3 normal;
    public float timestamp;
    
    // Legacy support
    public float angle
    {
        get { return horizontalAngle; }
        set { horizontalAngle = value; }
    }
    
    public MappingData()
    {
        timestamp = Time.time;
    }
}