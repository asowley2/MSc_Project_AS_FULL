using UnityEngine;

public static class SensorCalculations
{
    // Calculate minimum rays needed for 2D horizontal mapping
    public static int CalculateOptimalRayCount(float sensorRange, float gridCellSize)
    {
        float circumferenceAtMaxRange = 2f * Mathf.PI * sensorRange;
        int minRaysNeeded = Mathf.CeilToInt(circumferenceAtMaxRange / gridCellSize);
        int recommendedRays = Mathf.CeilToInt(minRaysNeeded * 1.2f);
        return RoundToNiceNumber(recommendedRays);
    }
    
    // Calculate optimal rays for spherical 3D mapping
    public static int CalculateSphericalRayCount(float sensorRange, float voxelSize)
    {
        // Surface area of sphere at max range
        float sphereSurfaceArea = 4f * Mathf.PI * sensorRange * sensorRange;
        
        // Area covered by each voxel face at max range (voxelSize x voxelSize)
        float voxelFaceArea = voxelSize * voxelSize;
        
        // Minimum rays needed to hit every voxel face
        int minRaysNeeded = Mathf.CeilToInt(sphereSurfaceArea / voxelFaceArea);
        
        // Safety margin
        int recommendedRays = Mathf.CeilToInt(minRaysNeeded * 1.3f);
        
        return RoundToNiceNumber(recommendedRays);
    }
    
    // Calculate optimal vertical layer distribution for spherical mapping
    public static SphericalConfiguration CalculateSphericalConfiguration(float sensorRange, float voxelSize, float verticalAngleRange = 150f)
    {
        int totalRays = CalculateSphericalRayCount(sensorRange, voxelSize);
        
        // Calculate optimal number of vertical layers
        // Based on vertical res needed at max range
        float verticalArcLength = (verticalAngleRange * Mathf.Deg2Rad) * sensorRange;
        int optimalVerticalLayers = Mathf.CeilToInt(verticalArcLength / voxelSize);
        
        // Ensure reasonable bounds
        optimalVerticalLayers = Mathf.Clamp(optimalVerticalLayers, 8, 32);
        
        // Calculate rays per layer
        int[] raysPerLayer = CalculateRaysPerLayer(optimalVerticalLayers, totalRays, verticalAngleRange);
        
        return new SphericalConfiguration
        {
            totalRays = totalRays,
            verticalLayers = optimalVerticalLayers,
            raysPerLayer = raysPerLayer,
            verticalAngleRange = verticalAngleRange
        };
    }
    
    private static int[] CalculateRaysPerLayer(int verticalLayers, int totalRays, float verticalAngleRange)
    {
        int[] raysPerLayer = new int[verticalLayers];
        float startAngle = -verticalAngleRange * 0.5f;
        float angleStep = verticalAngleRange / (verticalLayers - 1);
        
        int remainingRays = totalRays;
        float totalWeight = 0f;
        
        // Calculate weights based on circumference at each latitude
        float[] weights = new float[verticalLayers];
        for (int i = 0; i < verticalLayers; i++)
        {
            float verticalAngle = startAngle + (i * angleStep);
            float cosAngle = Mathf.Cos(verticalAngle * Mathf.Deg2Rad);
            weights[i] = Mathf.Max(0.1f, cosAngle); // Minimum weight to ensure coverage at poles
            totalWeight += weights[i];
        }
        
        // Distribute rays proportionally
        for (int i = 0; i < verticalLayers; i++)
        {
            if (i == verticalLayers - 1)
            {
                // Last layer gets remaining rays
                raysPerLayer[i] = remainingRays;
            }
            else
            {
                int layerRays = Mathf.RoundToInt((weights[i] / totalWeight) * totalRays);
                layerRays = Mathf.Max(8, layerRays); // Minimum 8 rays per layer
                raysPerLayer[i] = layerRays;
                remainingRays -= layerRays;
            }
        }
        
        return raysPerLayer;
    }
    
    private static int RoundToNiceNumber(int value)
    {
        int[] niceNumbers = { 16, 32, 64, 128, 256, 512, 1024, 2048, 4096, 8192 };
        
        foreach (int nice in niceNumbers)
        {
            if (nice >= value) return nice;
        }
        
        return value;
    }
    
    // Calculate the actual mapping res for spherical
    public static float CalculateSphericalResolution(float sensorRange, int totalRays)
    {
        float sphereSurfaceArea = 4f * Mathf.PI * sensorRange * sensorRange;
        float areaPerRay = sphereSurfaceArea / totalRays;
        float linearResolution = Mathf.Sqrt(areaPerRay);
        return linearResolution;
    }
    
    public static float CalculateActualResolution(float sensorRange, int rayCount)
    {
        // Actual res based on ray count
        float anglePerRay = 360f / rayCount;
        float anglePerRayRadians = anglePerRay * Mathf.Deg2Rad;
        float arcLengthAtMaxRange = sensorRange * anglePerRayRadians;
        return arcLengthAtMaxRange;
    }
    
    public static bool MeetsMappingRequirements(float sensorRange, int rayCount, float gridCellSize)
    {
        float actualResolution = CalculateActualResolution(sensorRange, rayCount);
        return actualResolution <= gridCellSize;
    }
    
    public static bool MeetsSphericalMappingRequirements(float sensorRange, int totalRays, float voxelSize)
    {
        float actualResolution = CalculateSphericalResolution(sensorRange, totalRays);
        return actualResolution <= voxelSize;
    }
    
    public static void LogSensorConfiguration(float sensorRange, int rayCount, float gridCellSize)
    {
        float actualResolution = CalculateActualResolution(sensorRange, rayCount);
        bool meetsRequirements = MeetsMappingRequirements(sensorRange, rayCount, gridCellSize);
        
        Debug.Log($"[SensorCalculations] 2D Mapping - " +
                  $"Range: {sensorRange}m, " +
                  $"Rays: {rayCount}, " +
                  $"Target Grid: {gridCellSize}m, " +
                  $"Actual Resolution: {actualResolution:F2}m, " +
                  $"Meets Requirements: {meetsRequirements}");
    }
    
    public static void LogSphericalConfiguration(SphericalConfiguration config, float sensorRange, float voxelSize)
    {
        float actualResolution = CalculateSphericalResolution(sensorRange, config.totalRays);
        bool meetsRequirements = MeetsSphericalMappingRequirements(sensorRange, config.totalRays, voxelSize);
        
        Debug.Log($"[SensorCalculations] Spherical Mapping - " +
                  $"Range: {sensorRange}m, " +
                  $"Total Rays: {config.totalRays}, " +
                  $"Vertical Layers: {config.verticalLayers}, " +
                  $"Target Voxel: {voxelSize}m, " +
                  $"Actual Resolution: {actualResolution:F2}m, " +
                  $"Meets Requirements: {meetsRequirements}");
    }
}

[System.Serializable]
public class SphericalConfiguration
{
    public int totalRays;
    public int verticalLayers;
    public int[] raysPerLayer;
    public float verticalAngleRange;
    
    public override string ToString()
    {
        return $"SphericalConfig(rays:{totalRays}, layers:{verticalLayers}, range:{verticalAngleRange}°)";
    }
}