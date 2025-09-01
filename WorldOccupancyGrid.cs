using UnityEngine;
using System.Collections.Generic;

// Basically a 3d array for voxels
public class WorldOccupancyGrid : MonoBehaviour
{
    [Header("World Configuration")]
    [SerializeField] public Vector3 worldSize = new Vector3(300f, 70f, 300f);
    [SerializeField] private float voxelSize = 4f; // 4m cubed voxels
    [SerializeField] public Vector3 worldOrigin = new Vector3(0f, 0f, 0f);

    [Header("Occupancy States")]
    [SerializeField] private bool debugMapping = true;

    // 3D array: [x, y, z] = VoxelState
    private VoxelState[,,] occupancyGrid;
    private Vector3Int gridDimensions;

    // Stats
    private int totalVoxels;
    private int exploredVoxels;
    private int occupiedVoxels;

    private float floodFillInterval = 120f; // Every 120 seconds
    private float lastFloodFillTime = 0f;

    void Start()
    {
        InitialiseGrid();
    }

    private void InitialiseGrid()
    {
        // Calculate grid dimensions
        gridDimensions = new Vector3Int(
            Mathf.CeilToInt(worldSize.x / voxelSize),
            Mathf.CeilToInt(worldSize.y / voxelSize),
            Mathf.CeilToInt(worldSize.z / voxelSize)
        );

        // Init 3D array
        occupancyGrid = new VoxelState[gridDimensions.x, gridDimensions.y, gridDimensions.z];
        totalVoxels = gridDimensions.x * gridDimensions.y * gridDimensions.z;

        // Init all voxels as unknown
        for (int x = 0; x < gridDimensions.x; x++)
        {
            for (int y = 0; y < gridDimensions.y; y++)
            {
                for (int z = 0; z < gridDimensions.z; z++)
                {
                    occupancyGrid[x, y, z] = VoxelState.Unknown;
                }
            }
        }

        if (debugMapping)
            Debug.Log($"[WorldOccupancyGrid] Initialised {gridDimensions} grid with {totalVoxels:N0} voxels");
    }

    // Process raycast data from drone sensors
    public void ProcessRaycastData(Vector3 rayStart, Vector3 rayEnd, bool hitSomething, Vector3 hitPoint, string hitTag = "", int hitLayer = -1)
    {
        // Counters for debug
        int airVoxelsMarked = 0;
        int structureVoxelsMarked = 0;
        int terrainVoxelsMarked = 0;

        if (hitSomething)
        {
            Vector3Int hitVoxel = WorldToGrid(hitPoint);

            // Mark freespace along ray path first
            airVoxelsMarked += MarkRayAsFreespace(rayStart, hitPoint);

            // Handle different object types based on layer or tag
            if (hitLayer == LayerMask.NameToLayer("Terrain"))
            {
                terrainVoxelsMarked += HandleFloorHit(hitPoint, hitVoxel);
            }
            else if (hitLayer == LayerMask.NameToLayer("Structure"))
            {

                if (IsValidVoxel(hitVoxel) && GetVoxelState(hitVoxel) == VoxelState.Unknown)
                {
                    SetVoxelState(hitVoxel, VoxelState.Occupied);
                    structureVoxelsMarked++;
                }
            }
            else
            {
                // Default behavior for unknown layers
                if (IsValidVoxel(hitVoxel) && (GetVoxelState(hitVoxel) == VoxelState.Unknown || GetVoxelState(hitVoxel) == VoxelState.Free))
                {
                    SetVoxelState(hitVoxel, VoxelState.Occupied);
                    structureVoxelsMarked++;
                }
            }
        }
        else
        {
            // Mark entire ray as free space
            airVoxelsMarked += MarkRayAsFreespace(rayStart, rayEnd);
        }
    }

    private int HandleFloorHit(Vector3 hitPoint, Vector3Int hitVoxel)
    {
        int terrainVoxelCount = 0;

        // Mark the voxel directly hit by the ray
        if (IsValidVoxel(hitVoxel) && (GetVoxelState(hitVoxel) == VoxelState.Unknown || GetVoxelState(hitVoxel) == VoxelState.Free))
        {
            SetVoxelState(hitVoxel, VoxelState.Occupied);
            terrainVoxelCount++;
        }
        return terrainVoxelCount;
    }

    private int MarkRayAsFreespace(Vector3 start, Vector3 end)
    {
        int freeVoxelCount = 0;

        Vector3 direction = (end - start).normalized;
        float rayLength = Vector3.Distance(start, end);
        float step = voxelSize * 0.1f; // Sample every 10% of a voxel

        for (float distance = 0; distance <= rayLength; distance += step)
        {
            Vector3 samplePoint = start + direction * distance;
            Vector3Int voxel = WorldToGrid(samplePoint);

            if (IsValidVoxel(voxel) && GetVoxelState(voxel) == VoxelState.Unknown)
            {
                SetVoxelState(voxel, VoxelState.Free);
                freeVoxelCount++;
            }
        }

        Vector3Int finalVoxel = WorldToGrid(end);
        if (IsValidVoxel(finalVoxel) && GetVoxelState(finalVoxel) == VoxelState.Unknown)
        {
            SetVoxelState(finalVoxel, VoxelState.Free);
            freeVoxelCount++;
        }

        return freeVoxelCount;
    }

    private void FillEnclosedAreas()
    {
        for (int y = 0; y < gridDimensions.y; y++) // Iterate over each horizontal slice
        {
            bool[,] visited = new bool[gridDimensions.x, gridDimensions.z];

            for (int x = 0; x < gridDimensions.x; x++)
            {
                for (int z = 0; z < gridDimensions.z; z++)
                {
                    // Skip already visited or occupied voxels
                    if (visited[x, z] || GetVoxelState(new Vector3Int(x, y, z)) == VoxelState.Occupied)
                        continue;

                    // Perform flood-fill to find the region
                    List<Vector3Int> region = new List<Vector3Int>();
                    bool isEnclosed = FloodFill2D(x, z, y, visited, region);

                    // If the region is enclosed, mark all voxels in the region as occupied
                    if (isEnclosed)
                    {
                        foreach (Vector3Int voxel in region)
                        {
                            SetVoxelState(voxel, VoxelState.Occupied);
                        }

                        if (debugMapping)
                        {
                            Debug.Log($"[FloodFill2D] Enclosed region found with {region.Count} voxels at y={y}.");
                        }
                    }
                }
            }
        }
    }

    private bool FloodFill2D(int startX, int startZ, int y, bool[,] visited, List<Vector3Int> region)
    {
        Queue<Vector3Int> queue = new Queue<Vector3Int>();
        queue.Enqueue(new Vector3Int(startX, y, startZ));
        bool isEnclosed = true;

        while (queue.Count > 0)
        {
            Vector3Int current = queue.Dequeue();
            int x = current.x;
            int z = current.z;

            // Skip if out of bounds or already visited
            if (x < 0 || x >= gridDimensions.x || z < 0 || z >= gridDimensions.z || visited[x, z])
                continue;

            visited[x, z] = true;
            region.Add(current);

            // If the voxel is on the edge of the grid, it's not enclosed
            if (x == 0 || x == gridDimensions.x - 1 || z == 0 || z == gridDimensions.z - 1)
            {
                isEnclosed = false;
            }

            // Check neighbors (4-connectivity: up, down, left, right)
            if (GetVoxelState(new Vector3Int(x, y, z)) != VoxelState.Occupied)
            {
                queue.Enqueue(new Vector3Int(x + 1, y, z));
                queue.Enqueue(new Vector3Int(x - 1, y, z));
                queue.Enqueue(new Vector3Int(x, y, z + 1));
                queue.Enqueue(new Vector3Int(x, y, z - 1));
            }
        }

        return isEnclosed;
    }

    public Vector3Int WorldToGrid(Vector3 worldPos)
    {
        // Shift the world position to align with the grid's positive index range
        Vector3 relative = worldPos + (worldSize / 2f); // Add half the world size to shift the center to (0, 0, 0)
        return new Vector3Int(
            Mathf.FloorToInt(relative.x / voxelSize),
            Mathf.FloorToInt(relative.y / voxelSize),
            Mathf.FloorToInt(relative.z / voxelSize)
        );
    }

    private Vector3 GridToWorld(Vector3Int gridPos)
    {
        return -worldSize / 2f + worldOrigin + new Vector3(
            gridPos.x * voxelSize + voxelSize * 0.5f,
            gridPos.y * voxelSize + voxelSize * 0.5f,
            gridPos.z * voxelSize + voxelSize * 0.5f
        );
    }

    private bool IsValidVoxel(Vector3Int voxel)
    {
        return voxel.x >= 0 && voxel.x < gridDimensions.x &&
               voxel.y >= 0 && voxel.y < gridDimensions.y &&
               voxel.z >= 0 && voxel.z < gridDimensions.z;
    }

    private VoxelState GetVoxelState(Vector3Int voxel)
    {
        if (!IsValidVoxel(voxel)) return VoxelState.Unknown;
        return occupancyGrid[voxel.x, voxel.y, voxel.z];
    }

    private void SetVoxelState(Vector3Int voxel, VoxelState state)
    {
        if (!IsValidVoxel(voxel)) return;

        VoxelState oldState = occupancyGrid[voxel.x, voxel.y, voxel.z];
        occupancyGrid[voxel.x, voxel.y, voxel.z] = state;

        // Update statistics
        if (oldState == VoxelState.Unknown && state != VoxelState.Unknown)
        {
            exploredVoxels++;
        }
        if (oldState != VoxelState.Occupied && state == VoxelState.Occupied)
        {
            occupiedVoxels++;
        }
        else if (oldState == VoxelState.Occupied && state != VoxelState.Occupied)
        {
            occupiedVoxels--;
        }
    }

    // Public API for querying the grid
    public bool IsVoxelOccupied(Vector3 worldPos)
    {
        Vector3Int voxel = WorldToGrid(worldPos);
        return GetVoxelState(voxel) == VoxelState.Occupied;
    }

    public bool IsVoxelFree(Vector3 worldPos)
    {
        Vector3Int voxel = WorldToGrid(worldPos);
        return GetVoxelState(voxel) == VoxelState.Free;
    }

    public float GetExplorationPercentage()
    {
        return (float)exploredVoxels / totalVoxels * 100f;
    }

    // Export grid data for analysis/visualization
    public VoxelState[,,] GetOccupancyGrid()
    {
        return (VoxelState[,,])occupancyGrid.Clone();
    }

    public void ClearGrid()
    {
        InitialiseGrid();
        exploredVoxels = 0;
        occupiedVoxels = 0;

        if (debugMapping)
            Debug.Log("[WorldOccupancyGrid] Grid cleared");
    }

    // Statistics
    void Update()
    {
        if (Time.time - lastFloodFillTime > floodFillInterval)
        {
            TriggerFloodFill();
            lastFloodFillTime = Time.time;
        }

        if (debugMapping && Time.frameCount % 300 == 0) // Every 10 seconds
        {
            Debug.Log($"[WorldOccupancyGrid] Exploration: {GetExplorationPercentage():F1}% " +
                     $"({exploredVoxels:N0}/{totalVoxels:N0} voxels), " +
                     $"Occupied: {occupiedVoxels:N0}");
        }
    }

    public void TriggerFloodFill()
    {
        if (debugMapping)
            Debug.Log("[WorldOccupancyGrid] Triggering flood-fill for enclosed areas...");

        FillEnclosedAreas();
    }

    public Vector3Int GetCurrentVoxel(Vector3 worldPos)
    {
        return WorldToGrid(worldPos);
    }

    public float GetDistanceToRandomFreeVoxelOutsideRange(Vector3 worldPos, int range = 5, int sampleDistance = 6, float maxDistance = 100f)
    {
        Vector3Int currentVoxel = GetCurrentVoxel(worldPos);
        List<Vector3Int> sampledVoxels = new List<Vector3Int>();

        // Sample voxels at a distance of `sampleDistance` from the current voxel
        for (int x = -sampleDistance; x <= sampleDistance; x += sampleDistance)
        {
            for (int y = -sampleDistance; y <= sampleDistance; y += sampleDistance)
            {
                for (int z = -sampleDistance; z <= sampleDistance; z += sampleDistance)
                {
                    // Skip voxels within the immediate range
                    if (Mathf.Abs(x) <= range && Mathf.Abs(y) <= range && Mathf.Abs(z) <= range)
                        continue;

                    Vector3Int sampledVoxel = currentVoxel + new Vector3Int(x, y, z);

                    // Check if the sampled voxel is valid and free
                    if (IsValidVoxel(sampledVoxel) && GetVoxelState(sampledVoxel) == VoxelState.Free)
                    {
                        sampledVoxels.Add(sampledVoxel);
                    }
                }
            }
        }

        // If no free voxels are found, return the maximum norm distance
        if (sampledVoxels.Count == 0)
        {
            return 1.0f; // No free voxels found
        }

        // Randomly select one of the free voxels
        Vector3Int selectedVoxel = sampledVoxels[Random.Range(0, sampledVoxels.Count)];
        Vector3 selectedVoxelWorldPos = GridToWorld(selectedVoxel);

        // Calculate the distance to the selected voxel
        float distance = Vector3.Distance(worldPos, selectedVoxelWorldPos);

        // Norm the distance to a range of 0 to 1
        return Mathf.Clamp01(distance / maxDistance);
    }
}



public enum VoxelState
{
    Unknown,    // Not yet explored
    Free,       // Air/empty space
    Occupied    // Contains structure/obstacle
}