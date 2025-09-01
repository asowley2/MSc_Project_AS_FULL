using System.Collections.Generic;
using UnityEngine;

public class StructureSpawner : MonoBehaviour
{
    [Header("Structure Generation")]
    [SerializeField] private int structureCount = 10;
    [SerializeField] private Vector2 structureWidthRange = new Vector2(2f, 8f);
    [SerializeField] private Vector2 structureDepthRange = new Vector2(2f, 8f);
    [SerializeField] private Vector2 structureHeightRange = new Vector2(10f, 80f);

    [Header("Spawn Area - Square/Rectangle")]
    [SerializeField] private Vector3 spawnAreaCenter = Vector3.zero;
    [SerializeField] private Vector2 spawnAreaSize = new Vector2(100f, 100f); // X and Z dimensions

    [Header("Structure Settings")]
    [SerializeField] private Material structureMaterial;
    [SerializeField] private bool addColliders = true;
    [SerializeField] private bool isTrigger = true;
    [SerializeField] private string structureTag = "Structure";

    [Header("Debug")]
    [SerializeField] private bool debugSpawning = true;
    [SerializeField] private int maxSpawnAttempts = 50;

    private List<GameObject> spawnedStructures = new List<GameObject>();


    public void SpawnStructures() // Now spawned in drone start
    {
        ClearExistingStructures();

        for (int i = 0; i < structureCount; i++)
        {
            SpawnSingleStructure();
        }

        if (debugSpawning)
            Debug.Log($"[StructureSpawner] Spawned {spawnedStructures.Count} structures");
    }

    private void SpawnSingleStructure()
    {
        Vector3 position;
        Vector3 size;

        if (FindValidStructurePosition(out position, out size))
        {
            GameObject structure = CreateStructure(position, size);
            spawnedStructures.Add(structure);
        }
        else if (debugSpawning)
        {
            Debug.LogWarning("Cannot spawn structure after max attempts");
        }
    }

    private bool FindValidStructurePosition(out Vector3 position, out Vector3 size)
    {
        position = Vector3.zero;
        size = Vector3.zero;

        for (int attempt = 0; attempt < maxSpawnAttempts; attempt++)
        {
            // Random spawn position of candidate structure
            float randomX = Random.Range(-spawnAreaSize.x * 0.5f, spawnAreaSize.x * 0.5f);
            float randomZ = Random.Range(-spawnAreaSize.y * 0.5f, spawnAreaSize.y * 0.5f);
            Vector3 candidate = spawnAreaCenter + new Vector3(randomX, 0f, randomZ);

            // Random size of candidate structure, based on defined ranges
            float width = Random.Range(structureWidthRange.x, structureWidthRange.y);
            float depth = Random.Range(structureDepthRange.x, structureDepthRange.y);
            float height = Random.Range(structureHeightRange.x, structureHeightRange.y);
            Vector3 candidateSize = new Vector3(width, height, depth);

            // Check if position is valid (within bounds)
            if (IsValidStructurePosition(candidate, candidateSize))
            {
                position = candidate;
                position.y = height * 0.5f; // Center structure
                size = candidateSize;
                return true;
            }
        }

        return false;
    }

    private bool IsValidStructurePosition(Vector3 position, Vector3 size)
    {
        // Check if structure fits within spawn area bounds
        float halfWidth = size.x * 0.5f;
        float halfDepth = size.z * 0.5f;

        float minX = spawnAreaCenter.x - spawnAreaSize.x * 0.5f;
        float maxX = spawnAreaCenter.x + spawnAreaSize.x * 0.5f;
        float minZ = spawnAreaCenter.z - spawnAreaSize.y * 0.5f;
        float maxZ = spawnAreaCenter.z + spawnAreaSize.y * 0.5f;

        if (position.x - halfWidth < minX || position.x + halfWidth > maxX ||
            position.z - halfDepth < minZ || position.z + halfDepth > maxZ)
        {
            return false; // Outside of spawn bounds
        }

        return true;
    }

    private GameObject CreateStructure(Vector3 position, Vector3 size)
    {
        GameObject structure = GameObject.CreatePrimitive(PrimitiveType.Cube);
        structure.name = $"Structure_{spawnedStructures.Count}";
        structure.tag = structureTag;

        structure.transform.position = position;
        structure.transform.localScale = size;
        structure.transform.SetParent(this.transform);

        int structureLayer = LayerMask.NameToLayer("Structure");
        if (structureLayer != -1)
        {
            structure.layer = structureLayer;
        }
        else
        {
            Debug.LogWarning("[StructureSpawner] Layer 'Structure' does not exist. Please create it in the Unity Editor.");
        }

        if (addColliders)
        {
            BoxCollider collider = structure.GetComponent<BoxCollider>();
            if (collider != null)
            {
                collider.isTrigger = isTrigger;
            }
        }

        if (structureMaterial != null)
        {
            Renderer renderer = structure.GetComponent<Renderer>();
            if (renderer != null)
            {
                renderer.material = structureMaterial;
            }
        }

        return structure;
    }

    private void ClearExistingStructures()
    {
        foreach (GameObject structure in spawnedStructures)
        {
            if (structure != null)
            {
                Destroy(structure);
            }
        }
        spawnedStructures.Clear();
    }

    public void RegenerateStructures()
    {
        SpawnStructures();
    }

    void OnDrawGizmosSelected()
    {
        Gizmos.color = Color.green;
        Vector3 spawnAreaPos = spawnAreaCenter;
        Vector3 spawnAreaScale = new Vector3(spawnAreaSize.x, 1f, spawnAreaSize.y);
        Gizmos.DrawWireCube(spawnAreaPos, spawnAreaScale);
    }
}
