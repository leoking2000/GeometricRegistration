#pragma once
#include <geo/geometry/Mesh.h>

namespace geo
{
    // Builds triangle-to-triangle adjacency information for a manifold triangle mesh.
    //
    // Two triangles are considered adjacent if they share an undirected edge
    // (i.e. the same pair of vertex indices regardless of winding order).
    //
    // The returned array contains one entry per triangle. For each triangle:
    //
    //   adj[t][0] -> triangle adjacent across edge (v0, v1)
    //   adj[t][1] -> triangle adjacent across edge (v1, v2)
    //   adj[t][2] -> triangle adjacent across edge (v2, v0)
    //
    // If an edge lies on the mesh boundary, the corresponding entry is set to
    // INVALID_INDEX.
    //
    // Assumptions:
    // - The mesh contains only triangles.
    // - The mesh is manifold (each edge is shared by at most two triangles).
    //
    // Complexity:
    // - Time:  O(T)
    // - Space: O(T)
    //
    // @param mesh Input triangle mesh.
    // @return Triangle adjacency table.
    std::vector<glm::uvec3> BuildTriangleAdjacency(const Mesh& mesh);

    // Grows a planar region starting from a seed triangle using a best-first region growing strategy.
    //
    // A neighbouring triangle may join the region if the angular deviation
    // between its face normal and the region's current area-weighted average
    // normal is less than or equal to angleThreshold.
    //
    // The region normal is updated incrementally after each accepted triangle
    // using an area-weighted average of face normals.
    //
    // Neighbour candidates are processed through a min-heap ordered by angular
    // deviation, causing the most compatible triangles to be considered first.
    // Candidate deviation is recomputed when popped from the heap so that the
    // decision reflects the latest region normal.
    //
    // On return, all triangles belonging to the grown region are assigned the
    // specified label in triangleLabel.
    //
    // Complexity:
    // - Time: O(R log R), where R is the number of triangles absorbed.
    // - Space: O(R).
    //
    // @param mesh            Input triangle mesh.
    // @param adj             Triangle adjacency table.
    // @param seedTri         Seed triangle index.
    // @param angleThreshold  Maximum allowed angular deviation (radians).
    // @param triangleLabel   Per-triangle region labels.
    // @param label           Label assigned to all accepted triangles.
    void GrowRegion(const Mesh& mesh, const std::vector<glm::uvec3>& adj, index_t seedTri, f32 angleThreshold,
        std::vector<index_t>& triangleLabel, index_t label);

    // Partitions a mesh into connected regions using repeated normal-based
    // region growing.
    //
    // Each unlabeled triangle becomes a seed for a new region. Region expansion
    // is performed by GrowRegion() using the supplied angular threshold.
    //
    // Every triangle receives exactly one region label.
    //
    // @param mesh            Input triangle mesh.
    // @param adj             Triangle adjacency table.
    // @param angleThreshold  Maximum allowed angular deviation (radians).
    //
    // @return Pair containing:
    //         - Per-triangle region labels.
    //         - Total number of generated regions.
    std::pair<std::vector<index_t>, index_t> RunRegionGrowing(
        const Mesh& mesh,const std::vector<glm::uvec3>& adj, f32 angleThreshold);

    // Dissolves regions smaller than a specified minimum size by reassigning
    // their triangles to neighbouring regions.
    //
    // For each triangle belonging to a region whose size is less than minSize,
    // the triangle is reassigned to the adjacent region containing the largest
    // number of triangles. The process is repeated until no further reassignments
    // occur.
    //
    // After all merges are complete, region labels are compacted so that the
    // resulting label set becomes contiguous: [0, newRegionCount)
    // 
    // @param adj            Triangle adjacency table.
    // @param triangleLabel  Per-triangle region labels. Modified in-place.
    // @param numLabels      Current number of region labels.
    // @param minSize        Minimum allowed region size in triangles.
    //
    // @return Number of remaining regions after merging and relabeling.
    index_t MergeSmallRegions(const std::vector<glm::uvec3>& adj,
        std::vector<index_t>& triangleLabel, index_t numLabels, index_t minSize);

    // each segmented region is classified as belonging to
    // the original intact surface or to the fracture break surface.
    enum class FacetType : u8
    {
        UNKNOWN = 0,    // not yet classified
        INTACT = 1,     // smooth region — original (un-fractured) surface
        FRACTURED = 2   // rough region — created by the break
    };

    // Facet: a connected, geometrically coherent patch of triangles
    struct Facet
    {
        // Triangle indices into the parent Mesh (from which this facet was extracted)
        std::vector<index_t> triangleIndices;

        // Classification result
        FacetType type = FacetType::UNKNOWN;

        // Area-weighted average of the face normals in this facet
        glm::vec3 averageNormal{ 0.0f };

        // Area-weighted centroid of all triangles in this facet
        glm::vec3 centroid{ 0.0f };

        // Sum of triangle areas
        f32 totalArea = 0.0f;

        // Median normalised Vr over all facet vertices, Used as a robust roughness estimator.
        f32 roughnessMedian = 0.0f;

        // Variance of normalised Vr across all facet vertices.
        // High variance -> rough surface -> FRACTURED.
        f32 roughnessVariance = 0.0f;
    };

    // Builds facet descriptors from a triangle segmentation.
    //
    // Triangles sharing the same label are grouped into a single facet.
    // For each facet, geometric statistics are accumulated:
    //
    // - Triangle membership
    // - Total surface area
    // - Area-weighted average normal
    // - Area-weighted centroid
    //
    // Labels are assumed to be compact in the range [0, numLabels).
    // Triangles marked INVALID_INDEX are ignored.
    //
    // @param mesh           Source mesh.
    // @param triangleLabel  Per-triangle facet labels.
    // @param numLabels      Total number of facet labels.
    //
    // @return One Facet structure per label.
    std::vector<Facet> BuildFacets(
        const Mesh& mesh, const std::vector<index_t>& triangleLabel, index_t numLabels);


    struct FacetExtractionParams
    {
        // Maximum angle (radians) between a candidate face normal and the
        // growing region's area-weighted average normal.
        //   Lower  -> more, finer facets (better for curved objects).
        //   Higher -> fewer, coarser facets (tolerates noise).
        f32 normalAngleThreshold = glm::radians(30.0f);

        // Facets with fewer triangles than this are dissolved and their
        // triangles are re-absorbed into the adjacent region with the most
        // triangles.  Eliminates spurious micro-segments produced by the
        // greedy nature of region growing.
        u32 minFacetTriangles = 10;

        // Radius of the sphere-volume integral invariant, expressed as a
        // fraction of the mesh bounding-box diagonal.
        // Determines the spatial scale at which roughness is measured.
        f32 integralInvariantRadiusRatio = 0.02f;

        // Variance threshold for INTACT vs FRACTURED classification.
        // the threshold is biased toward false positives 
        // (intact -> FRACTURED) rather than false negatives (fractured -> INTACT),
        // because missing a fracture surface prevents the matching algorithm
        // from ever finding the correct solution.
        f32 roughnessVarianceThreshold = 0.05f;
    };

    struct FacetExtractionResult
    {
        // Extracted and classified facets
        std::vector<Facet> facets;

        // Per-triangle lookup: triangleToFacet[t] == index into facets[].
        std::vector<index_t> triangleToFacet;

        // triangleAdjacency[t][e] = index of the triangle sharing edge e of triangle t, or INVALID_IDX
        std::vector<glm::uvec3> triangleAdjacency;
    };

    // Runs:
    //   1. Triangle adjacency construction
    //   2. RGBF region growing with global-average normal metric
    //   3. Small-region post-processing
    //   4. Per-facet sphere-volume integral invariant variance
    //   5. Binary classification: INTACT / FRACTURED
    FacetExtractionResult ExtractAndClassifyFacets(const Mesh& mesh, const FacetExtractionParams& params = {});
}
