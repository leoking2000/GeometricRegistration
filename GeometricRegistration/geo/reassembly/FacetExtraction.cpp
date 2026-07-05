#include <omp.h>
#include <algorithm>
#include <cassert>
#include <cmath>
#include <limits>
#include <queue>
#include <unordered_map>
#include <vector>
#include <geo/logging/LogMacros.h>
#include "FacetExtraction.h"

#pragma warning( disable : 6993)

namespace geo
{
    // glm::acos can return NaN when the argument is slightly outside [-1,1] due to floating-point rounding.  
    // Clamp first.
    static inline f32 SafeAcos(f32 d) noexcept
    {
        return std::acos(glm::clamp(d, -1.f, 1.f));
    }

    // Returns the normalised vector, or zero if the input is near-zero.
    // Used to guard area-weighted normal averaging against degenerate triangles.
    static inline glm::vec3 SafeNormalize(const glm::vec3& v) noexcept
    {
        const f32 len = glm::length(v);
        return (len > std::numeric_limits<f32>::epsilon()) ? v / len : glm::vec3(0.0f);
    }

    // ========================================================================================
    // Triangle Adjacency
    //
    // Two triangles are adjacent iff they share an undirected edge (two vertex
    // indices, regardless of winding order).  We hash each edge (a, b) with a < b
    // packed into a 64-bit key and find paired triangles in a single linear pass.
    //
    // adj[t][e] = index of the triangle sharing edge e of triangle t, or INVALID_IDX
    // for boundary edges.
    // ========================================================================================

    static inline u64 PackEdge(u32 a, u32 b) noexcept
    {
        if (a > b) {
            std::swap(a, b);
        }

        return (u64(a) << 32) | u64(b);
    }

    std::vector<glm::uvec3> BuildTriangleAdjacency(const Mesh& mesh)
    {
        const index_t T = mesh.TriangleCount();

        std::vector<glm::uvec3> adj(T);
        for (glm::uvec3& a : adj)
        {
            a = glm::uvec3(INVALID_INDEX);
        }

        // edge key -> (triangle index, edge slot within that triangle)
        std::unordered_map<u64, std::pair<index_t, u32>> edgeMap;
        edgeMap.reserve(size_t(T) * 2);

        for (index_t t = 0; t < T; t++)
        {
            const TriangleData& tri = mesh.Triangle(t);

            for (u32 e = 0; e < 3; e++)
            {
                const u32 v0  = tri.vertexIndices[e];
                const u32 v1  = tri.vertexIndices[(e + 1) % 3];
                const u64 key = PackEdge(v0, v1);

                auto [it, inserted] = edgeMap.emplace(key, std::make_pair(t, e));
                if (!inserted)
                {
                    // Paired edge found — link the two triangles bidirectionally
                    const auto [ot, oe] = it->second;

                    adj[t][e]   = ot;
                    adj[ot][oe] = t;
                }
            }
        }

        return adj;
    }

    // ========================================================================================
    // Region Growing (RGBF, Global-Average Normal Metric)
    // A min-heap always expands the frontier candidate with the smallest angular
    // deviation from the growing region's current area-weighted average normal.
    // The deviation is re-evaluated on pop (not at enqueue time) so it reflects
    // the most recent region normal - this is important because merging neighbours
    // shifts the average and may bring previously-rejected candidates back inside
    // the threshold.
    //
    // Complexity per region of size R: O(R log R) heap operations.
    // Each unassigned triangle enters the heap at most O(degree) <= 3 times
    // (once per neighbouring triangle that joins the region).
    // ========================================================================================

    void GrowRegion(const Mesh& mesh, const std::vector<glm::uvec3>& adj, index_t seedTri, f32 angleThreshold,
        std::vector<index_t>& triangleLabel, index_t label)
    {
        assert(triangleLabel[seedTri] == INVALID_INDEX);

        const TriangleData& seed = mesh.Triangle(seedTri);

        // Running area-weighted average normal for this region
        glm::vec3 regionNormal = seed.faceNormal;
        f32       regionArea   = seed.area;

        triangleLabel[seedTri] = label;

        // Min-heap on (deviation, triIndex) - "best first"
        using Item = std::pair<f32, index_t>;
        std::priority_queue<Item, std::vector<Item>, std::greater<Item>> pq;

        // Prime the queue with the seed's unassigned neighbours
        for (int e = 0; e < 3; e++)
        {
            const index_t nb = adj[seedTri][e];
            if (nb == INVALID_INDEX || triangleLabel[nb] != INVALID_INDEX) continue;
            pq.push({ SafeAcos(glm::dot(regionNormal, mesh.Triangle(nb).faceNormal)), nb });
        }

        while (!pq.empty())
        {
            const auto [dev, t] = pq.top();
            pq.pop();

            // May have been claimed by a later GrowRegion call (different region)
            // or already absorbed earlier in this call (duplicate queue entry).
            if (triangleLabel[t] != INVALID_INDEX) continue;

            const TriangleData& tri = mesh.Triangle(t);

            // Re-evaluate with the CURRENT region normal (may have shifted since enqueue)
            const f32 angle = SafeAcos(glm::dot(regionNormal, tri.faceNormal));
            if (angle > angleThreshold) continue;   // reject - will become its own seed

            // merge
            triangleLabel[t] = label;

            // Update area-weighted running average normal
            regionNormal = SafeNormalize(regionNormal * regionArea + tri.faceNormal * tri.area);
            regionArea  += tri.area;

            // Expand the frontier
            for (int e = 0; e < 3; ++e)
            {
                const index_t nb = adj[t][e];
                if (nb == INVALID_INDEX || triangleLabel[nb] != INVALID_INDEX) continue;
                pq.push({ SafeAcos(glm::dot(regionNormal, mesh.Triangle(nb).faceNormal)), nb });
            }
        }
    }

    // Seeds one region per unvisited triangle, yielding a complete partition.
    std::pair<std::vector<index_t>, index_t> RunRegionGrowing(
        const Mesh& mesh, const std::vector<glm::uvec3>& adj, f32 angleThreshold)
    {
        const index_t T = mesh.TriangleCount();
        std::vector<index_t> triangleLabel(T, INVALID_INDEX);
        index_t numLabels = 0;

        for (index_t seed = 0; seed < T; seed++)
        {
            if (triangleLabel[seed] != INVALID_INDEX) continue;

            GrowRegion(mesh, adj, seed, angleThreshold, triangleLabel, numLabels);

            numLabels++;
        }

        return { std::move(triangleLabel), numLabels };
    }

    // ========================================================================================
    // Dissolve and Merge Small Regions
    // 
    // Implementation: iterate over all triangles; any triangle whose current facet
    // is below minSize is re-assigned to the adjacent facet with the largest
    // triangle count.  Repeat until no more merges occur.  Then compact the label
    // space by closing gaps left by dissolved regions.
    //
    // Worst-case passes: O(minSize) — in practice 1–3 passes.
    // ========================================================================================

    index_t MergeSmallRegions(
        const std::vector<glm::uvec3>& adj, 
        std::vector<index_t>& triangleLabel, index_t numLabels,
        index_t minSize)
    {
        const index_t T = index_t(triangleLabel.size());

        // Current triangle count per label
        std::vector<index_t> sizes(numLabels, 0);
        for (const index_t lbl : triangleLabel)
        {
            if (lbl != INVALID_INDEX) ++sizes[lbl];
        }

        bool changed = true;
        while (changed)
        {
            changed = false;

            for (index_t t = 0; t < T; ++t)
            {
                const index_t lbl = triangleLabel[t];
                if (lbl == INVALID_INDEX || sizes[lbl] >= minSize) continue;

                // Find the neighbour region with the most triangles (merge target)
                index_t bestLabel = INVALID_INDEX;
                index_t bestSize = 0;

                for (int e = 0; e < 3; ++e)
                {
                    const index_t nb = adj[t][e];
                    if (nb == INVALID_INDEX) continue;
                    const index_t nbLbl = triangleLabel[nb];
                    if (nbLbl == INVALID_INDEX || nbLbl == lbl) continue;

                    if (sizes[nbLbl] > bestSize)
                    {
                        bestSize = sizes[nbLbl];
                        bestLabel = nbLbl;
                    }
                }

                if (bestLabel != INVALID_INDEX)
                {
                    --sizes[lbl];
                    ++sizes[bestLabel];
                    triangleLabel[t] = bestLabel;
                    changed = true;
                }
            }
        }

        // Compact: renumber [0, newCount) to close gaps left by dissolved regions
        std::vector<index_t> remap(numLabels, INVALID_INDEX);
        index_t newCount = 0;

        for (index_t t = 0; t < T; ++t)
        {
            const index_t lbl = triangleLabel[t];
            if (lbl != INVALID_INDEX && remap[lbl] == INVALID_INDEX)
                remap[lbl] = newCount++;
        }

        for (index_t& lbl : triangleLabel)
        {
            if (lbl != INVALID_INDEX) lbl = remap[lbl];
        }

        return newCount;
    }

    // ========================================================================================
    // Build Facet Structs from Triangle Labels
    // ========================================================================================

    std::vector<Facet> BuildFacets(const Mesh& mesh, const std::vector<index_t>& triangleLabel, index_t numLabels)
    {
        std::vector<Facet> facets(numLabels);

        for (index_t t = 0; t < static_cast<index_t>(triangleLabel.size()); ++t)
        {
            const index_t lbl = triangleLabel[t];
            if (lbl == INVALID_INDEX) continue;

            const TriangleData& tri = mesh.Triangle(t);
            Facet& fac = facets[lbl];

            fac.triangleIndices.push_back(t);
            fac.averageNormal += tri.faceNormal * tri.area;
            fac.totalArea += tri.area;

            // Area-weighted centroid contribution
            const glm::vec3 triCentroid =
                (mesh.TriangleVertex(t, 0) +
                    mesh.TriangleVertex(t, 1) +
                    mesh.TriangleVertex(t, 2)) * (1.f / 3.f);
            fac.centroid += triCentroid * tri.area;
        }

        for (Facet& fac : facets)
        {
            if (fac.totalArea > std::numeric_limits<f32>::epsilon())
            {
                fac.averageNormal = SafeNormalize(fac.averageNormal);
                fac.centroid /= fac.totalArea;
            }
        }

        return facets;
    }



    FacetExtractionResult ExtractAndClassifyFacets(const Mesh& mesh, const FacetExtractionParams& params)
    {
        return FacetExtractionResult();
    }


}