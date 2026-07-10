#pragma once
#include <gtest/gtest.h>
#include <geo/GeometricRegistration.h>

TEST(BuildTriangleAdjacencyTest, EmptyMesh)
{
    geo::Mesh mesh(
        "empty",
        {},
        {}
    );

    const auto adj = geo::BuildTriangleAdjacency(mesh);

    EXPECT_TRUE(adj.empty());
}

TEST(BuildTriangleAdjacencyTest, SingleTriangle)
{
    geo::Mesh mesh(
        "single",
        {
            {0.f, 0.f, 0.f},
            {1.f, 0.f, 0.f},
            {0.f, 1.f, 0.f}
        },
        {
            {0, 1, 2}
        }
    );

    const auto adj = geo::BuildTriangleAdjacency(mesh);

    ASSERT_EQ(adj.size(), 1u);

    EXPECT_EQ(adj[0][0], INVALID_INDEX);
    EXPECT_EQ(adj[0][1], INVALID_INDEX);
    EXPECT_EQ(adj[0][2], INVALID_INDEX);
}

TEST(BuildTriangleAdjacencyTest, TwoTrianglesSharingOneEdge)
{
    geo::Mesh mesh(
        "two_tris",
        {
            {0.f, 0.f, 0.f}, // 0
            {1.f, 0.f, 0.f}, // 1
            {0.f, 1.f, 0.f}, // 2
            {1.f, 1.f, 0.f}  // 3
        },
        {
            {0, 1, 2}, // t0
            {2, 1, 3}  // t1
        }
    );

    const auto adj = geo::BuildTriangleAdjacency(mesh);

    ASSERT_EQ(adj.size(), 2u);

    EXPECT_EQ(adj[0][0], INVALID_INDEX);
    EXPECT_EQ(adj[0][1], 1u);
    EXPECT_EQ(adj[0][2], INVALID_INDEX);

    EXPECT_EQ(adj[1][0], 0u);
    EXPECT_EQ(adj[1][1], INVALID_INDEX);
    EXPECT_EQ(adj[1][2], INVALID_INDEX);
}

TEST(BuildTriangleAdjacencyTest, SharedEdgeDetectedRegardlessOfWinding)
{
    geo::Mesh mesh(
        "winding",
        {
            {0.f, 0.f, 0.f}, // 0
            {1.f, 0.f, 0.f}, // 1
            {0.f, 1.f, 0.f}, // 2
            {1.f, 1.f, 0.f}  // 3
        },
        {
            {0, 1, 2},
            {1, 2, 3}
        }
    );

    const auto adj = geo::BuildTriangleAdjacency(mesh);

    ASSERT_EQ(adj.size(), 2u);

    bool found01 = false;
    bool found10 = false;

    for (u32 e = 0; e < 3; ++e)
    {
        found01 |= (adj[0][e] == 1u);
        found10 |= (adj[1][e] == 0u);
    }

    EXPECT_TRUE(found01);
    EXPECT_TRUE(found10);
}

TEST(BuildTriangleAdjacencyTest, TetrahedronHasNoBoundaryEdges)
{
    geo::Mesh mesh(
        "tetra",
        {
            {0.f, 0.f, 0.f}, // 0
            {1.f, 0.f, 0.f}, // 1
            {0.f, 1.f, 0.f}, // 2
            {0.f, 0.f, 1.f}  // 3
        },
        {
            {0, 1, 2},
            {0, 3, 1},
            {1, 3, 2},
            {0, 2, 3}
        }
    );

    const auto adj = geo::BuildTriangleAdjacency(mesh);

    ASSERT_EQ(adj.size(), 4u);

    for (const auto& a : adj)
    {
        EXPECT_NE(a[0], INVALID_INDEX);
        EXPECT_NE(a[1], INVALID_INDEX);
        EXPECT_NE(a[2], INVALID_INDEX);
    }
}

TEST(RunRegionGrowingTest, EmptyMesh)
{
    geo::Mesh mesh(
        "empty",
        {},
        {}
    );

    auto adj = geo::BuildTriangleAdjacency(mesh);

    auto [labels, numRegions] =
        geo::RunRegionGrowing(mesh, adj, glm::radians(10.0f));

    EXPECT_TRUE(labels.empty());
    EXPECT_EQ(numRegions, 0u);
}

TEST(RunRegionGrowingTest, SingleTriangleProducesOneRegion)
{
    geo::Mesh mesh(
        "single",
        {
            {0,0,0},
            {1,0,0},
            {0,1,0}
        },
        {
            {0,1,2}
        }
    );

    auto adj = geo::BuildTriangleAdjacency(mesh);

    auto [labels, numRegions] =
        geo::RunRegionGrowing(mesh, adj, glm::radians(10.0f));

    ASSERT_EQ(labels.size(), 1u);

    EXPECT_EQ(numRegions, 1u);
    EXPECT_EQ(labels[0], 0u);
}

TEST(RunRegionGrowingTest, CoplanarQuadProducesSingleRegion)
{
    geo::Mesh mesh(
        "quad",
        {
            {0,0,0},
            {1,0,0},
            {1,1,0},
            {0,1,0}
        },
        {
            {0,1,2},
            {0,2,3}
        }
    );

    auto adj = geo::BuildTriangleAdjacency(mesh);

    auto [labels, numRegions] =
        geo::RunRegionGrowing(mesh, adj, glm::radians(1.0f));

    ASSERT_EQ(labels.size(), 2u);

    EXPECT_EQ(numRegions, 1u);
    EXPECT_EQ(labels[0], labels[1]);
}

TEST(RunRegionGrowingTest, DisconnectedTrianglesProduceSeparateRegions)
{
    geo::Mesh mesh(
        "disconnected",
        {
            {0,0,0},
            {1,0,0},
            {0,1,0},

            {10,0,0},
            {11,0,0},
            {10,1,0}
        },
        {
            {0,1,2},
            {3,4,5}
        }
    );

    auto adj = geo::BuildTriangleAdjacency(mesh);

    auto [labels, numRegions] =
        geo::RunRegionGrowing(mesh, adj, glm::radians(10.0f));

    ASSERT_EQ(labels.size(), 2u);

    EXPECT_EQ(numRegions, 2u);
    EXPECT_NE(labels[0], labels[1]);
}

TEST(RunRegionGrowingTest, NinetyDegreeFoldProducesTwoRegions)
{
    geo::Mesh mesh(
        "fold",
        {
            {0,0,0}, // 0
            {1,0,0}, // 1
            {0,1,0}, // 2
            {0,0,1}  // 3
        },
        {
            {0,1,2},
            {0,2,3}
        }
    );

    auto adj = geo::BuildTriangleAdjacency(mesh);

    auto [labels, numRegions] =
        geo::RunRegionGrowing(mesh, adj, glm::radians(30.0f));

    ASSERT_EQ(labels.size(), 2u);

    EXPECT_EQ(numRegions, 2u);
    EXPECT_NE(labels[0], labels[1]);
}

TEST(RunRegionGrowingTest, NinetyDegreeFoldMergedWithLargeThreshold)
{
    geo::Mesh mesh(
        "fold",
        {
            {0,0,0},
            {1,0,0},
            {0,1,0},
            {0,0,1}
        },
        {
            {0,1,2},
            {0,2,3}
        }
    );

    auto adj = geo::BuildTriangleAdjacency(mesh);

    auto [labels, numRegions] =
        geo::RunRegionGrowing(mesh, adj, glm::radians(100.0f));

    ASSERT_EQ(labels.size(), 2u);

    EXPECT_EQ(numRegions, 1u);
    EXPECT_EQ(labels[0], labels[1]);
}

TEST(RunRegionGrowingTest, EveryTriangleReceivesAValidLabel)
{
    geo::Mesh mesh(
        "tetra",
        {
            {0,0,0},
            {1,0,0},
            {0,1,0},
            {0,0,1}
        },
        {
            {0,1,2},
            {0,3,1},
            {1,3,2},
            {0,2,3}
        }
    );

    auto adj = geo::BuildTriangleAdjacency(mesh);

    auto [labels, numRegions] =
        geo::RunRegionGrowing(mesh, adj, glm::radians(30.0f));

    ASSERT_EQ(labels.size(), mesh.TriangleCount());

    for (auto l : labels)
    {
        EXPECT_NE(l, INVALID_INDEX);
    }

    EXPECT_EQ(numRegions, 4u);
}

TEST(MergeSmallRegionsTest, EmptyInput)
{
    std::vector<glm::uvec3> adj;
    std::vector<index_t> labels;

    const auto numRegions =
        geo::MergeSmallRegions(adj, labels, 0, 2);

    EXPECT_EQ(numRegions, 0u);
    EXPECT_TRUE(labels.empty());
}

TEST(MergeSmallRegionsTest, SingleLargeRegionUnchanged)
{
    std::vector<glm::uvec3> adj =
    {
        {INVALID_INDEX, INVALID_INDEX, 1},
        {0, INVALID_INDEX, INVALID_INDEX}
    };

    std::vector<index_t> labels =
    {
        0, 0
    };

    const auto numRegions =
        geo::MergeSmallRegions(adj, labels, 1, 2);

    EXPECT_EQ(numRegions, 1u);
    EXPECT_EQ(labels[0], 0u);
    EXPECT_EQ(labels[1], 0u);
}

TEST(MergeSmallRegionsTest, SmallRegionMergedIntoLargerNeighbour)
{
    std::vector<glm::uvec3> adj =
    {
        {1, INVALID_INDEX, INVALID_INDEX},
        {0, 2, INVALID_INDEX},
        {1, INVALID_INDEX, INVALID_INDEX}
    };

    std::vector<index_t> labels =
    {
        0,
        1,
        1
    };

    const auto numRegions =
        geo::MergeSmallRegions(adj, labels, 2, 2);

    EXPECT_EQ(numRegions, 1u);

    EXPECT_EQ(labels[0], 0u);
    EXPECT_EQ(labels[1], 0u);
    EXPECT_EQ(labels[2], 0u);
}

TEST(MergeSmallRegionsTest, LabelCompactionRemovesGaps)
{
    std::vector<glm::uvec3> adj =
    {
        {1, INVALID_INDEX, INVALID_INDEX},
        {0, INVALID_INDEX, INVALID_INDEX}
    };

    std::vector<index_t> labels =
    {
        0,
        2
    };

    const auto numRegions =
        geo::MergeSmallRegions(adj, labels, 3, 2);

    EXPECT_EQ(numRegions, 1u);

    EXPECT_EQ(labels[0], 0u);
    EXPECT_EQ(labels[1], 0u);
}

TEST(MergeSmallRegionsTest, RegionWithoutNeighboursCannotBeMerged)
{
    std::vector<glm::uvec3> adj =
    {
        {INVALID_INDEX, INVALID_INDEX, INVALID_INDEX}
    };

    std::vector<index_t> labels =
    {
        0
    };

    const auto numRegions =
        geo::MergeSmallRegions(adj, labels, 1, 10);

    EXPECT_EQ(numRegions, 1u);
    EXPECT_EQ(labels[0], 0u);
}

TEST(MergeSmallRegionsTest, MultipleSmallRegionsCollapseIntoLargest)
{
    std::vector<glm::uvec3> adj =
    {
        {1, INVALID_INDEX, INVALID_INDEX}, // t0
        {0, 2, INVALID_INDEX},             // t1
        {1, 3, INVALID_INDEX},             // t2
        {2, 4, INVALID_INDEX},             // t3
        {3, INVALID_INDEX, INVALID_INDEX}  // t4
    };

    std::vector<index_t> labels =
    {
        0,
        1,
        1,
        2,
        2
    };

    const auto numRegions =
        geo::MergeSmallRegions(adj, labels, 3, 3);

    EXPECT_EQ(numRegions, 1u);

    for (auto l : labels)
    {
        EXPECT_EQ(l, 0u);
    }
}

TEST(MergeSmallRegionsTest, OutputLabelsAreCompact)
{
    std::vector<glm::uvec3> adj =
    {
        {1, INVALID_INDEX, INVALID_INDEX},
        {0, INVALID_INDEX, INVALID_INDEX},
        {INVALID_INDEX, INVALID_INDEX, INVALID_INDEX}
    };

    std::vector<index_t> labels =
    {
        5,
        5,
        9
    };

    const auto numRegions =
        geo::MergeSmallRegions(adj, labels, 10, 1);

    EXPECT_EQ(numRegions, 2u);

    std::set<index_t> unique(labels.begin(), labels.end());

    EXPECT_EQ(unique.size(), 2u);
    EXPECT_TRUE(unique.count(0));
    EXPECT_TRUE(unique.count(1));
}

TEST(BuildFacetsTest, EmptyInput)
{
    geo::Mesh mesh(
        "empty",
        {},
        {}
    );

    std::vector<index_t> labels;

    auto facets = geo::BuildFacets(mesh, labels, 0);

    EXPECT_TRUE(facets.empty());
}

TEST(BuildFacetsTest, SingleTriangleFacet)
{
    geo::Mesh mesh(
        "single",
        {
            {0.f, 0.f, 0.f},
            {1.f, 0.f, 0.f},
            {0.f, 1.f, 0.f}
        },
        {
            {0, 1, 2}
        }
    );

    std::vector<index_t> labels = { 0 };

    auto facets = geo::BuildFacets(mesh, labels, 1);

    ASSERT_EQ(facets.size(), 1u);

    const auto& f = facets[0];

    EXPECT_EQ(f.triangleIndices.size(), 1u);
    EXPECT_EQ(f.triangleIndices[0], 0u);

    EXPECT_GT(f.totalArea, 0.0f);

    EXPECT_NEAR(glm::length(f.averageNormal), 1.0f, 1e-5f);

    EXPECT_NEAR(f.centroid.x, 1.0f / 3.0f, 1e-5f);
    EXPECT_NEAR(f.centroid.y, 1.0f / 3.0f, 1e-5f);
    EXPECT_NEAR(f.centroid.z, 0.0f, 1e-5f);
}

TEST(BuildFacetsTest, TwoTrianglesSameFacet)
{
    geo::Mesh mesh(
        "quad",
        {
            {0.f, 0.f, 0.f},
            {1.f, 0.f, 0.f},
            {1.f, 1.f, 0.f},
            {0.f, 1.f, 0.f}
        },
        {
            {0, 1, 2},
            {0, 2, 3}
        }
    );

    std::vector<index_t> labels =
    {
        0,
        0
    };

    auto facets = geo::BuildFacets(mesh, labels, 1);

    ASSERT_EQ(facets.size(), 1u);

    EXPECT_EQ(facets[0].triangleIndices.size(), 2u);

    EXPECT_NEAR(glm::length(facets[0].averageNormal), 1.0f, 1e-5f);

    EXPECT_GT(facets[0].totalArea, 0.0f);
}

TEST(BuildFacetsTest, TwoIndependentFacets)
{
    geo::Mesh mesh(
        "quad",
        {
            {0.f, 0.f, 0.f},
            {1.f, 0.f, 0.f},
            {1.f, 1.f, 0.f},
            {0.f, 1.f, 0.f}
        },
        {
            {0, 1, 2},
            {0, 2, 3}
        }
    );

    std::vector<index_t> labels =
    {
        0,
        1
    };

    auto facets = geo::BuildFacets(mesh, labels, 2);

    ASSERT_EQ(facets.size(), 2u);

    EXPECT_EQ(facets[0].triangleIndices.size(), 1u);
    EXPECT_EQ(facets[1].triangleIndices.size(), 1u);

    EXPECT_EQ(facets[0].triangleIndices[0], 0u);
    EXPECT_EQ(facets[1].triangleIndices[0], 1u);
}

TEST(BuildFacetsTest, InvalidTrianglesIgnored)
{
    geo::Mesh mesh(
        "quad",
        {
            {0.f, 0.f, 0.f},
            {1.f, 0.f, 0.f},
            {1.f, 1.f, 0.f},
            {0.f, 1.f, 0.f}
        },
        {
            {0, 1, 2},
            {0, 2, 3}
        }
    );

    std::vector<index_t> labels =
    {
        0,
        INVALID_INDEX
    };

    auto facets = geo::BuildFacets(mesh, labels, 1);

    ASSERT_EQ(facets.size(), 1u);

    EXPECT_EQ(facets[0].triangleIndices.size(), 1u);
    EXPECT_EQ(facets[0].triangleIndices[0], 0u);
}

TEST(BuildFacetsTest, AreaWeightedCentroidForQuad)
{
    geo::Mesh mesh(
        "quad",
        {
            {0.f, 0.f, 0.f},
            {1.f, 0.f, 0.f},
            {1.f, 1.f, 0.f},
            {0.f, 1.f, 0.f}
        },
        {
            {0, 1, 2},
            {0, 2, 3}
        }
    );

    std::vector<index_t> labels =
    {
        0,
        0
    };

    auto facets = geo::BuildFacets(mesh, labels, 1);

    ASSERT_EQ(facets.size(), 1u);

    EXPECT_NEAR(facets[0].centroid.x, 0.5f, 1e-5f);
    EXPECT_NEAR(facets[0].centroid.y, 0.5f, 1e-5f);
    EXPECT_NEAR(facets[0].centroid.z, 0.0f, 1e-5f);
}
