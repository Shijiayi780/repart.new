#pragma once
#include <vector>
#include <stdexcept>

// One record per aggregated, resource-legal cross-community candidate.
// Buffers belong to individual rating workers; no shared logging lock.
struct CommunityRejection {
    int u, v, source_label, target_label;
    float original_score;
    const char* reason = "CROSS_COMMUNITY";
};

struct CommunityRatingCall {
    int active_nodes;
    int source;
    const char* phase;
    std::vector<CommunityRejection> rejected;
};

struct CommunityRatingCounts {
    std::size_t calls = 0;
    std::size_t cross_candidates = 0;
    std::size_t positive_cross_candidates = 0;
};
