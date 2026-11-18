#pragma once

#include <cstddef>

namespace community_tie_guidance {

// -1 denotes a mixed coarse node, never a community shared by mixed nodes.
inline bool samePureCommunity(int source_label, int target_label) {
    return source_label >= 0 && source_label == target_label;
}

inline int mergedPureLabel(int source_label, int target_label) {
    return samePureCommunity(source_label, target_label) ? source_label : -1;
}

// Consume resource-legal candidates in the ORIGINAL iteration order.
// No score computation, candidate generation, sorting, or graph mutation here.
class Choice {
public:
    explicit Choice(bool enabled) : enabled_(enabled) {}

    void consider(int target, float score, bool preferred) {
        // Shadow of the original "greater OR equal overwrites" rule.
        if (score > best_score_ || score == best_score_)
            original_target_ = target;

        if (score > best_score_) {
            best_score_ = score;
            selected_target_ = target;
            best_preferred_ = preferred;
            top_count_ = 1;
            preferred_top_count_ = preferred ? 1 : 0;
        } else if (score == best_score_) {
            ++top_count_;
            preferred_top_count_ += preferred ? 1 : 0;
            if (!enabled_ || preferred >= best_preferred_) {
                selected_target_ = target;
                best_preferred_ = preferred;
            }
        }
    }

    bool valid() const { return best_score_ > 0.0f; }
    float score() const { return best_score_; }
    int target() const { return valid() ? selected_target_ : -1; }
    int originalTarget() const { return valid() ? original_target_ : -1; }
    std::size_t topCount() const { return valid() ? top_count_ : 0; }
    std::size_t preferredTopCount() const {
        return valid() ? preferred_top_count_ : 0;
    }
    bool changed() const { return valid() && target() != originalTarget(); }

private:
    bool enabled_;
    float best_score_ = -1.0f;
    int selected_target_ = -1;
    int original_target_ = -1;
    bool best_preferred_ = false;
    std::size_t top_count_ = 0;
    std::size_t preferred_top_count_ = 0;
};

} // namespace community_tie_guidance
