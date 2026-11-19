#pragma once
// Included after the original metric functions and VertexPairRating declaration.
#include <cstdlib>
#include <map>
#include <chrono>
#include <atomic>
#include <cstdint>
#include <sstream>
#include <set>
#include "../community/detector_bridge.h"

namespace community_search {
inline std::vector<int> initial_labels;
inline bool active = false;
inline bool verify = false;
inline int delay_levels = 0;
inline std::size_t current_level = 0;
inline bool coarse_detection = false;
inline bool labels_ready = false;

inline void initialize(const Hypergraph& graph) {
    const char* mode = std::getenv("REPART_SEARCH_MODE");
    active = mode && (std::string(mode) == "H" || std::string(mode) == "SAME" || std::string(mode) == "R");
    verify = std::getenv("REPART_SEARCH_VERIFY") != nullptr;
    delay_levels = 0;current_level = 0;
    initial_labels.clear();
    coarse_detection = active && std::string(mode) == "R";
    labels_ready = false;
    if (!active) return;
    if(const char* value=std::getenv("REPART_SEARCH_DELAY_LEVELS")) {
        std::size_t used=0;long long parsed=std::stoll(value,&used);
        if(used!=std::string(value).size()||parsed<0||parsed>std::numeric_limits<int>::max())
            throw std::invalid_argument("H_INVALID_DELAY_LEVELS");
        delay_levels=static_cast<int>(parsed);
    }
    if(coarse_detection) {
        initial_labels.assign(graph.hypernodes.size(),-1);
        return;
    }
    const auto start = std::chrono::steady_clock::now();
    if (std::string(mode) == "SAME") initial_labels.assign(graph.hypernodes.size(), 0);
    else initial_labels = cf_bridge::detect(graph).node_labels;
    labels_ready = true;
#ifdef REPART_CF_OBSERVE
    cf_observe::record("h_detection", mode, initial_labels.size(),
        std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count());
    std::uint64_t checksum=1469598103934665603ULL;
    for(int label:initial_labels) {checksum^=static_cast<std::uint64_t>(label+1);checksum*=1099511628211ULL;}
    cf_observe::record("h_schedule",delay_levels,checksum);
#endif
}

// Detect on the first eligible bound coarse graph; lift its labels to all
// original members so the unchanged H purity/index logic can be reused.
inline void prepareCoarseLabels(const Hypergraph& graph) {
    if(!coarse_detection || labels_ready || current_level<static_cast<std::size_t>(delay_levels))return;
    const auto start=std::chrono::steady_clock::now();
    const auto detected=cf_bridge::detect(graph);
    std::vector<int> lifted(initial_labels.size(),-1);
    std::set<int> communities;
    std::size_t nodes=0,edges=0;
    for(const auto& node:graph.hypernodes)if(!node.deleted) {
        ++nodes;
        const int label=detected.node_labels.at(node.id);
        if(label<0)throw std::logic_error("R_ACTIVE_NODE_WITHOUT_LABEL");
        communities.insert(label);
        const auto assign=[&](int original) {
            if(lifted.at(original)!=-1)throw std::logic_error("R_DUPLICATE_ORIGINAL_MEMBER");
            lifted[original]=label;
#ifdef REPART_CF_OBSERVE
            cf_observe::record("r_member",original,node.id,label);
#endif
        };
        if(node.vertex_ids.empty())assign(node.id);
        else for(int original:node.vertex_ids)assign(original);
        if(lifted.at(node.id)!=label)throw std::logic_error("R_REPRESENTATIVE_NOT_MEMBER");
    }
    for(int label:lifted)if(label<0)throw std::logic_error("R_MISSING_ORIGINAL_MEMBER");
    for(const auto& edge:graph.hyperedges)if(!edge.deleted) {
        ++edges;
#ifdef REPART_CF_OBSERVE
        std::ostringstream pins;
        for(int p=edge._begin;p<edge._begin+edge._size;++p){if(p!=edge._begin)pins<<',';pins<<graph._incidence_array[p];}
        cf_observe::record("r_net",edge.id,edge.weight,pins.str());
#endif
    }
    initial_labels=std::move(lifted);labels_ready=true;
#ifdef REPART_CF_OBSERVE
    std::uint64_t checksum=1469598103934665603ULL;
    for(int label:initial_labels){checksum^=static_cast<std::uint64_t>(label+1);checksum*=1099511628211ULL;}
    cf_observe::record("r_detection",current_level,nodes,edges,communities.size());
    cf_observe::record("h_detection","R",initial_labels.size(),
        std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count());
    cf_observe::record("h_schedule",delay_levels,checksum);
#endif
}

class Batch;
inline Batch* current = nullptr;

class Batch {
    Hypergraph& graph_;
    const FPGAManager& manager_;
    std::vector<int> labels_;
    std::vector<std::unordered_map<int, std::vector<int>>> index_;
    std::vector<int> affected_;
    bool enabled_ = false;
    mutable std::atomic<std::size_t> calls_{0},hits_{0},mixed_fallback_{0},no_positive_fallback_{0};

    std::vector<std::size_t> purity() const {
        std::size_t pure=0,mixed=0,pure_mass=0,mass=0;
        for(const auto& node:graph_.hypernodes)if(!node.deleted) {
            int label=initial_labels.at(node.id);
            for(int original:node.vertex_ids)if(initial_labels.at(original)!=label){label=-1;break;}
            std::size_t count=node.vertex_ids.empty()?1:node.vertex_ids.size();mass+=count;
            if(label>=0){++pure;pure_mass+=count;}else ++mixed;
        }
        return {pure,mixed,pure_mass,mass};
    }

    void rebuild(int e) {
        auto& groups = index_.at(e);
        groups.clear();
        const auto& edge = graph_.hyperedges[e];
        if (edge.deleted || edge._size > edgeSizeThreshold(graph_)) return;
        for (int p = edge._begin; p < edge._begin + edge._size; ++p) {
            const int v = graph_._incidence_array[p];
            if (labels_[v] >= 0) groups[labels_[v]].push_back(v);
        }
    }

public:
    explicit Batch(Hypergraph& graph, const FPGAManager& manager)
        : graph_(graph), manager_(manager) {
        if (!active) return;
        if (current) throw std::logic_error("H_NESTED_BATCH");
        prepareCoarseLabels(graph);
        labels_.assign(graph.hypernodes.size(), -1);
        for (const auto& node : graph.hypernodes) {
            if (node.deleted) continue;
            int label = initial_labels.at(node.id);
            for (int original : node.vertex_ids)
                if (initial_labels.at(original) != label) { label = -1; break; }
            labels_[node.id] = label;
        }
        enabled_=current_level>=static_cast<std::size_t>(delay_levels);
#ifdef REPART_CF_OBSERVE
        const auto p=purity();
        cf_observe::record("h_batch_begin",current_level,delay_levels,enabled_,p[0],p[1],p[2],p[3]);
#endif
        if(!enabled_)return;
        index_.resize(graph.hyperedges.size());
        for (const auto& edge : graph.hyperedges) rebuild(edge.id);
        current = this;
    }
    ~Batch() {
        if (current == this) current = nullptr;
#ifdef REPART_CF_OBSERVE
        if(active){const auto p=purity();cf_observe::record("h_batch_end",current_level,delay_levels,enabled_,p[0],p[1],p[2],p[3],
            calls_.load(),hits_.load(),mixed_fallback_.load(),no_positive_fallback_.load());}
#endif
    }
    Batch(const Batch&) = delete;
    Batch& operator=(const Batch&) = delete;

    // False delegates to the unmodified original complete-search implementation.
    bool search(Hypergraph& graph, int u, VertexPairRating& result) const {
        ++calls_;
        if (&graph != &graph_ || labels_[u] < 0) {++mixed_fallback_;return false;}
        std::unordered_map<int, float> ratings;
        const auto scan = [&](int e) {
            const auto& edge = graph_.hyperedges[e];
            if (edge._size > edgeSizeThreshold(graph_)) return;
            const float score = heavyEdgeScore(graph_, e);
            const auto found = index_[e].find(labels_[u]);
            if (found == index_[e].end()) return;
            for (int v : found->second) if (v != u) ratings[v] += score;
        };
        for (int e : graph_.hypernodes[u].incident_nets_as_source) scan(e);
        for (int e : graph_.hypernodes[u].incident_nets_as_target) scan(e);
        if (verify) {
            std::unordered_map<int, float> expected;
            const auto check_net = [&](int e) {
                const auto& edge = graph_.hyperedges[e];
                if (edge._size > edgeSizeThreshold(graph_)) return;
                const float score = heavyEdgeScore(graph_, e);
                for (int p = edge._begin; p < edge._begin + edge._size; ++p) {
                    int v = graph_._incidence_array[p];
                    if (v != u && labels_[v] == labels_[u]) expected[v] += score;
                }
            };
            for (int e : graph_.hypernodes[u].incident_nets_as_source) check_net(e);
            for (int e : graph_.hypernodes[u].incident_nets_as_target) check_net(e);
            if (ratings != expected) throw std::logic_error("H_INDEX_ORACLE_MISMATCH");
        }
        float best = -1.0f;
        int target = -1;
        for (const auto& item : ratings) {
            if (!belowMinWeightFPGA(graph_, u, item.first, manager_)) continue;
            const float penalty = heavyNodePenalty(graph_, u, item.first, manager_);
            const float score = item.second / penalty;
            if (score > best || score == best) { best = score; target = item.first; }
        }
        if (!(best > 0.0f)) {++no_positive_fallback_;return false;}
        ++hits_;
        result = VertexPairRating(target, best);
        return true;
    }

    void beforeContract(int u, int v) {
        if (!enabled_) return;
        affected_.clear();
        const auto add = [&](int node) {
            const auto& hn = graph_.hypernodes[node];
            affected_.insert(affected_.end(), hn.incident_nets_as_source.begin(), hn.incident_nets_as_source.end());
            affected_.insert(affected_.end(), hn.incident_nets_as_target.begin(), hn.incident_nets_as_target.end());
        };
        add(u); add(v);
        std::sort(affected_.begin(), affected_.end());
        affected_.erase(std::unique(affected_.begin(), affected_.end()), affected_.end());
    }
    void afterContract(int u, int v) {
        if (!enabled_) return;
        if (labels_[u] < 0 || labels_[u] != labels_[v]) labels_[u] = -1;
        labels_[v] = -1;
        for (int e : affected_) rebuild(e);
        if (verify) {
            int expected = initial_labels.at(u);
            for (int original : graph_.hypernodes[u].vertex_ids)
                if (initial_labels.at(original) != expected) { expected = -1; break; }
            if (expected != labels_[u]) throw std::logic_error("H_LABEL_ORACLE_MISMATCH");
        }
    }
};
} // namespace community_search
