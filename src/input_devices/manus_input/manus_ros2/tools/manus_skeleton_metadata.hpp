#pragma once

#include "ManusSDKTypes.h"
#include <map>
#include <set>
#include <stdexcept>
#include <vector>

namespace manus_tools {
using NodeMetadata = std::map<uint32_t, NodeInfo>;

inline NodeMetadata index_metadata(const std::vector<NodeInfo>& nodes) {
    NodeMetadata result;
    for (const auto& node : nodes) {
        if (!result.emplace(node.nodeId, node).second) {
            throw std::runtime_error("Duplicate MANUS hierarchy node ID");
        }
    }
    if (result.empty()) throw std::runtime_error("Empty MANUS hierarchy");
    return result;
}

template<class RawNodes>
bool matches_raw_nodes(const NodeMetadata& metadata, const RawNodes& nodes) {
    if (metadata.size() != nodes.size()) return false;
    std::set<uint32_t> seen;
    for (const auto& node : nodes) {
        if (!seen.insert(node.id).second || metadata.find(node.id) == metadata.end()) {
            return false;
        }
    }
    return true;
}
} // namespace manus_tools
