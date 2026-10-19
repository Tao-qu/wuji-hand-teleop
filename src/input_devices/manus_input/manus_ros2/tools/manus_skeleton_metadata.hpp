#pragma once

#include "ManusSDKTypes.h"
#include <map>
#include <vector>

namespace manus_tools {
using NodeMetadata = std::map<uint32_t, NodeInfo>;

inline NodeMetadata index_metadata(const std::vector<NodeInfo>& nodes) {
    NodeMetadata result;
    for (const auto& node : nodes) {
        result[node.nodeId] = node;
    }
    return result;
}
} // namespace manus_tools
