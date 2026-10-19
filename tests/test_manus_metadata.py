"""Compile the MANUS metadata helper against SDK types, without loading the SDK."""

from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
MANUS = ROOT / "src/input_devices/manus_input/manus_ros2"

CPP_SOURCE = r"""
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>
#include "manus_skeleton_metadata.hpp"

void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

NodeInfo metadata_node(uint32_t id, uint32_t parent) {
    NodeInfo node{};
    node.nodeId = id;
    node.parentId = parent;
    return node;
}

std::vector<SkeletonNode> raw_nodes(std::initializer_list<uint32_t> ids) {
    std::vector<SkeletonNode> result;
    for (const auto id : ids) {
        SkeletonNode node{};
        node.id = id;
        result.push_back(node);
    }
    return result;
}

manus_tools::NodeMetadata sparse_metadata() {
    // SDK IDs are identifiers, not offsets into this three-element vector.
    return manus_tools::index_metadata({
        metadata_node(42, 1),
        metadata_node(4096, 42),
        metadata_node(1, 0),
    });
}

void sparse_unordered() {
    const auto metadata = sparse_metadata();
    const auto raw = raw_nodes({4096, 1, 42});
    require(manus_tools::matches_raw_nodes(metadata, raw),
            "Sparse IDs must match regardless of hierarchy/raw order");
    const uint32_t expected_parents[] = {42, 0, 1};
    for (std::size_t i = 0; i < raw.size(); ++i) {
        const auto& info = metadata.at(raw[i].id);
        require(info.nodeId == raw[i].id, "ID lookup returned the wrong node");
        require(info.parentId == expected_parents[i],
                "ID lookup must preserve the corresponding metadata");
    }
}

void one_based_unordered() {
    const auto metadata = manus_tools::index_metadata({
        metadata_node(3, 2), metadata_node(1, 0),
        metadata_node(4, 3), metadata_node(2, 1),
    });
    const auto raw = raw_nodes({2, 4, 1, 3});
    require(manus_tools::matches_raw_nodes(metadata, raw),
            "One-based IDs must match regardless of order");
    for (const auto& node : raw) {
        const auto& info = metadata.at(node.id);
        require(info.nodeId == node.id && info.parentId == node.id - 1,
                "One-based IDs must not be used as vector offsets");
    }
}

void duplicate_metadata() {
    bool rejected = false;
    try {
        (void)manus_tools::index_metadata({
            metadata_node(42, 1), metadata_node(1, 0), metadata_node(42, 99),
        });
    } catch (const std::runtime_error&) {
        rejected = true;
    }
    require(rejected, "Duplicate hierarchy IDs must be rejected");
}

void empty_metadata() {
    bool rejected = false;
    try {
        (void)manus_tools::index_metadata({});
    } catch (const std::runtime_error&) {
        rejected = true;
    }
    require(rejected, "Empty hierarchy must be rejected");
}

void missing_raw() {
    const auto metadata = sparse_metadata();
    require(!manus_tools::matches_raw_nodes(metadata, raw_nodes({1, 42})),
            "Missing raw node must be rejected");
    require(!manus_tools::matches_raw_nodes(metadata, raw_nodes({})),
            "Empty raw frame must be rejected");
}

void unknown_raw() {
    const auto metadata = sparse_metadata();
    require(!manus_tools::matches_raw_nodes(metadata, raw_nodes({1, 42, 999})),
            "Equal node counts must not hide an unknown/missing ID");
    require(!manus_tools::matches_raw_nodes(metadata, raw_nodes({1, 42, 4096, 999})),
            "Extra raw node must be rejected");
}

void duplicate_raw() {
    const auto metadata = sparse_metadata();
    require(!manus_tools::matches_raw_nodes(metadata, raw_nodes({1, 42, 42})),
            "Equal node counts must not hide a duplicate/missing raw ID");
}

int main(int argc, char** argv) {
    try {
        require(argc == 2, "Expected one test case name");
        const std::string test = argv[1];
        if (test == "sparse_unordered") sparse_unordered();
        else if (test == "one_based_unordered") one_based_unordered();
        else if (test == "duplicate_metadata") duplicate_metadata();
        else if (test == "empty_metadata") empty_metadata();
        else if (test == "missing_raw") missing_raw();
        else if (test == "unknown_raw") unknown_raw();
        else if (test == "duplicate_raw") duplicate_raw();
        else throw std::runtime_error("Unknown test case");
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
"""


class ManusMetadataTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        compiler = shutil.which("g++")
        if compiler is None:
            raise unittest.SkipTest("g++ is required for the MANUS metadata regression tests")
        directory = tempfile.TemporaryDirectory(prefix="manus-metadata-test-")
        cls.addClassCleanup(directory.cleanup)
        source = Path(directory.name) / "metadata_test.cpp"
        cls.binary = Path(directory.name) / "metadata_test"
        source.write_text(CPP_SOURCE, encoding="utf-8")
        result = subprocess.run(
            [
                compiler, "-std=c++17", "-Wall", "-Wextra", "-Werror",
                "-D_GLIBCXX_ASSERTIONS", "-O0", "-g",
                "-I", str(MANUS / "tools"),
                "-I", str(MANUS / "ManusSDK/include"),
                str(source), "-o", str(cls.binary),
            ],
            capture_output=True, text=True, timeout=30,
        )
        if result.returncode:
            raise AssertionError(f"Metadata test compilation failed:\n{result.stdout}{result.stderr}")

    def run_case(self, name):
        result = subprocess.run(
            [str(self.binary), name], capture_output=True, text=True, timeout=5,
        )
        self.assertEqual(result.returncode, 0, f"{name}: {result.stdout}{result.stderr}")

    def test_sparse_ids_map_metadata_in_raw_order(self):
        self.run_case("sparse_unordered")

    def test_one_based_ids_map_metadata_in_raw_order(self):
        self.run_case("one_based_unordered")

    def test_duplicate_metadata_is_rejected(self):
        self.run_case("duplicate_metadata")

    def test_empty_metadata_is_rejected(self):
        self.run_case("empty_metadata")

    def test_missing_raw_nodes_are_rejected(self):
        self.run_case("missing_raw")

    def test_unknown_raw_nodes_are_rejected(self):
        self.run_case("unknown_raw")

    def test_duplicate_raw_nodes_are_rejected(self):
        self.run_case("duplicate_raw")


if __name__ == "__main__":
    unittest.main()
