#pragma once

#include <cstddef>
#include <cstdint>
#include <glm/glm.hpp>
#include <memory>
#include <vector>

#include "time_types.h"

/**
 * @brief Per-voxel attributes used by the proof of concept.
 *
 * A voxel is considered occupied when density is greater than zero.
 */
struct VoxelAttribute {
    glm::vec3 color{0.0F};
    float intensity{0.0F};
    float temperature{0.0F};
    glm::vec3 velocity{0.0F};
    float density{0.0F};
    float confidence{0.0F};
    std::uint32_t source_modality_mask{0U};
    voxel4d::TimestampNanoseconds last_observed_timestamp_nanoseconds{0};
    int semantic_label{0};
};

/** @brief A node in the sparse voxel octree. */
struct OctreeNode {
    glm::vec3 center;
    float size;
    int depth;
    bool is_leaf{true};
    VoxelAttribute attribute{};
    std::vector<std::shared_ptr<OctreeNode>> children;

    OctreeNode(const glm::vec3& node_center, float node_size, int node_depth)
        : center(node_center), size(node_size), depth(node_depth) {}
};

/**
 * @brief Hierarchical voxel storage for the Voxel4D proof of concept.
 *
 * Insertions create the necessary path to max_depth. Search returns the leaf
 * containing a position, which may be empty (density == 0).
 */
class SparseVoxelOctree {
   public:
    SparseVoxelOctree(const glm::vec3& root_center, float root_size, int max_depth);

    /** @return true when the value was inserted; false when it is out of bounds. */
    bool insert(const glm::vec3& position, const VoxelAttribute& attribute);

    /** @return the containing leaf, or nullptr for an out-of-bounds position. */
    std::shared_ptr<OctreeNode> search(const glm::vec3& position) const;

    /** @return true when an existing in-bounds leaf was updated. */
    bool update(const glm::vec3& position, const VoxelAttribute& attribute);

    [[nodiscard]] std::shared_ptr<OctreeNode> get_root() const {
        return root_;
    }
    [[nodiscard]] std::size_t get_node_count() const {
        return node_count_;
    }
    [[nodiscard]] std::size_t get_leaf_count() const {
        return leaf_count_;
    }
    [[nodiscard]] int get_max_depth() const {
        return max_depth_;
    }
    [[nodiscard]] bool contains(const glm::vec3& position) const;

   private:
    std::shared_ptr<OctreeNode> root_;
    int max_depth_;
    std::size_t node_count_{1};
    std::size_t leaf_count_{1};

    static int get_child_index(const glm::vec3& position, const glm::vec3& center);
    void subdivide(const std::shared_ptr<OctreeNode>& node);
    std::shared_ptr<OctreeNode> search_recursive(const std::shared_ptr<OctreeNode>& node,
                                                 const glm::vec3& position) const;
};
