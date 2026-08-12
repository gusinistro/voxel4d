#include "octree.h"

#include <stdexcept>

SparseVoxelOctree::SparseVoxelOctree(const glm::vec3& root_center, float root_size, int max_depth)
    : max_depth_(max_depth) {
    if (root_size <= 0.0F) {
        throw std::invalid_argument("root_size must be greater than zero");
    }
    if (max_depth < 0) {
        throw std::invalid_argument("max_depth must be non-negative");
    }

    root_ = std::make_shared<OctreeNode>(root_center, root_size, 0);
}

int SparseVoxelOctree::get_child_index(const glm::vec3& position, const glm::vec3& center) {
    int index = 0;
    if (position.x >= center.x) {
        index |= 1;
    }
    if (position.y >= center.y) {
        index |= 2;
    }
    if (position.z >= center.z) {
        index |= 4;
    }
    return index;
}

bool SparseVoxelOctree::contains(const glm::vec3& position) const {
    const glm::vec3 half_extent(root_->size * 0.5F);
    const glm::vec3 min_bound = root_->center - half_extent;
    const glm::vec3 max_bound = root_->center + half_extent;

    return position.x >= min_bound.x && position.x <= max_bound.x && position.y >= min_bound.y &&
           position.y <= max_bound.y && position.z >= min_bound.z && position.z <= max_bound.z;
}

void SparseVoxelOctree::subdivide(const std::shared_ptr<OctreeNode>& node) {
    if (!node || !node->is_leaf) {
        return;
    }

    const float child_size = node->size * 0.5F;
    const float center_offset = child_size * 0.5F;

    node->is_leaf = false;
    node->children.resize(8);
    for (int index = 0; index < 8; ++index) {
        const glm::vec3 offset((index & 1) != 0 ? center_offset : -center_offset,
                               (index & 2) != 0 ? center_offset : -center_offset,
                               (index & 4) != 0 ? center_offset : -center_offset);

        node->children[static_cast<std::size_t>(index)] =
            std::make_shared<OctreeNode>(node->center + offset, child_size, node->depth + 1);
        ++node_count_;
    }

    // One leaf becomes eight leaves.
    leaf_count_ += 7;
}

bool SparseVoxelOctree::insert(const glm::vec3& position, const VoxelAttribute& attribute) {
    if (!contains(position)) {
        return false;
    }

    auto current = root_;
    for (int depth = 0; depth < max_depth_; ++depth) {
        if (current->is_leaf) {
            subdivide(current);
        }
        const int child_index = get_child_index(position, current->center);
        current = current->children[static_cast<std::size_t>(child_index)];
    }

    current->attribute = attribute;
    return true;
}

std::shared_ptr<OctreeNode> SparseVoxelOctree::search_recursive(
    const std::shared_ptr<OctreeNode>& node, const glm::vec3& position) const {
    if (!node || node->is_leaf) {
        return node;
    }

    const int child_index = get_child_index(position, node->center);
    return search_recursive(node->children[static_cast<std::size_t>(child_index)], position);
}

std::shared_ptr<OctreeNode> SparseVoxelOctree::search(const glm::vec3& position) const {
    if (!contains(position)) {
        return nullptr;
    }
    return search_recursive(root_, position);
}

bool SparseVoxelOctree::update(const glm::vec3& position, const VoxelAttribute& attribute) {
    const auto node = search(position);
    if (!node) {
        return false;
    }

    node->attribute = attribute;
    return true;
}
