#include <glm/glm.hpp>
#include <memory>
#include <stdexcept>
#include <vector>

#include "octree.h"
#include "raytracer.h"
#include "test_support.h"

int main() {
    voxel4d::test::TestContext test;

    test.expect_throws<std::invalid_argument>(
        [] { static_cast<void>(SparseVoxelOctree(glm::vec3(0.0F), 0.0F, 1)); },
        "Octree must reject a zero root size");
    test.expect_throws<std::invalid_argument>(
        [] { static_cast<void>(SparseVoxelOctree(glm::vec3(0.0F), 1.0F, -1)); },
        "Octree must reject a negative maximum depth");

    const auto octree = std::make_shared<SparseVoxelOctree>(glm::vec3(0.0F), 2.0F, 2);
    test.expect(octree->contains(glm::vec3(1.0F, -1.0F, 1.0F)),
                "Octree bounds must include root boundary positions");
    test.expect(!octree->contains(glm::vec3(1.01F, 0.0F, 0.0F)),
                "Octree bounds must reject positions outside the root cube");
    test.expect(octree->search(glm::vec3(1.01F, 0.0F, 0.0F)) == nullptr,
                "Out-of-bounds search must return nullptr");

    VoxelAttribute occupied{};
    occupied.color = glm::vec3(0.25F, 0.5F, 0.75F);
    occupied.density = 1.0F;
    occupied.semantic_label = 42;
    const glm::vec3 occupied_position(0.25F, 0.25F, 0.25F);
    test.expect(octree->insert(occupied_position, occupied),
                "In-bounds voxel insertion must succeed");
    test.expect(!octree->insert(glm::vec3(1.1F, 0.0F, 0.0F), occupied),
                "Out-of-bounds voxel insertion must fail");

    const auto inserted_leaf = octree->search(occupied_position);
    test.expect(inserted_leaf != nullptr, "Inserted voxel must be searchable");
    if (inserted_leaf) {
        test.expect_near(inserted_leaf->attribute.density, 1.0F, 1.0e-6F,
                         "Inserted density must be retained");
        test.expect(inserted_leaf->attribute.semantic_label == 42,
                    "Inserted semantic label must be retained");
    }

    VoxelAttribute updated = occupied;
    updated.density = 0.5F;
    updated.semantic_label = 7;
    test.expect(octree->update(occupied_position, updated), "In-bounds voxel update must succeed");
    test.expect(!octree->update(glm::vec3(-1.1F, 0.0F, 0.0F), updated),
                "Out-of-bounds voxel update must fail");
    const auto updated_leaf = octree->search(occupied_position);
    test.expect(updated_leaf != nullptr, "Updated voxel must remain searchable");
    if (updated_leaf) {
        test.expect_near(updated_leaf->attribute.density, 0.5F, 1.0e-6F,
                         "Updated density must be retained");
        test.expect(updated_leaf->attribute.semantic_label == 7,
                    "Updated semantic label must be retained");
    }

    test.expect_throws<std::invalid_argument>([] { static_cast<void>(VoxelRaytracer(nullptr)); },
                                              "Ray tracer must reject a null Octree");

    VoxelRaytracer raytracer(octree);
    const Ray hit_ray{glm::vec3(-2.0F, 0.25F, 0.25F), glm::vec3(1.0F, 0.0F, 0.0F)};
    const RayHitResult hit = raytracer.trace_ray(hit_ray, 10.0F);
    test.expect(hit.hit, "DDA must report the occupied leaf intersected by the ray");
    test.expect(hit.voxel == updated_leaf, "DDA must return the matching occupied Octree leaf");
    test.expect_near(hit.distance, 2.0F, 1.0e-4F,
                     "DDA hit distance must match the entered occupied leaf boundary");
    test.expect(!raytracer.trace_ray(hit_ray, 1.5F).hit,
                "DDA must honor a maximum distance before the occupied leaf");
    test.expect(!raytracer.trace_ray(Ray{}, 10.0F).hit,
                "DDA must reject a zero-length ray direction without a hit");

    const std::vector<RayHitResult> batch = raytracer.trace_rays(
        {hit_ray, Ray{glm::vec3(-2.0F, -0.75F, -0.75F), glm::vec3(1.0F, 0.0F, 0.0F)}});
    test.expect(batch.size() == 2U, "Batch tracing must return one result for each ray");
    test.expect(batch.at(0).hit && !batch.at(1).hit,
                "Batch tracing must preserve independent hit results");

    return test.failures() == 0 ? 0 : 1;
}
