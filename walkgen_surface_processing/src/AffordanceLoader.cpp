#include "AffordanceLoader.hpp"
#include <iostream>

AffordanceLoader::AffordanceLoader() {
}

void AffordanceLoader::load(std::string const& filename, MatrixN const& R, VectorN const& T) {
  hpp::fcl::MeshLoader loader;

  typedef std::shared_ptr<hpp::fcl::BVHModelBase> BVHModelBase_Ptr_t;
  BVHModelBase_Ptr_t bvh_mdel = loader.load(filename);

  fcl::CollisionObject* obj(new fcl::CollisionObject(bvh_mdel, R, T));

  hpp::affordance::SupportOperationPtr_t support(new hpp::affordance::SupportOperation());
  std::vector<hpp::affordance::OperationBasePtr_t> operations;
  operations.push_back(support);
  hpp::affordance::SemanticsDataPtr_t h = hpp::affordance::affordanceAnalysis(obj, operations);

  std::vector<uint>
      indexAdded;  // Index of the vertex added to the affordance, avoid adding multiple times the same vertex.
  for (int i = 0; i < (int)h->affordances_[0].size(); i++) {
    hpp::affordance::AffordancePtr_t affordance = h->affordances_[0][i];
    std::vector<Vector3> affordance_vertices;
    indexAdded.clear();

    for (uint j = 0; j < affordance->indices_.size(); j++) {
      uint tri_index = affordance->indices_[j];                   // Index of the triangle
      fcl::Triangle triangle = bvh_mdel->tri_indices[tri_index];  // vector of vertices indexes
      for (int k = 0; k < 3; k++) {
        uint vertex_index = (uint)triangle[k];
        if (std::find(indexAdded.begin(), indexAdded.end(), vertex_index) == indexAdded.end()) {
          // Move according to Rotation and translation
          affordance_vertices.push_back(obj->getRotation() * bvh_mdel->vertices[vertex_index] + obj->getTranslation());
          indexAdded.push_back(vertex_index);
        }
      }
    }
    affordances_.push_back(affordance_vertices);
  }
}
