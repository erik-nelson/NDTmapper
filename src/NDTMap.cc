#include <ndt_mapper/NDTMap.h>

namespace gu = geometry_utils;
namespace gr = gu::ros;

ndtm::NDTMap::NDTMap() :
  initialized(false)
{}

ndtm::NDTMap::~NDTMap()
{}

bool ndtm::NDTMap::initialize(const map_params_t& map_params_)
{
  // Store the parameters locally
  map_params = map_params_;

  // Allocate and initialize unknown cells in the map
  for (double x = 0; x < static_cast<unsigned int>(map_params.cellsx); ++x)
    for (double y = 0; y < static_cast<unsigned int>(map_params.cellsy); ++y)
      for (double z = 0; z < static_cast<unsigned int>(map_params.cellsz); ++z)
      {
        gu::Vector3 center = gu::toVector3(x, y, z);

        GaussianVoxel::Ptr voxel_ptr =
          GaussianVoxel::Ptr(new GaussianVoxel(center,
                                               map_params.resolution,
                                               map_params.resolution,
                                               map_params.resolution,
                                               map_params.prob_occ,
                                               map_params.prob_emp));
        map.push_back(voxel_ptr);
      }


  initialized = true;
  return initialized;
}

#if 0
void ndtm::NDTMap::reset()
{
}

void ndtm::NDTMap::insertPointCloud(const sensor_msgs::PointCloud& pcld
#endif
