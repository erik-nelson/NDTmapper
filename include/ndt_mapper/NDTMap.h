#ifndef __NDTMAP_H__
#define __NDTMAP_H__

#include <ndt_mapper/GaussianVoxel.h>
#include <ndt_mapper/PointCloud.h>
#include <geometry_utils/GeometryUtilsROS.h>

namespace ndtm
{

  class NDTMap
  {

  public:

    typedef struct
    {
      double prob_occ;
      double prob_emp;
      double resolution;
      double sizex;
      double sizey;
      double sizez;
      unsigned int cellsx;
      unsigned int cellsy;
      unsigned int cellsz;
    } map_params_t;

    explicit NDTMap();
    ~NDTMap();

    bool initialize(const map_params_t& map_params_);

    // Overload () for x,y,z access into 1D map array of GaussianVoxel ptrs
    // 1D array is far faster than 3D array
    GaussianVoxel::ConstPtr operator() (const unsigned int& x,
                                        const unsigned int& y,
                                        const unsigned int& z);

    // Overload [] for 1D access into map array of GaussianVoxel ptrs
    GaussianVoxel::ConstPtr operator[] (const unsigned int& idx);

  private:

    bool initialized;
    map_params_t map_params;

    // The map is a long 1D array of dynamically
    // allocated GaussianVoxel pointers
    typedef std::vector<GaussianVoxel::Ptr> map_t;
    map_t map;

  }; //\class NDTMap

} //\namespace ndtm

#endif
