#ifndef __NDTMAPPER_H__
#define __NDTMAPPER_H__

#include <ros/ros.h>
#include <parameter_utils/ParameterUtils.h>
#include <ndt_mapper/NDTMap.h>

class NDTMapper
{

public:

  explicit NDTMapper();
  ~NDTMapper();

  bool initialize(const ros::NodeHandle& n);

private:
  bool loadParameters(const ros::NodeHandle& n);
  bool registerCallbacks(const ros::NodeHandle& n);

  std::string name;

  ndtm::NDTMap ndtmap;
  ndtm::NDTMap::map_params_t ndtmap_params;

};

#endif
