#include <ndt_mapper/NDTMapper.h>

namespace pu = parameter_utils;

NDTMapper::NDTMapper()
{}

NDTMapper::~NDTMapper()
{}

bool NDTMapper::initialize(const ros::NodeHandle& n)
{
  name = ros::names::append(n.getNamespace(), "NDTMapper");

  if (!loadParameters(n))
  {
    ROS_ERROR("%s: failed to load parameters", name.c_str());
    return false;
  }

  if (!registerCallbacks(n))
  {
    ROS_ERROR("%s: failed to register callbacks", name.c_str());
    return false;
  }

  // Initialize the NDTMap class object with the acquired parameters
  ndtmap.initialize(ndtmap_params);

  return true;
}

bool NDTMapper::loadParameters(const ros::NodeHandle& n)
{

  if (!pu::get("ndt_mapper/prob_occ", ndtmap_params.prob_occ)) return false;
  if (!pu::get("ndt_mapper/prob_emp", ndtmap_params.prob_emp)) return false;
  if (!pu::get("ndt_mapper/resolution", ndtmap_params.resolution)) return false;
  if (!pu::get("ndt_mapper/sizex", ndtmap_params.sizex)) return false;
  if (!pu::get("ndt_mapper/sizey", ndtmap_params.sizey)) return false;
  if (!pu::get("ndt_mapper/sizez", ndtmap_params.sizez)) return false;

  ndtmap_params.cellsx =
    static_cast<unsigned int>(ndtmap_params.sizex / ndtmap_params.resolution + 1);
  ndtmap_params.cellsy =
    static_cast<unsigned int>(ndtmap_params.sizey / ndtmap_params.resolution + 1);
  ndtmap_params.cellsz =
    static_cast<unsigned int>(ndtmap_params.sizez / ndtmap_params.resolution + 1);

  return true;
}

bool NDTMapper::registerCallbacks(const ros::NodeHandle& n)
{
  ros::NodeHandle nl(n, "ndt_mapper");
  return true;
}
