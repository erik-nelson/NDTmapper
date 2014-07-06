#ifndef __VOXEL_H__
#define __VOXEL_H__

#include <boost/shared_ptr.hpp>

namespace ndtm
{
  template <typename POINT>
  class Voxel
  {

  public:

    typedef boost::shared_ptr<Voxel> Ptr;
    typedef boost::shared_ptr<const Voxel> ConstPtr;

    explicit Voxel() :
      has_center(false),
      has_dims(false)
    {}

    explicit Voxel(const POINT& center_,
                   const double& sizex_,
                   const double& sizey_,
                   const double& sizez_) :
      center(center_),
      sizex(sizex_),
      sizey(sizey_),
      sizez(sizez_),
      has_center(true),
      has_dims(true)
    {}

    virtual ~Voxel()
    {}

    inline bool isInitialized() const
    {
      return has_center && has_dims;
    }

    POINT getCenter() const
    {
      return center;
    }

    inline void setCenter(const POINT& center_)
    {
      center = center_;
    }

    inline void getDimensions(double& x,
                              double& y,
                              double& z) const
    {
      x = sizex;
      y = sizey;
      z = sizez;
    }

    inline void setDimensions(const double& x,
                              const double& y,
                              const double& z)
    {
      sizex = x;
      sizey = y;
      sizez = z;
    }

    POINT getLowerBackLeft() const
    {
      POINT lowerBackLeft(center.at(0) - sizex / 2.0,
                          center.at(1) - sizey / 2.0,
                          center.at(2) - sizez / 2.0);
      return lowerBackLeft;
    }

    POINT getUpperFrontRight() const
    {
      POINT upperFrontRight(center.at(0) - sizex / 2.0,
                            center.at(1) - sizey / 2.0,
                            center.at(2) - sizez / 2.0);
      return upperFrontRight;
    }

    bool isInside(const POINT& p) const
    {
      POINT min = getLowerBackLeft();
      POINT max = getUpperFrontRight();

      return p.at(0) > min.at(0)
        && p.at(1) > min.at(1)
        && p.at(2) > min.at(2)
        && p.at(0) < max.at(0)
        && p.at(1) < max.at(1)
        && p.at(2) < max.at(2);
    }

  protected:

    POINT center;
    double sizex, sizey, sizez;
    bool has_center, has_dims;

  }; //\class Voxel

} //\namespace ndtm
#endif
