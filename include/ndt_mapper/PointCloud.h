#ifndef __POINTCLOUD_H__
#define __POINTCLOUD_H__

#include <boost/shared_ptr.hpp>

namespace ndtm
{

  template <typename POINT>
  class PointCloud
  {

  public:

    // Typedefs
    typedef std::vector<POINT> point_list_t;
    typedef typename PointCloud::point_list_t::iterator iterator;
    typedef typename PointCloud::point_list_t::const_iterator const_iterator;
    typedef boost::shared_ptr<PointCloud> Ptr;
    typedef boost::shared_ptr<const PointCloud> ConstPtr;

    // Ctors, Dtors
    explicit PointCloud();
    explicit PointCloud(const point_list_t& in) : points(in) {}
    explicit PointCloud(const PointCloud& in) : points(in.getPoints()) {}
    explicit PointCloud(const PointCloud::Ptr in) : points(in->getPoints()) {}
    explicit PointCloud(const PointCloud::ConstPtr in) : points(in->getPoints()) {}
    ~PointCloud() { clear(); }

    // Iterator access
    iterator begin() { return points.begin(); }
    iterator end() { return points.end(); }
    const_iterator begin() const { return points.begin(); }
    const_iterator end() const { return points.end(); }
    POINT front() { return points.front(); }
    POINT back() { return points.back(); }

    // Duplicate std::vector behavior on point cloud typedef
    size_t size() const { return points.size(); }
    void clear() { points.clear(); }
    inline void reserve(size_t size) { points.reserve(size); }

    inline void push_back(const double& x,
                          const double& y,
                          const double& z)
    {
      points.push_back( POINT(x, y, z) );
    }

    inline void push_back(const POINT& p)
    {
      points.push_back(p);
    }

    inline void push_back(const POINT* p)
    {
      points.push_back(*p);
    }

    void push_back(const PointCloud<POINT>& pcld)
    {
      points.insert(points.end(), pcld.begin(), pcld.end());
    }

    void push_back(const PointCloud<POINT>::Ptr& pcld)
    {
      push_back(*pcld);
    }

    void push_back(const PointCloud<POINT>::ConstPtr& pcld)
    {
      push_back(*pcld);
    }

    // Point cloud operations
    POINT getPoint(const unsigned int& i)
    {
      return points[i];
    }

    point_list_t getPoints() const
    {
      return points;
    }

    void getBBX(POINT& min, POINT& max) const
    {
      //TODO
      printf("Not currently implemented\n");
    }

    void cullToBBX(POINT& min, POINT& max)
    {
      //TODO
      printf("Not currently implemented\n");
    }

    void subsample(const unsigned int& num)
    {
      //TODO
#if DEBUG
      assert(num <= points.size() && num >= 0);
#endif
      printf("Not currently implemented\n");
    }

    void subsample(const double& ratio)
    {
      //TODO
#if DEBUG
      assert(ratio > 0.0 && ratio < 1.0);
#endif
      printf("Not currently implemented\n");
    }

    void rotate(const double& roll,
                const double& pitch,
                const double& yaw)
    {
      //TODO
      printf("Not currently implemented\n");
    }

    void print(const std::string& prefix) const
    {
      //TODO
      printf("Not currently implemented\n");
    }

  private:

    point_list_t points;

  }; //\class PointCloud

} //\namespace ndtm
#endif
