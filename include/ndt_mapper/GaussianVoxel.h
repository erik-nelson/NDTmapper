#ifndef __GAUSSIANVOXEL_H__
#define __GAUSSIANVOXEL_H__

#include <ndt_mapper/Voxel.h>

#include <geometry_utils/GeometryUtils.h>
#include <list>

namespace gu = geometry_utils;

namespace ndtm
{

  class GaussianVoxel : public Voxel<gu::Vector3>
  {

  public:

    typedef boost::shared_ptr<GaussianVoxel> Ptr;
    typedef boost::shared_ptr<const GaussianVoxel> ConstPtr;

    struct params_t
    {
      gu::Vector3 mean;
      gu::Matrix3x3 cov;
      unsigned int n_obs;
      double log_odds_occ;

      params_t() {}
      params_t(const gu::Vector3& mean_,
               const gu::Matrix3x3& cov_,
               const unsigned int& n_obs_,
               const double& log_odds_occ_) :
        mean(mean_),
        cov(cov_),
        n_obs(n_obs_),
        log_odds_occ(log_odds_occ_) {}
    };

    typedef enum {OCCUPIED, EMPTY} obs_type_t;
    typedef std::pair<gu::Vector3, obs_type_t> observation_t;
    typedef std::list<observation_t> observation_list_t;

    explicit GaussianVoxel();

    explicit GaussianVoxel(const gu::Vector3& center_,
                           const double& sizex_,
                           const double& sizey_,
                           const double& sizez_,
                           const double& prob_occ_,
                           const double& prob_emp_);

    explicit GaussianVoxel(const gu::Vector3& center_,
                           const double& sizex_,
                           const double& sizey_,
                           const double& sizez_,
                           const double& prob_occ_,
                           const double& prob_emp_,
                           const params_t& params_);

    virtual ~GaussianVoxel();

    inline bool isInitialized();

    void setMean(const gu::Vector3& mean);
    void setCovariance(const gu::Matrix3x3& cov);
    inline void setNumPoints(const unsigned int& n_obs);
    void setOccupancy(const double& occ);
    inline void setLogOddsOccupancy(const double& log_odds_occ);
    void setParams(const params_t& params_);

    gu::Vector3 getMean() const;
    gu::Matrix3x3 getCovariance() const;
    inline unsigned int getNumObs() const;
    double getOccupancy() const;
    inline double getLogOddsOccupancy() const;
    params_t getParams() const;

    void addPointObservation(const observation_t& o);
    void addPointObservations(const observation_list_t& o);

    bool update();

  private:

    bool updateGaussian(params_t& out);
    bool updateOccupancy(double& out);

    inline double toLogOdds(const double& occ) const;
    inline double fromLogOdds(const double& log_odds_occ) const;

    void countEmptyAndOccupied(const observation_list_t& obs,
                               unsigned int& ne,
                               unsigned int& no);

    params_t params;
    bool has_params;

    // Occupancy update probabilities
    double prob_occ, prob_emp;

    observation_list_t obs_accum; // cleared on every gaussian update
  }; //\class GaussianVoxel

} //\namespace ndtm

#endif
