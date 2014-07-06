#include <ndt_mapper/GaussianVoxel.h>

namespace gu = geometry_utils;

ndtm::GaussianVoxel::GaussianVoxel() :
  Voxel<gu::Vector3>(),
  prob_occ( 0.6 ), prob_emp( 0.49 ),
  has_params(false)
{}

ndtm::GaussianVoxel::GaussianVoxel(const gu::Vector3& center_,
                                   const double& sizex_,
                                   const double& sizey_,
                                   const double& sizez_,
                                   const double& prob_occ_,
                                   const double& prob_emp_) :
  Voxel<gu::Vector3>(center_, sizex_, sizey_, sizez_),
  prob_occ(prob_occ_), prob_emp(prob_emp_),
  params(center_, arma::eye(3, 3), 0, 0.5),
  has_params(true)
{}

ndtm::GaussianVoxel::GaussianVoxel(const gu::Vector3& center_,
                                   const double& sizex_,
                                   const double& sizey_,
                                   const double& sizez_,
                                   const double& prob_occ_,
                                   const double& prob_emp_,
                                   const params_t& params_) :

  Voxel<gu::Vector3>(center_, sizex_, sizey_, sizez_),
  prob_occ(prob_occ_), prob_emp(prob_emp_),
  params(params_), has_params(true)
{}

ndtm::GaussianVoxel::~GaussianVoxel()
{
  obs_accum.clear();
}

inline bool ndtm::GaussianVoxel::isInitialized()
{
  return has_center && has_dims && has_params;
}

void ndtm::GaussianVoxel::setMean(const gu::Vector3& mean)
{
  params.mean = mean;
}

void ndtm::GaussianVoxel::setCovariance(const gu::Matrix3x3& cov)
{
  params.cov = cov;
}

inline void ndtm::GaussianVoxel::setNumPoints(const unsigned int& n_obs)
{
  params.n_obs = n_obs;
}

void ndtm::GaussianVoxel::setOccupancy(const double& occ)
{
  params.log_odds_occ = toLogOdds(occ);
}

inline void ndtm::GaussianVoxel::setLogOddsOccupancy(const double& log_odds_occ)
{
  params.log_odds_occ = log_odds_occ;
}

void ndtm::GaussianVoxel::setParams(const params_t& params_)
{
  params = params_;
}

gu::Vector3 ndtm::GaussianVoxel::getMean() const
{
  return params.mean;
}

gu::Matrix3x3 ndtm::GaussianVoxel::getCovariance() const
{
  return params.cov;
}

inline unsigned int ndtm::GaussianVoxel::getNumObs() const
{
  return params.n_obs;
}

double ndtm::GaussianVoxel::getOccupancy() const
{
  return fromLogOdds(params.log_odds_occ);
}

inline double ndtm::GaussianVoxel::getLogOddsOccupancy() const
{
  return params.log_odds_occ;
}

ndtm::GaussianVoxel::params_t ndtm::GaussianVoxel::getParams() const
{
  return params;
}

void ndtm::GaussianVoxel::addPointObservation(const ndtm::GaussianVoxel::observation_t& o)
{
  obs_accum.push_back(o);
};

void ndtm::GaussianVoxel::addPointObservations(const ndtm::GaussianVoxel::observation_list_t& o)
{
  obs_accum.insert( obs_accum.end(), o.begin(), o.end() );
}

bool ndtm::GaussianVoxel::updateGaussian(params_t& out)
{
  double n = static_cast<double>(params.n_obs);
  double m = static_cast<double>(obs_accum.size());

  gu::Vector3 mean_prior = params.mean;
  gu::Vector3 mean_inter;

  for (std::list<observation_t>::const_iterator it = obs_accum.begin();
       it != obs_accum.end(); ++it)
  {
    mean_inter(0) += (*it).first(0);
    mean_inter(1) += (*it).first(1);
    mean_inter(2) += (*it).first(2);
  }
  mean_inter /= m;

  // Equation 2 of Saarinen et. al., ICRA 2013
  gu::Vector3 mean_posterior = 1.0 / (n + m) * (n * mean_prior + m * mean_inter);

  if (!arma::is_finite(mean_posterior))
  {
    printf("Mean is inf or nan\n");
    return false;
  }

  gu::Matrix3x3 cov_prior = params.cov;
  gu::Matrix3x3 cov_inter;

  // Equation 11 of Saarinen et. al., ICRA 2013
  gu::Matrix3x3 cov_posterior = cov_prior + cov_inter + (m*n/(m+n))
    * (mean_prior - mean_inter) * arma::trans(mean_prior - mean_inter);

  if (!arma::is_finite(cov_posterior))
  {
    printf("Cov is inf or nan\n");
    return false;
  }

  out.mean = mean_posterior;
  out.cov = cov_posterior;
  out.n_obs = params.n_obs + obs_accum.size();
  obs_accum.clear();

  return true;
}

bool ndtm::GaussianVoxel::updateOccupancy(double& out)
{
  // Equation 12 of Saarinen et. al., ICRA 2013
  unsigned int ne, no;
  ne = no = 0;

  countEmptyAndOccupied(obs_accum, ne, no);

  if (no > 0)
    out = params.log_odds_occ + ( no * toLogOdds( prob_occ ) );
  else
    out = params.log_odds_occ + ( ne * toLogOdds( prob_emp ) );

  if (std::isinf(out) || std::isnan(out))
  {
    printf("Occupancy is inf or nan\n");
    return false;
  }

  return true;
}

bool ndtm::GaussianVoxel::update()
{
  // Only update parameters if we successfully update both
  // updateGaussian() and updateOccupancy()
  params_t update_params;
  if (updateGaussian(update_params))
  {
    double update_log_odds_occ;
    if(updateOccupancy(update_log_odds_occ))
    {
      params.mean = update_params.mean;
      params.cov = update_params.cov;
      params.n_obs = update_params.n_obs;
      params.log_odds_occ = update_log_odds_occ;
      return true;
    }
  }

  return false;
}

inline double ndtm::GaussianVoxel::toLogOdds(const double& occ) const
{
  // Convert occ in [0, 1] to log_odds_occ in (-infty, infty)
  return log( occ / (1.0 - occ ) );
}

inline double ndtm::GaussianVoxel::fromLogOdds(const double& log_odds_occ) const
{
  // Convert log_odds_occ in (-infty, infty) to occ in [0, 1]
  return 1.0 / ( 1.0 + exp(-log_odds_occ) );
}

void ndtm::GaussianVoxel::countEmptyAndOccupied(const observation_list_t& obs,
                                                unsigned int& ne,
                                                unsigned int& no)
{
  // Ensure the counts are zero upon being passed in
  ne = no = 0;

  for (observation_list_t::const_iterator it = obs.begin();
       it != obs.end(); ++it)
  {
    switch (it->second)
    {
      case EMPTY:
        ne++;
      case OCCUPIED:
        no++;
      default:
        break;
    }
  }
}
