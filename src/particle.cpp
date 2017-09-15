#include "particle.h"
#include <boost/random.hpp>
#include <iostream>
#define DBL_MAX 999999999;

namespace pmr
{
  namespace math
  {

    boost::mt19937 generator;
    Particle::Particle()
    {
      pose=Pose(0,0,0,0,0,0);
      velocity=Pose(0,0,0,0,0,0);
      pbestTrans_value=DBL_MAX;
      pbestRot_value=DBL_MAX;
      pbest_pose=Pose(0,0,0,0,0,0);
      minVel=Pose(0,0,0,0,0,0);
      maxVel=minVel;
      minPos=pose;
      maxPos=pose;
    }

    void Particle::setMinMaxBound(
        Pose & min_pose,Pose & max_pose)
    {
      minPos=min_pose;
      maxPos=max_pose;
    }

    void Particle::setMinMaxVelocity(
        Pose & min_vel, Pose & max_vel)
    {
      minVel=min_vel;
      maxVel=max_vel;
    }

    void Particle::randomize()
    {
      velocity=getRandomPose(minVel, maxVel);
      pose=getRandomPose(minPos, maxPos);
    }

    Pose Particle::getRandomPose(Pose& min, Pose& max)
    {
      boost::uniform_real<float> distr_x(min.x,max.x);
      boost::uniform_real<float> distr_y(min.y,max.y);
      boost::uniform_real<float> distr_z(min.z,max.z);
      boost::uniform_real<float> distr_phi(min.phi,max.phi);
      boost::uniform_real<float> distr_theta(min.theta,max.theta);
      boost::uniform_real<float> distr_psi(min.psi,max.psi);
      
      Pose p(
          distr_x(generator),
          distr_y(generator),
          distr_z(generator),
          distr_phi(generator),
          distr_theta(generator),
          distr_psi(generator));

      return p;
    }

    std::ostream & operator << (std::ostream & out, Particle & p)
    {
      out<<std::endl
        <<"pbestTrans value: "<<p.pbestTrans_value<<std::endl
        <<"pbestRot value: "<<p.pbestRot_value<<std::endl
        <<"pbest pose: "<<p.pbest_pose
        <<"velocity: "<<p.velocity
        <<"pose: "<<p.pose;
    }
  }
}
