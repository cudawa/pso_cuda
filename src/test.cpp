//#include "particle.h"
#include "swarm.h"
#include <iostream>
using namespace pmr;

//Unit Test
void swarmComputeTest();

int main(int argc, char** argv)
{
  swarmComputeTest();
}

void swarmComputeTest()
{
  math::Swarm sw;
  math::Pose target(0.15,-0.23,-0.001,M_PI/2.3,-M_PI/4.3, M_PI/5.7);
  math::Pose minBound(-1,-1,-1,-M_PI/2,-M_PI/2,-M_PI/2);
  math::Pose maxBound(1,1,1,M_PI/2,M_PI/2,M_PI/2);
  math::Pose minVel(-0.1,-0.1,-0.1,-M_PI/90,-M_PI/90,-M_PI/90);
  math::Pose maxVel(0.1,0.1,0.1,M_PI/90,M_PI/90,M_PI/90);
  sw.setTarget(target);
  sw.setSearchSpaceBound(minBound,maxBound);
  sw.setSearchSpeedBound(minVel,maxVel);
  sw.setMaxIteration(100);
  sw.setPopulationSize(20);
  math::PSOSolution sol=sw.compute();
  std::cout<<"======================================"<<std::endl;
  std::cout<<">>> target Pose:"<<std::endl<<target<<std::endl;
  std::cout<<">>> compute finished!"<<std::endl
    <<"best pose: "<<sol.gbest_pose<<std::endl
    <<"best trans score: "<<sol.gbestTrans_value<<std::endl
    <<"best rot score: "<<sol.gbestRot_value<<std::endl;
}


