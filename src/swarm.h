#ifndef PMR_MATH_SWARM_H
#define PMR_MATH_SWARM_H
#include "particle.h"
#include <pthread.h>
namespace pmr
{
  namespace math
  {

    class PSOSolution
    {
      public:
        Pose gbest_pose;
        double gbestTrans_value;
        double gbestRot_value;
        PSOSolution(Pose & pose,double & bestTransValue, double & bestRotValue)
        {
          gbest_pose=pose;
          gbestTrans_value=bestTransValue;
          gbestRot_value=bestRotValue;
        }
    };

    class Swarm
    {
      public:
        std::vector<Particle> particles;//粒子群

        Swarm();
        void setPopulationSize(int size);//种群大小
        int getPopulationSize();
        void setTarget(Pose & targetPose);
        void setMaxIteration(int maxIter);
        void setSearchSpaceBound(Pose & min, Pose & max);
        void setSearchSpeedBound(Pose & min, Pose & max);

        PSOSolution compute();
      
      private:
        int population_size;//种群大小
        int max_iteration;//最大迭代次数
        int generation;//当前代数
        Pose gbest_pose;//种群最优位置
        double gbestTrans_value;//种群在平移维度的最优适应度
        double gbestRot_value;//种群在旋转维度的最优适应度
        double gw_value;
        double c1,c2;//学习因子
        Pose limit_v,limit_p;
        Pose _targetPose;//目标位置姿态
        pthread_t this_thead;
        double fitnessScore;//适应度
        Pose _minBound, _maxBound;//搜索空间边界
        Pose _minVelocity,_maxVelocity;
        double transNormPara, rotNormPara;//归一化因子

        Pose assignValue(Pose & value, const Pose & bound);
        void evaluateFitness(int particle_id);
        void updateVelocity(int particle_id);
        void updatePosition(int particle_id);
        double calPoseDistance(Pose & p1, Pose & p2);
        double calPointDistance(Pose & p1, Pose & p2);
        void initialize();
    };

  }
}
#endif
