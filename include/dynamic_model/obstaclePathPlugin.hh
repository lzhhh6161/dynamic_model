#ifndef OBSTACLEPATHPLUGIN_HH
#define OBSTACLEPATHPLUGIN_HH
#include <gazebo/gazebo.hh>
#include <ignition/math.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <math.h>

namespace gazebo
{
  class DynamicObstacle : public ModelPlugin{
  private:
  	// Gazebo stuff
  	physics::ModelPtr model; // Pointer to the model
  	event::ConnectionPtr updateConnection; // Pointer to the update event connection

  	// custom attr
  	sdf::ElementPtr sdf;
  	double velocity;
  	bool orientation;
    bool loop;
    double angularVelocity;

  	std::vector<ignition::math::Vector3d> path;
  	std::vector<std::vector<double>> pathWithAngle;
  	std::vector<double> timeKnot;
    
    std::vector<double> accelerations;  // 每个路径段的加速度
    double maxVelocity;                 // 最大速度限制
    double currentVelocity;             // 当前速度（用于加速计算）

  public: 
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
    std::vector<double>& interpolateAngle(double start, double end, double dx);
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(DynamicObstacle)
}

#endif