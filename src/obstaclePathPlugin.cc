#include <dynamic_model/obstaclePathPlugin.hh>

namespace gazebo
{
  void DynamicObstacle::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
      // 存储模型指针和SDF指针
      this->model = _parent;
      this->sdf = _sdf;

      // 读取基础参数（线速度、方向跟随、角速度）
      // 线速度：加速度为0时使用（匀速模式）
      if (this->sdf->HasElement("velocity")) {
          this->velocity = _sdf->Get<double>("velocity");
      } else {
          this->velocity = 1.0;  // 默认匀速速度
      }

      // 方向跟随开关
      if (this->sdf->HasElement("orientation")) {
          this->orientation = _sdf->Get<bool>("orientation");
      } else {
          this->orientation = true;  // 默认开启方向跟随
      }

      // 角速度（仅方向跟随开启时生效）
      if (this->orientation) {
          if (this->sdf->HasElement("angular_velocity")) {
              this->angularVelocity = _sdf->Get<double>("angular_velocity");
          } else {
              this->angularVelocity = 0.8;  // 默认角速度
          }
      }

      // 强制禁用loop模式，固定为往返路径（A→B→C→B→A）
      this->loop = false;

      // 读取原始路径点（如A、B、C）
      this->path.clear();
      if (this->sdf->HasElement("path")) {
          sdf::ElementPtr waypointElem = _sdf->GetElement("path")->GetElement("waypoint");
          while (waypointElem) {
              ignition::math::Vector3d wp = waypointElem->Get<ignition::math::Vector3d>();
              this->path.push_back(wp);
              waypointElem = waypointElem->GetNextElement("waypoint");
          }
      }

      // 生成往返路径：在原始路径后添加反向路径（不含起点，避免重复）
      // 例：原始路径[A,B,C] → 生成[A,B,C,B,A]
      std::vector<ignition::math::Vector3d> temp = this->path;
      for (int i = temp.size() - 2; i >= 0; --i) {
          this->path.push_back(temp[i]);
      }

      // 处理路径角度（根据方向跟随开关生成带角度的路径点）
      this->pathWithAngle.clear();
      if (this->orientation) {
          // 方向跟随开启：计算每个点的朝向（沿路径切线方向）
          double yawLast;
          for (int i = 0; i < this->path.size(); ++i) {
              if (i == 0) {
                  // 第一个点：朝向第二个点
                  double xCurr = this->path[i].X(), yCurr = this->path[i].Y();
                  double xNext = this->path[i+1].X(), yNext = this->path[i+1].Y();
                  double yawStart = atan2(yNext - yCurr, xNext - xCurr);
                  this->pathWithAngle.push_back({xCurr, yCurr, this->path[i].Z(), yawStart});
                  yawLast = yawStart;
              } else {
                  // 后续点：添加两个关键帧（转向前后的角度）
                  double xCurr = this->path[i].X(), yCurr = this->path[i].Y(), zCurr = this->path[i].Z();
                  double xPrev = this->path[i-1].X(), yPrev = this->path[i-1].Y();
                  double xNext, yNext;

                  // 确定下一个点（最后一个点的下一个是前一个点）
                  if (i + 1 < this->path.size()) {
                      xNext = this->path[i+1].X();
                      yNext = this->path[i+1].Y();
                  } else {
                      xNext = this->path[i-1].X();
                      yNext = this->path[i-1].Y();
                  }

                  // 当前点的目标朝向（指向下一个点）
                  double yawCurr = atan2(yNext - yCurr, xNext - xCurr);

                  // 添加转向前后的两个关键帧（确保转向平滑）
                  this->pathWithAngle.push_back({xCurr, yCurr, zCurr, yawLast});  // 转向前角度
                  this->pathWithAngle.push_back({xCurr, yCurr, zCurr, yawCurr});  // 转向后角度
                  yawLast = yawCurr;
              }
          }
      } else {
          // 方向跟随关闭：所有点朝向固定（偏航角为0）
          for (int i = 0; i < this->path.size(); ++i) {
              ignition::math::Vector3d wp = this->path[i];
              this->pathWithAngle.push_back({wp.X(), wp.Y(), wp.Z(), 0.0});
          }
      }

      // 读取分段加速度参数（每个路径段对应一个加速度）
      this->accelerations.clear();
      if (this->sdf->HasElement("accelerations")) {
          sdf::ElementPtr accelElem = _sdf->GetElement("accelerations")->GetElement("a");
          while (accelElem) {
              this->accelerations.push_back(accelElem->Get<double>());
              accelElem = accelElem->GetNextElement("a");
          }
      }
      // 若加速度数量不足，用默认值0填充（默认匀速）
      int requiredAccelCount = this->pathWithAngle.size() - 1;  // 路径段数量 = 关键帧数量 - 1
      while (this->accelerations.size() < requiredAccelCount) {
          this->accelerations.push_back(0.0);
      }

      // 读取最大速度限制（防止加速度过大导致速度失控）
      if (this->sdf->HasElement("max_velocity")) {
          this->maxVelocity = _sdf->Get<double>("max_velocity");
      } else {
          this->maxVelocity = 2.0;  // 默认最大速度
      }

      // 计算每个关键帧的时间节点（核心：根据分段加速度计算耗时）
      this->timeKnot.clear();
      double totalTime = 0.0;
      this->timeKnot.push_back(totalTime);  // 初始时间0
      this->currentVelocity = 0.0;         // 初始速度0

      for (int i = 0; i < this->pathWithAngle.size() - 1; ++i) {
          std::vector<double> poseCurr = this->pathWithAngle[i];
          std::vector<double> poseNext = this->pathWithAngle[i+1];

          // 判断当前段是平移还是旋转（坐标不变则为旋转）
          bool isRotation = (poseCurr[0] == poseNext[0] && 
                            poseCurr[1] == poseNext[1] && 
                            poseCurr[2] == poseNext[2]);

          if (!isRotation) {
              // 平移段：根据当前段加速度计算时间
              double xCurr = poseCurr[0], yCurr = poseCurr[1], zCurr = poseCurr[2];
              double xNext = poseNext[0], yNext = poseNext[1], zNext = poseNext[2];
              double distance = sqrt(pow(xNext - xCurr, 2) + 
                                    pow(yNext - yCurr, 2) + 
                                    pow(zNext - zCurr, 2));

              double accel = this->accelerations[i];  // 当前段加速度
              double duration = 0.0;

              if (accel == 0) {
                  // 加速度为0：匀速运动（使用基础速度）
                  duration = distance / this->velocity;
                  this->currentVelocity = this->velocity;  // 保持匀速
              } else {
                  // 带加速度的运动：先加速到最大速度，再匀速（若距离足够）
                  double timeToMax = (this->maxVelocity - this->currentVelocity) / accel;
                  double distanceToMax = this->currentVelocity * timeToMax + 
                                        0.5 * accel * pow(timeToMax, 2);

                  if (distanceToMax >= distance) {
                      // 距离不足：未达最大速度就完成本段
                      duration = sqrt((2 * distance) / accel + pow(this->currentVelocity / accel, 2)) 
                              - this->currentVelocity / accel;
                      this->currentVelocity += accel * duration;
                  } else {
                      // 距离足够：先加速到最大速度，剩余距离匀速
                      double remainingDistance = distance - distanceToMax;
                      double uniformTime = remainingDistance / this->maxVelocity;
                      duration = timeToMax + uniformTime;
                      this->currentVelocity = this->maxVelocity;  // 保持最大速度
                  }
              }

              totalTime += duration;
              this->timeKnot.push_back(totalTime);
          } else {
              // 旋转段：使用角速度计算时间（不受加速度影响）
              double yawCurr = poseCurr[3];
              double yawNext = poseNext[3];
              // 计算最小旋转角度（避免绕圈）
              double angleDiff = std::abs(atan2(sin(yawNext - yawCurr), cos(yawNext - yawCurr)));
              double duration = angleDiff / this->angularVelocity;

              totalTime += duration;
              this->timeKnot.push_back(totalTime);
          }
      }

      // 新增：对路径进行密集线性插值，解决样条插值的速度问题
      std::vector<std::vector<double>> densePath;  // 密集插值后的路径点
      std::vector<double> denseTimeKnot;           // 对应密集点的时间节点
      const int interpolateSteps = 500;  // 每段插值步数，可根据需要调整（越大越接近线性）

      for (int i = 0; i < this->pathWithAngle.size() - 1; ++i) {
          auto& poseCurr = this->pathWithAngle[i];
          auto& poseNext = this->pathWithAngle[i+1];
          double tStart = this->timeKnot[i];
          double tEnd = this->timeKnot[i+1];
          double tStep = (tEnd - tStart) / interpolateSteps;

          // 计算各轴的线性步长
          double xStep = (poseNext[0] - poseCurr[0]) / interpolateSteps;
          double yStep = (poseNext[1] - poseCurr[1]) / interpolateSteps;
          double zStep = (poseNext[2] - poseCurr[2]) / interpolateSteps;
          double yawStep = (poseNext[3] - poseCurr[3]) / interpolateSteps;

          // 插入中间点，形成密集关键帧
          for (int j = 0; j <= interpolateSteps; ++j) {
              double t = tStart + j * tStep;
              double x = poseCurr[0] + j * xStep;
              double y = poseCurr[1] + j * yStep;
              double z = poseCurr[2] + j * zStep;
              double yaw = poseCurr[3] + j * yawStep;

              densePath.push_back({x, y, z, yaw});
              denseTimeKnot.push_back(t);
          }
      }

      // 创建动画并设置关键帧（使用密集插值后的路径）
      gazebo::common::PoseAnimationPtr anim(
          new gazebo::common::PoseAnimation("obstaclePath", totalTime, true));
      gazebo::common::PoseKeyFrame* key;

      // 使用密集插值后的关键帧
      for (int i = 0; i < densePath.size(); ++i) {
          double t = denseTimeKnot[i];
          auto& pose = densePath[i];
          double x = pose[0], y = pose[1], z = pose[2], yaw = pose[3];

          key = anim->CreateKeyFrame(t);
          key->Translation(ignition::math::Vector3d(x, y, z));
          key->Rotation(ignition::math::Quaterniond(0, 0, yaw));  // 仅绕z轴旋转（偏航角）
      }

      // 应用动画到模型
      this->model->SetAnimation(anim);
  }

    std::vector<double>& DynamicObstacle::interpolateAngle(double start, double end, double dx){
      static std::vector<double> interpolation;
      interpolation.clear();  // 清除上一次的插值结果，避免累积
      double angleDiff = end - start;
      double angleDiffABS = std::abs(angleDiff);

      if (angleDiff >= 0 and angleDiffABS <= M_PI){
        for (double mid=start+dx; mid<end; mid+=dx){
          interpolation.push_back(mid);
        }
      }
      else if (angleDiff >= 0 and angleDiffABS > M_PI){
        // minus unitl -PI
        double mid = start-dx;
        while (mid >= -M_PI){
          interpolation.push_back(mid);
          mid -= dx;
        }

        // minus from PI to end
        mid = M_PI;
        while (mid > end){
          interpolation.push_back(mid);
          mid -= dx;
        }
      }
      else if (angleDiff < 0 and angleDiffABS <= M_PI){
        for (double mid=start-dx; mid>end; mid-=dx){
          interpolation.push_back(mid);
        }
      }
      else if (angleDiff < 0 and angleDiffABS > M_PI){
        // plus until PI
        double mid = start+dx;
        while (mid <= M_PI){
          interpolation.push_back(mid);
          mid += dx;
        }

        // plus from -PI to end
        mid = -M_PI;
        while (mid < end){
          interpolation.push_back(mid);
          mid += dx;
        }
      }
      return interpolation;
    }
}
