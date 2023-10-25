#ifndef DART_REALTIME_SSID
#define DART_REALTIME_SSID

#include <memory>
#include <thread>
#include <mutex>

#include <Eigen/Dense>
#include <iostream>
#include <fstream>

#include "dart/math/MathTypes.hpp"
#include "dart/realtime/VectorLog.hpp"

namespace dart {
namespace simulation {
class World;
}

namespace trajectory {
class LossFn;
class Problem;
class Solution;
class Optimizer;
} // namespace trajectory

namespace realtime {

// SSID = System + State IDentification
class SSID
{
public:
  SSID(
      std::shared_ptr<simulation::World> world,
      std::shared_ptr<trajectory::LossFn> loss,
      int planningHistoryMillis,
      Eigen::VectorXs sensorDims,
      int steps);

  /// This updates the loss function that we're going to move in real time to
  /// minimize. This can happen quite frequently, for example if our loss
  /// function is to track a mouse pointer in a simulated environment, we may
  /// reset the loss function every time the mouse moves.
  void setLoss(std::shared_ptr<trajectory::LossFn> loss);

  /// This sets the optimizer that MPC will use. This will override the default
  /// optimizer. This should be called before start().
  void setOptimizer(std::shared_ptr<trajectory::Optimizer> optimizer);

  /// This returns the current optimizer that MPC is using
  std::shared_ptr<trajectory::Optimizer> getOptimizer();

  /// This sets the problem that MPC will use. This will override the default
  /// problem. This should be called before start().
  void setProblem(std::shared_ptr<trajectory::Problem> problem);

  /// This registers a function that can be used to estimate the initial state
  /// for the inference system from recent sensor history and the timestamp
  void setInitialPosEstimator(
      std::function<Eigen::VectorXs(Eigen::MatrixXs, long)>
          initialPosEstimator);

  void setInitialVelEstimator(
    std::function<Eigen::VectorXs(Eigen::MatrixXs, long)>
    initialVelEstimator);

  /// This returns the current problem definition that MPC is using
  std::shared_ptr<trajectory::Problem> getProblem();

  /// This logs that the sensor output is a specific vector now
  void registerSensorsNow(Eigen::VectorXs sensors, int sensor_id);

  /// This logs that the controls are a specific vector now
  void registerControlsNow(Eigen::VectorXs sensors);

  /// This logs that the sensor output was a specific vector at a specific
  /// moment
  void registerSensors(long now, Eigen::VectorXs sensors, int sensor_id);

  /// This logs that our controls were this value at this time
  void registerControls(long now, Eigen::VectorXs controls);

  /// This starts our main thread and begins running optimizations
  void start();

  /// This stops our main thread, waits for it to finish, and then returns
  void stop();

  /// This runs inference to find mutable values, starting at `startTime`
  void runInference(long startTime);

  Eigen::VectorXs runPlotting(long startTime, s_t upper, s_t lower, int samples);

  Eigen::MatrixXs runPlotting2D(long startTime, Eigen::Vector3s upper, Eigen::Vector3s lower, int x_samples, int y_samples, size_t rest_dim);

  void saveCSVMatrix(std::string filename, Eigen::MatrixXs matrix);

  /// This registers a listener to get called when we finish replanning
  void registerInferListener(
      std::function<
          void(long, Eigen::VectorXs, Eigen::VectorXs, Eigen::VectorXs, long)>
          inferListener);

  // wrapper for locking the buffer
  void attachMutex(std::mutex &mutex_lock);
  void registerLock();
  void registerUnlock();

protected:
  /// This is the function for the optimization thread to run when we're live
  void optimizationThreadLoop();

  bool mRunning;
  std::shared_ptr<simulation::World> mWorld;
  std::shared_ptr<trajectory::LossFn> mLoss;
  int mPlanningHistoryMillis;
  int mPlanningSteps;
  Eigen::VectorXs mSensorDims;
  std::vector<VectorLog> mSensorLogs;
  VectorLog mControlLog;

  std::shared_ptr<trajectory::Optimizer> mOptimizer;
  std::shared_ptr<trajectory::Problem> mProblem;
  std::shared_ptr<trajectory::Solution> mSolution;
  std::thread mOptimizationThread;
  std::mutex* mRegisterMutex;
  bool mLockRegistered = false;
  Eigen::VectorXs mParameters;

  // These are listeners that get called when we finish replanning
  std::vector<std::function<void(
      long, Eigen::VectorXs, Eigen::VectorXs, Eigen::VectorXs, long)> >
      mInferListeners;

  // This is the function that estimates our initial state before launching
  // learning
  std::function<Eigen::VectorXs(Eigen::MatrixXs, long)> mInitialPosEstimator;
  std::function<Eigen::VectorXs(Eigen::MatrixXs, long)> mInitialVelEstimator;
};

} // namespace realtime
} // namespace dart

#endif