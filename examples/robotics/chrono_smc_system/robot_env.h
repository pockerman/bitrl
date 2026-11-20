#ifndef ROBOT_ENV_H
#define ROBOT_ENV_H

#include "bitsim/bitsim_types.h"
#include "bitrl/envs/env_base.h"
#include "bitrl/envs/env_types.h"
#include "bitrl/envs/time_step.h"
#include "bitrl/envs/time_step_type.h"

#include <chrono/physics/ChSystemNSC.h>

#include "robot.h"

#include <any>
#include <string>
#include <unordered_map>

namespace chrono_smc_system
{

using namespace bitsim;

constexpr uint_t STATE_SPACE_SIZE = 3;
constexpr uint_t ACTION_SPACE_SIZE = 3;

using bitrl::TimeStep;
using bitrl::TimeStepTp;

typedef TimeStep<chrono::ChVector3d> time_step_type;
typedef bitrl::envs::ContinuousVectorStateContinuousVectorActionEnv<STATE_SPACE_SIZE,
                                                                    STATE_SPACE_SIZE>
    space_type;

class RobotEnv final : public bitrl::envs::EnvBase<time_step_type, space_type>
{
  public:
    typedef typename space_type::action_type action_type;

    RobotEnv() = default;

    virtual void make(const std::string &version,
                      const std::unordered_map<std::string, std::any> &make_options,
                      const std::unordered_map<std::string, std::any> &reset_options) override;

    virtual void close() override {}
    virtual time_step_type reset() override;
    virtual time_step_type step(const action_type & /*action*/) override;
    void simulate();

  private:
    Robot robot_;
    chrono::ChSystemNSC sys_;
    uint_t sim_counter_{0};
    real_t current_time_{0.0};
    void build_system_(std::shared_ptr<chrono::ChContactMaterialNSC> material);
};

} // namespace chrono_smc_system

#endif // ROBOT_ENV_H
