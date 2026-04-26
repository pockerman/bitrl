//
// Created by alex on 11/22/25.
//

#include "bitrl/bitrl_config.h"

#ifdef BITRL_MQTT

#include "bitrl/bitrl_types.h"
#include "bitrl/dynamics/diff_drive_dynamics.h"
#include "bitrl/dynamics/system_state.h"
#include "bitrl/network/mqtt_subscriber.h"
#include "bitrl/sensors/ekf_sensor_fusion.h"
#include "bitrl/sensors/sensor_type_enum.h"
#include "../../src/bitrl/sensors/messages/vector_message.h"
#include "bitrl/utils/unit_converter.h"

#include <chrono>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

namespace example
{
using namespace bitrl;

const real_t DT = 0.1;

// the motion model to feed
class MotionModel
{
  public:
    typedef dynamics::SysState<4> state_type;
    typedef DynMat<real_t> matrix_type;

    MotionModel(std::array<std::string, 4> &&names);

    void init_matrices();
    const matrix_type &get_matrix(const std::string &mname) const;
    const state_type &get_state() const { return state_; }
    state_type &get_state() { return state_; }
    bool has_matrix(const std::string & /*name*/) const { return false; }

    void evaluate(const std::string &input_message, const std::string &sensor_message,
                  const std::string &topic);

    // update the Jacobian matrix
    void update_jacobian();

  private:
    state_type state_;
    matrix_type F_;
    matrix_type Fj_;
    matrix_type B_;
};

MotionModel::MotionModel(std::array<std::string, 4> &&names)
    : state_(std::move(names), 0.0), F_(4, 4), Fj_(4, 4), B_(4, 2)
{
}

void MotionModel::init_matrices()
{
    F_ << 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    Fj_ << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    B_ << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0;

    state_[0] = 0.0;
    state_[1] = 0.0;
    state_[2] = 0.0;
    state_[3] = 0.0;
}

const MotionModel::matrix_type &MotionModel::get_matrix(const std::string &name) const
{
    if (name == "F")
    {
        return Fj_;
    }

    throw std::runtime_error("MotionModel::get_matrix: unknown name");
}

void MotionModel::evaluate(const std::string &input_message, const std::string &sensor_message,
                           const std::string &topic)
{
    // turn the input message into a vector
    auto vector_message = sensors::EigenVectorMessage<real_t>::parse(input_message);

    if (vector_message.has_value())
    {
        const DynVec<real_t> u = vector_message.value().message;

        // get the state as vector
        auto x = state_.as_vector();
        auto yaw = state_.get("YAW");

        // reform the B matrix
        B_ << DT * std::cos(yaw), 0.0, DT * std::sin(yaw), 0.0, 0.0, DT, 1.0, 0.0;

        x = F_ * x.transpose() + B_ * u.transpose();
        // update the state
        for (uint i = 0; i < x.size(); ++i)
        {
            state_[i] = x[i];
        }

        update_jacobian();
    }
}

void MotionModel::update_jacobian()
{
    auto yaw = state_.get("YAW");
    auto v = state_.get("V");

    Fj_ << 1.0, 0.0, -DT * v * std::sin(yaw), DT * std::cos(yaw), 0.0, 1.0, DT * v * std::cos(yaw),
        DT * std::sin(yaw), 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0;
}

class ObservationModel
{

  public:
    // the ExtendedKalmanFilter expects an exposed
    // input_type
    typedef DynVec<real_t> input_type;

    ObservationModel();

    // simply return the state
    const DynVec<real_t> evaluate(const DynVec<real_t> &state) const;
    const DynVec<real_t> convert_sensor_message(const std::string &sensor_message,
                                                const std::string &topic) const;
    bool has_matrix(const std::string & /*name*/) const { return false; }

    // get the H or M matrix
    const DynMat<real_t> &get_matrix(const std::string &name) const;

  private:
    DynMat<real_t> H_;
    DynMat<real_t> Hj_;
};

ObservationModel::ObservationModel() : H_(2, 4), Hj_(2, 4)
{
    H_ = DynMat<real_t>::Zero(2, 4);
    Hj_ = DynMat<real_t>::Zero(2, 4);
    H_(0, 0) = 1.0;
    H_(1, 1) = 1.0;
    Hj_(0, 0) = 1.0;
    Hj_(1, 1) = 1.0;
}

const DynVec<real_t> ObservationModel::convert_sensor_message(const std::string &sensor_message,
                                                              const std::string & /*topic*/) const
{
    auto message = bitrl::sensors::EigenVectorMessage<real_t>::parse(sensor_message);
    if (message.has_value())
    {
        return message.value().message;
    }

    throw std::logic_error("sensor_message is not an EigenVector");
}

const DynVec<real_t> ObservationModel::evaluate(const DynVec<real_t> &state) const
{
    // make it a column vector so that
    // we can to the matrix vector multiplication
    ColVec<real_t> state_(state.transpose());
    return H_ * state_;
}

const DynMat<real_t> &ObservationModel::get_matrix(const std::string &name) const
{
    if (name == "H")
    {
        return Hj_;
    }

    throw std::logic_error("Invalid matrix name. Name " + name + " not found");
}

} // namespace example

int main()
{

    using namespace example;

    std::array<std::string, 4> names = {"X", "Y", "YAW", "V"};
    MotionModel motion_model(std::move(names));
    motion_model.init_matrices();
    ObservationModel observation_model;

    sensors::EKFSensorFusion<MotionModel, ObservationModel> sensor_fusion(motion_model,
                                                                          observation_model);

    DynMat<real_t> P = DynMat<real_t>::Zero(4, 4);
    P(0, 0) = 1.0;
    P(1, 1) = 1.0;
    P(2, 2) = 1.0;
    P(3, 3) = 1.0;

    DynMat<real_t> Q = DynMat<real_t>::Zero(4, 4);
    Q(0, 0) = 0.1 * 0.1; // variance of location on x-axis
    Q(1, 1) = 0.1 * 0.1; // variance of location on y-axis
    Q(2, 2) = utils::unit_converter::degrees_to_rad(1.0) *
              utils::unit_converter::degrees_to_rad(1.0); // variance of yaw angle
    Q(3, 3) = 1.0;                                        // variance of velocity

    DynMat<real_t> R = DynMat<real_t>::Zero(2, 2);
    R(0, 0) = 1.0;
    R(1, 1) = 1.0;

    sensor_fusion.with_matrix("P", P).with_matrix("Q", Q).with_matrix("R", R);

    sensor_fusion.add_sensor_topic("tcp://localhost:1883", "GPS_TOPIC",
                                   sensors::SensorTypeEnum::VECTOR_SENSOR);
    sensor_fusion.add_input_topic("tcp://localhost:1883", "U_TOPIC");

    network::MqttSubscriber state_publisher("tcp://localhost:1883", "STATE_TOPIC");
    state_publisher.connect();

    auto total_sim_time = 0.0;
    while (total_sim_time <= 50.0)
    {
        total_sim_time += DT;

        // make a sensor update
        sensor_fusion.step();
        auto state = motion_model.get_state();
        auto state_str = state.as_json();

        // let's send the state to the Python
        // version
        state_publisher.publish(state_str);
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    return 0;
}
#else

#include <iostream>

int main()
{
    std::cerr << "This example requires MQTT to be enable. Reconfigure bitrl with ENABLE_MQTT=ON"
              << std::endl;
    return 1;
}

#endif
