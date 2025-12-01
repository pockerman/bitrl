//
// Created by alex on 11/22/25.
//

#include "bitrl/bitrl_types.h"
#include "bitrl/network/mqtt_subscriber.h"
#include "bitrl/sensors/ekf_sensor_fusion.h"
#include "bitrl/sensors/sensor_type_enum.h"
#include "bitrl/sensors/vector_message.h"
#include "bitrl/dynamics/diff_drive_dynamics.h"
#include "bitrl/utils/unit_converter.h"

#include <iostream>
#include <thread>
#include <chrono>
#include <stdexcept>
#include <string>
#include <any>



namespace example
{
    using namespace bitrl;

    const real_t DT = 0.1;
    // Noise matrices
    const DynMat<real_t> INPUT_NOISE = (Eigen::Vector2d(1.0, utils::unit_converter::degrees_to_rad(30.0))).array().square().matrix().asDiagonal();
    const DynMat<real_t> GPS_NOISE   = (Eigen::Vector2d(0.5, 0.5)).array().square().matrix().asDiagonal();


    class MotionModel
    {
    public:

        typedef dynamics::DiffDriveDynamics::matrix_type matrix_type;
        typedef dynamics::DiffDriveDynamics::state_type state_type;
        typedef dynamics::DiffDriveDynamics::input_type input_type;

        MotionModel();

        const matrix_type& get_matrix(const std:: string& mname)const{return diff_drive_dynamics_.get_matrix(mname);}
        const state_type& get_state()const{return diff_drive_dynamics_.get_state(); }
        state_type& get_state(){return diff_drive_dynamics_.get_state();}
        bool has_matrix(const std::string& name)const{return diff_drive_dynamics_.has_matrix(name);}
        void initialize_matrices(const input_type& input){diff_drive_dynamics_.initialize_matrices(input);}
        void set_time_step(real_t dt){diff_drive_dynamics_.set_time_step(dt);}
        void set_matrix_update_flag(bool flag){diff_drive_dynamics_.set_matrix_update_flag(flag);}
        void evaluate(real_t dt, const std::string& input_message,
                      const std::string& sensor_message, const std::string& topic);

    private:
        dynamics::DiffDriveDynamics diff_drive_dynamics_;

    };

    MotionModel::MotionModel()
        :
    diff_drive_dynamics_()
    {}


    void
    MotionModel::evaluate(real_t dt, const std::string& input_message,
                         const std::string& sensor_message, const std::string& topic)
    {
        // turn the input message into a vector
        auto vector_message = sensors::EigenVectorMessage<real_t>::parse(input_message);

        if (vector_message.has_value())
        {
            const DynVec<real_t> input_val = vector_message.value().message;

            std::array<real_t, 2> motion_control_error;
            motion_control_error[0] = 0.0;
            motion_control_error[1] = 0.0;

            std::map<std::string, std::any> input;
            input["v"] = input_val[0];
            input["w"] = input_val[1];
            input["errors"] = motion_control_error;

            diff_drive_dynamics_.evaluate(input);
            return;
        }

    }

    class ObservationModel
    {

    public:

        // the ExtendedKalmanFilter expects an exposed
        // input_type
        typedef  DynVec<real_t> input_type;

        ObservationModel();

        // simply return the state
        const DynVec<real_t> evaluate(const DynVec<real_t>& state)const;
        const DynVec<real_t> convert_sensor_message(const std::string& sensor_message, const std::string& topic)const;

        // get the H or M matrix
        const DynMat<real_t>& get_matrix(const std::string& name)const;

    private:

        DynMat<real_t> H;
        DynMat<real_t> M;
    };

    ObservationModel::ObservationModel()
    :
      H(2, 3),
      M(2, 2)
    {
        H = DynMat<real_t>::Zero(2,3);
        M = DynMat<real_t>::Zero(2,2);
        H(0, 0) = 1.0;
        H(1,1) = 1.0;
        M(0,0) = 1.0;
        M(1, 1) = 1.0;

    }

    const DynVec<real_t>
    ObservationModel::convert_sensor_message(const std::string& sensor_message, const std::string& /*topic*/)const
    {
        auto message = bitrl::sensors::EigenVectorMessage<real_t>::parse(sensor_message);
        if (message.has_value())
        {
            return message.value().message;
        }

        throw std::logic_error("sensor_message is not an EigenVector");
    }

    const DynVec<real_t>
    ObservationModel::evaluate(const DynVec<real_t>& state)const
    {
        return H * state;
    }

    const DynMat<real_t>&
    ObservationModel::get_matrix(const std::string& name)const{
        if(name == "H"){
            return H;
        }
        else if(name == "M"){
            return M;
        }

        throw std::logic_error("Invalid matrix name. Name "+name+ " not found");
    }

}

int main() {

    using namespace  example;

    MotionModel motion_model;
    motion_model.set_time_step(DT);
    motion_model.set_matrix_update_flag(true);

    std::array<real_t, 2> motion_control_error;
    motion_control_error[0] = 0.0;
    motion_control_error[1] = 0.0;

    std::map<std::string, std::any> input;
    input["v"] = 1.0;
    input["w"] = 0.0;
    input["errors"] = motion_control_error;
    motion_model.initialize_matrices(input);




    ObservationModel observation_model;
    sensors::EKFSensorFusion<MotionModel, ObservationModel> sensor_fusion(motion_model, observation_model);

    DynMat<real_t> P = DynMat<real_t>::Zero(3, 3);
    P(0, 0) = 1.0;
    P(1, 1) = 1.0;
    P(2, 2) = 1.0;

    DynMat<real_t> Q = DynMat<real_t>::Zero(2, 2);
    Q(0,0) = 0.001;
    Q(1, 1) = 0.001;

    DynMat<real_t> R = DynMat<real_t>::Zero(2, 2);
    R(0,0) = 1.0;
    R(1, 1) = 1.0;

    sensor_fusion.with_matrix("P", P).with_matrix("Q", Q).with_matrix("R", R);

    sensor_fusion.add_sensor_topic("tcp://localhost:1883", "GPS_TOPIC", sensors::SensorTypeEnum::VECTOR_SENSOR);
    sensor_fusion.add_input_topic("tcp://localhost:1883", "U_TOPIC");

    auto total_sim_time = 0.0;
    while (true)
    {
        total_sim_time += DT;

        // make a sensor update
        sensor_fusion.step(DT);
        auto state = motion_model.get_state();
        std::cout<<"State computed: "<<state<<std::endl;

    }
    return 0;
}


