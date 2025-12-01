//
// Created by alex on 11/23/25.
//

#ifndef EKF_SENSOR_FUSION_H
#define EKF_SENSOR_FUSION_H

#include "bitrl/bitrl_types.h"
#include "bitrl/sensors/sensor_type_enum.h"
#include "Eigen/Dense"

#include <chrono>
#include <string>
#include <memory>
#include <unordered_map>
#include <map>


namespace bitrl
{
    namespace network
    {
        // forward declaration
        class MqttSubscriber;
    }

    namespace sensors
    {
        template<typename MotionModelType, typename ObservationModelType>
        class EKFSensorFusion
        {
        public:

            typedef MotionModelType motion_model_type;
            typedef ObservationModelType observation_model_type;
            typedef DynMat<real_t> matrix_type;
            typedef std::string topic_type;

            ///
            ///
            EKFSensorFusion(motion_model_type& motion_model,
                            observation_model_type& observation_model);
            ~EKFSensorFusion();

            void step(real_t dt);

            EKFSensorFusion& with_matrix(const std::string& name, const matrix_type& mat);

            std::optional<std::string> read_from_topic(const topic_type& topic,
                std::chrono::milliseconds timeout = std::chrono::milliseconds(1000));

            void add_input_topic(const std::string& mqtt_url, const topic_type& topic);

            void add_sensor_topic(const std::string& mqtt_url,
                                  const topic_type& topic,
                                  SensorTypeEnum sensor_type);

            ///  Add the modelled sensor noise
            /// @param topic
            /// @param noise
            void add_sensor_noise(const topic_type& topic, const DynMat<real_t>& noise);

            /// Predict the next state
            /// @param dt
            void predict(real_t dt, const std::string& input_message,
                         const std::string& sensor_message, const topic_type& topic);

            void update(real_t dr, const std::string& sensor_message, const topic_type& topic);

            /// \brief Returns the name-th matrix
            const matrix_type& operator[](const std::string& name)const;

            /// \brief Returns the name-th matrix
            matrix_type& operator[](const std::string& name);

            /// \brief Returns true if the matrix with the given name exists
            bool has_matrix(const std::string& name)const;

        private:

            motion_model_type* motion_model_;
            const observation_model_type* observation_model_;

            /// \brief Matrices used by the filter internally
            std::map<std::string, matrix_type> matrices_;
            std::unique_ptr<bitrl::network::MqttSubscriber> input_subscriber_;
            std::unordered_map<topic_type, std::unique_ptr<bitrl::network::MqttSubscriber>> sensors_;
            std::unordered_map<topic_type, SensorTypeEnum> topic_to_sensor_type_;

        };

        template<typename MotionModelType, typename ObservationModelType>
        EKFSensorFusion<MotionModelType, ObservationModelType>::EKFSensorFusion(motion_model_type& motion_model,
         observation_model_type& observation_model)
            :
        motion_model_(&motion_model),
        observation_model_(&observation_model),
        sensors_(),
        topic_to_sensor_type_()
        {}

        template<typename MotionModelType, typename ObservationModelType>
        EKFSensorFusion<MotionModelType, ObservationModelType>::~EKFSensorFusion()
        {}

        template<typename MotionModelType, typename ObservationModelType>
        bool
        EKFSensorFusion<MotionModelType, ObservationModelType>::has_matrix(const std::string& name)const{
            auto itr = matrices_.find(name);
            return itr != matrices_.end();
        }

        template<typename MotionModelType, typename ObservationModelType>
        const typename EKFSensorFusion<MotionModelType, ObservationModelType>::matrix_type&
        EKFSensorFusion<MotionModelType, ObservationModelType>::operator[](const std::string& name)const{

            auto itr = matrices_.find(name);
            if(itr == matrices_.end()){
                throw std::invalid_argument("Matrix: "+name+" does not exist");
            }
            return itr->second;
        }

        template<typename MotionModelType, typename ObservationModelType>
        typename EKFSensorFusion<MotionModelType, ObservationModelType>::matrix_type&
        EKFSensorFusion<MotionModelType, ObservationModelType>::operator[](const std::string& name){

           auto itr = matrices_.find(name);
           if(itr == matrices_.end()){
               throw std::invalid_argument("Matrix: "+name+" does not exist");
           }
           return itr->second;
        }

        template<typename MotionModelType, typename ObservationModelType>
        EKFSensorFusion<MotionModelType, ObservationModelType>&
        EKFSensorFusion<MotionModelType, ObservationModelType>::with_matrix(const std::string& name, const matrix_type& mat)
        {
            if(name != "Q" && name != "K" && name != "R" && name != "P"){
                throw std::logic_error("Invalid matrix name. Name: "+
                                       name+
                                       " not in [Q, K, R, P]");
            }
            matrices_.insert_or_assign(name, mat);
            return *this;
        }

        template<typename MotionModelType, typename ObservationModelType>
        void
        EKFSensorFusion<MotionModelType, ObservationModelType>::add_input_topic(const std::string& mqtt_url,
                                                                                const topic_type& topic)
        {
            input_subscriber_ =  std::make_unique<bitrl::network::MqttSubscriber>(mqtt_url, topic);
            input_subscriber_ -> connect();
        }

        template<typename MotionModelType, typename ObservationModelType>
        void
        EKFSensorFusion<MotionModelType, ObservationModelType>::add_sensor_topic(const std::string& mqtt_url,
                                          const std::string& topic,
                                          SensorTypeEnum sensor_type)
        {
            auto subscriber = std::make_unique<bitrl::network::MqttSubscriber>(mqtt_url, topic);
            subscriber -> connect(); // connect to MQTT broker
            sensors_[topic] = std::move(subscriber);
            topic_to_sensor_type_.insert({topic, sensor_type});
        }

        template<typename MotionModelType, typename ObservationModelType>
        std::optional<std::string>
        EKFSensorFusion<MotionModelType, ObservationModelType>::read_from_topic(const std::string& topic,
                                    std::chrono::milliseconds timeout )
        {

            auto topic_itr = sensors_.find(topic);

            if (topic_itr == sensors_.end())
            {
                return std::nullopt;
            }

            auto topic_message = topic_itr->second->poll(timeout);

            if (!topic_message.has_value()) {
                return std::nullopt;
            }

            return topic_message;
        }

        template<typename MotionModelType, typename ObservationModelType>
        void
        EKFSensorFusion<MotionModelType, ObservationModelType>::step(real_t dt)
        {

            // loop over the sensor topics and perform
            // and update
            for (auto& sensor : sensors_)
            {
                auto sensor_message = read_from_topic(sensor.first, std::chrono::milliseconds(200));

                // if we have a message, estimate and update
                // the state
                if (sensor_message.has_value())
                {
                    // read the input assocoate with the sensor read
                    auto input_message = input_subscriber_ -> poll(std::chrono::milliseconds(1000));
                    predict(dt, input_message.value(), sensor_message.value(), sensor.first);
                    update(dt, sensor_message.value(), sensor.first);
                }
            }
        }

        template<typename MotionModelType, typename ObservationModelType>
        void
        EKFSensorFusion<MotionModelType, ObservationModelType>::predict(real_t dt,
                                                                         const std::string& input_message,
                                                                         const std::string& sensor_message,
                                                                         const topic_type& topic)
        {
            /// make a state prediction using the
            /// motion model
            motion_model_->evaluate(dt, input_message, sensor_message, topic);

            auto& P = (*this)["P"];
            auto& Q = (*this)["Q"];

            // get the Jacobian description for the
            // motion model
            auto& F = motion_model_->get_matrix("F");
            auto F_T = F.transpose();

            P = F * P * F_T;

            if (motion_model_ -> has_matrix("L"))
            {
                auto& L = motion_model_->get_matrix("L");
                auto L_T = L.transpose();

                P += L * Q * L_T;
            }
            else
            {
                // Most EKFs simply add Q
                P += Q;
            }
        }

        template<typename MotionModelType, typename ObservationModelType>
        void
        EKFSensorFusion<MotionModelType, ObservationModelType>::update(real_t dt,
                                                                         const std::string& sensor_message,
                                                                         const topic_type& topic)
        {
            auto& state = motion_model_->get_state();
            auto& P = (*this)["P"];
            auto& R = (*this)["R"];

            auto z = observation_model_ -> convert_sensor_message(sensor_message, topic);
            auto zpred = observation_model_ -> evaluate(state.as_vector());

            auto& H = observation_model_->get_matrix("H");
            auto H_T = H.transpose();

            // compute \partial{h}/\partial{v} the jacobian of the observation model
            // w.r.t the error vector
            auto& M = observation_model_->get_matrix("M");
            auto M_T = M.transpose(); //trans(M);

            try{

                // S = H*P*H^T + M*R*M^T
                auto S = H*P*H_T + M*R*M_T;
                auto S_inv = S.inverse();

                if(has_matrix("K")){
                    auto& K = (*this)["K"];
                    K = P*H_T*S_inv;
                }
                else{
                    auto K = P*H_T*S_inv;
                    with_matrix("K", K);
                }

                auto& K = (*this)["K"];
                auto innovation = z - zpred;

                // we need to take the transpose
                auto  innovation_t = innovation.transpose();

                if(K.cols() != innovation_t.rows()){
                    throw std::runtime_error("Matrix columns: "+
                                              std::to_string(K.cols())+
                                              " not equal to vector size: "+
                                              std::to_string(innovation_t.rows()));
                }

                //auto vec = K * innovation_t;
                state += K * innovation_t;

                //IdentityMatrix<real_t> I(state.size());
                auto I = matrix_type::Identity(state.size(), state.size());
                /// update the covariance matrix
                P =  (I - K*H)*P;
            }
            catch(...){

                // this is a singular matrix what
                // should we do? Simply use the predicted
                // values and log the fact that there was a singular matrix
                throw;
            }
        }


    }
}

#endif //SENSOR_FUSION_H
