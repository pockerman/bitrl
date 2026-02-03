#ifndef EXTENDED_KALMAN_FILTER_H
#define EXTENDED_KALMAN_FILTER_H

#include "bitrl/bitrl_types.h"
#include "Eigen/Dense"
#include <boost/noncopyable.hpp>
#include <map>
#include <string>
#include <iostream>

namespace bitrl{
namespace estimation{
	
///
/// Implements the Extended Kalman filter algorithm.
/// The class expects a number of inputs in order to be useful. Concretely
/// the application must specity
///
/// MotionModelTp a motion model
/// ObservationModelTp observation model
///
/// The MotionModelTp should expose the following functions
///
/// evaluate(MotionModelTp input)-->State&
/// get_matrix(const std::string)--->DynMat
///
/// In particular, the class expects the L, F matrices to be supplied via the
/// get_matix function of the motion model.
///
/// Similarly, the  ObservationModelTp should expose the following functions
///
/// evaluate(ObservationModelTp& input)--->DynVec
/// get_matrix(const std::string)--->DynMat
///
/// In particular, the class expects the M, H matrices to be supplied via the
/// get_matix function of the observation model.
///
/// Finally, the application should supply the P, Q, R matrices associated
/// with the process
///

template<typename MotionModelTp, typename ObservationModelTp>
class ExtendedKalmanFilter: private boost::noncopyable
{
public:

    typedef MotionModelTp motion_model_type;
    typedef ObservationModelTp observation_model_type;
    typedef typename motion_model_type::input_type motion_model_input_type;
    typedef typename motion_model_type::matrix_type matrix_type;
    typedef typename motion_model_type::state_type state_type;
    typedef typename observation_model_type::input_type observation_model_input_type;

    /// \brief Constructor
    ExtendedKalmanFilter();

	///
    /// \brief Constructor. Initialize with a writable reference
	/// of the motion model and a read only reference of the observation model
	///
    ExtendedKalmanFilter(motion_model_type& motion_model,
                         const observation_model_type& observation_model);

	///
    /// \brief Destructor
	///
    ~ExtendedKalmanFilter();

    /// \brief Estimate the state. This function simply
    /// wraps the predict and update steps described by the
    /// functions below
    void estimate(const std::tuple<motion_model_input_type, 
	                               observation_model_input_type>& input );

    /// \brief Predicts the state vector x and the process covariance matrix P using
    /// the given input control u accroding to the following equations
    ///
    /// \hat{x}_{k = f(x_{k-1}, u_{k}, w_k)
    /// \hat{P}_{k} = F_{k-1} * P_{k-1} * F_{k-1}^T + L_{k-1} * Q_{k-1} * L_{k-1}^T
    ///
    /// where x_{k-1} is the state at the previous step, u_{k}
    /// is the control signal and w_k is the error associated with the
    /// control signal. In input argument passed to the function is meant
    /// to model in a tuple all the arguments needed. F, L are Jacobian matrices
    /// and Q is the covariance matrix associate with the control signal
    void predict(const motion_model_input_type& input);

    /// \brief Updates the gain matrix K, the  state vector x and covariance matrix P
    /// using the given measurement z_k according to the following equations
    ///
    /// K_k = \hat{P}_{k} * H_{k}^T * (H_k * \hat{P}_{k} * H_{k}^T + M_k * R_k * M_k^T)^{-1}
    /// x_k = \hat{x}_{k} + K_k * (z_k - h( \hat{x}_{k}, 0))
    /// P_k = (I - K_k * H_k) * \hat{P}_{k}
    void update(const observation_model_input_type& z);

    /// \brief Set the motion model
    void set_motion_model(motion_model_type& motion_model)
    {motion_model_ptr_ = &motion_model;}

    /// \brief Set the observation model
    void set_observation_model(const observation_model_type& observation_model)
    {observation_model_ptr_ = &observation_model;}

    /// \brief Set the matrix used by the filter
    void set_matrix(const std::string& name, const matrix_type& mat);

    /// \brief Returns true if the matrix with the given name exists
    bool has_matrix(const std::string& name)const;

    /// \brief Returns the state
    const state_type& get_state()const{return motion_model_ptr_->get_state();}

    /// \brief Returns the state
    state_type& get_state(){return motion_model_ptr_->get_state();}

    /// \brief Returns the state property with the given name
    real_t get(const std::string& name)const{return motion_model_ptr_->get_state_property(name);}

    /// \brief Returns the name-th matrix
    const DynMat<real_t>& operator[](const std::string& name)const;

    /// \brief Returns the name-th matrix
    DynMat<real_t>& operator[](const std::string& name);
	
	///
	/// \brief Set the matrix and return *this
	///
	ExtendedKalmanFilter& with_matrix(const std::string& name, const matrix_type& mat);
           
protected:

    /// \brief pointer to the function that computes f
    motion_model_type* motion_model_ptr_;

    /// \brief pointer to the function that computes h
    const observation_model_type* observation_model_ptr_;

    /// \brief Matrices used by the filter internally
    std::map<std::string, matrix_type> matrices_;

};  

template<typename MotionModelTp, typename ObservationModelTp>
ExtendedKalmanFilter<MotionModelTp,ObservationModelTp>::ExtendedKalmanFilter()
    :
    motion_model_ptr_(nullptr),
    observation_model_ptr_(nullptr)
{}

template<typename MotionModelTp, typename ObservationModelTp>
ExtendedKalmanFilter<MotionModelTp,
                     ObservationModelTp>::ExtendedKalmanFilter(motion_model_type& motion_model,
                                                               const observation_model_type& observation_model)
    :
    motion_model_ptr_(&motion_model),
    observation_model_ptr_(&observation_model)
{}

template<typename MotionModelTp, typename ObservationModelTp>
ExtendedKalmanFilter<MotionModelTp,
                     ObservationModelTp>::~ExtendedKalmanFilter()
{}

template<typename MotionModelTp, typename ObservationModelTp>
void
ExtendedKalmanFilter<MotionModelTp,
                     ObservationModelTp>::set_matrix(const std::string& name,
                                                     const matrix_type& mat){

    if(name != "Q" && name != "K" && name != "R" && name != "P"){
        throw std::logic_error("Invalid matrix name. Name: "+
                               name+
                               " not in [Q, K, R, P]");
    }

    matrices_.insert_or_assign(name, mat);
}

template<typename MotionModelTp, typename ObservationModelTp>
ExtendedKalmanFilter<MotionModelTp, ObservationModelTp>& 
ExtendedKalmanFilter<MotionModelTp, ObservationModelTp>::with_matrix(const std::string& name, 
                                                                     const matrix_type& mat){
	set_matrix(name, mat);
	return *this;
}

template<typename MotionModelTp, typename ObservationModelTp>
bool
ExtendedKalmanFilter<MotionModelTp,ObservationModelTp>::has_matrix(const std::string& name)const{

    auto itr = matrices_.find(name);
    return itr != matrices_.end();
}

template<typename MotionModelTp, typename ObservationModelTp>
const DynMat<real_t>&
ExtendedKalmanFilter<MotionModelTp,ObservationModelTp>::operator[](const std::string& name)const{

    auto itr = matrices_.find(name);

    if(itr == matrices_.end()){
        throw std::invalid_argument("Matrix: "+name+" does not exist");
    }

    return itr->second;
}

template<typename MotionModelTp, typename ObservationModelTp>
DynMat<real_t>&
ExtendedKalmanFilter<MotionModelTp,ObservationModelTp>::operator[](const std::string& name){

    auto itr = matrices_.find(name);

    if(itr == matrices_.end()){
        throw std::invalid_argument("Matrix: "+name+" does not exist");
    }

    return itr->second;
}


template<typename MotionModelTp, typename ObservationModelTp>
void
ExtendedKalmanFilter<MotionModelTp,
                     ObservationModelTp>::estimate(const std::tuple<motion_model_input_type,
                                                   observation_model_input_type>& input ){

    predict(input.template get<0>());
    update(input.template get<1>());
}

template<typename MotionModelTp, typename ObservationModelTp>
void
ExtendedKalmanFilter<MotionModelTp,ObservationModelTp>::predict(const motion_model_input_type& u){

    /// make a state prediction using the
    /// motion model
    motion_model_ptr_->evaluate(u);

    auto& P = (*this)["P"];
    auto& Q = (*this)["Q"];

    auto& L = motion_model_ptr_->get_matrix("L");
    auto L_T = L.transpose(); //trans(L);

    auto& F = motion_model_ptr_->get_matrix("F");
    auto F_T = F.transpose(); //trans(F);

    P = F * P * F_T + L*Q*L_T;
}

template<typename MotionModelTp, typename ObservationModelTp>
void
ExtendedKalmanFilter<MotionModelTp,
                     ObservationModelTp>::update(const observation_model_input_type&  z){

    auto& state = motion_model_ptr_->get_state();
    auto& P = (*this)["P"];
    auto& R = (*this)["R"];

    auto zpred = observation_model_ptr_->evaluate(z);

    auto& H = observation_model_ptr_->get_matrix("H");
    auto H_T = H.transpose(); //trans(H);

    // compute \partial{h}/\partial{v} the jacobian of the observation model
    // w.r.t the error vector
    auto& M = observation_model_ptr_->get_matrix("M");
    auto M_T = M.transpose(); //trans(M);

     try{

        // S = H*P*H^T + M*R*M^T
        auto S = H*P*H_T + M*R*M_T;

        auto S_inv = S.inverse(); //inv(S);

        if(has_matrix("K")){
            auto& K = (*this)["K"];
            K = P*H_T*S_inv;
        }
        else{
            auto K = P*H_T*S_inv;
            set_matrix("K", K);
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
        state += K * innovation_t; //.add(K*innovation);

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


#endif