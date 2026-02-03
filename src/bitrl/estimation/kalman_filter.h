#include "bitrl/bitrl_types.h"
#include "bitrl/utils/std_map_utils.h"
#include "Eigen/Dense"
#include <boost/noncopyable.hpp>

#include <any>
#include <map>
#include <string>
#include <stdexcept> //for std::invalid_argument


namespace bitrl{
namespace estimation{
	

///
/// \brief Configuration class for the Kalman filter
///

template<typename MotionModelType, typename ObservationModelType>
struct KalmanFilterConfig
{
	
	
    typedef MotionModelType motion_model_type;
    typedef ObservationModelType observation_model_type;

	///
	/// \brief Pointer to the motion model
	///
    motion_model_type* motion_model;
	
	///
	/// \brief Pointer to the observation model
	///
    const observation_model_type* observation_model;

    DynMat<real_t> B;
    DynMat<real_t> P;
    DynMat<real_t> Q;
    DynMat<real_t> K;
    DynMat<real_t> R;
};

///
/// \brief Linear Kalman Filter implementation.
/// See: An Introduction to the Kalman Filter, TR 95-041
/// by Greg Welch1 and Gary Bishop
/// https://www.cs.unc.edu/~welch/media/pdf/kalman_intro.pdf
///
/// The algorithm is implemented as follows:
///
/// prediction step:
///
/// \f[\hat{\mathbf{x}}_{k} = F_k \mathbf{x}_{k-1} + B_k   \mathbf{u}_k + \mathbf{w}_k\f]
///
/// \f[\hat{P}_{k} = F_{k-1}  P_{k-1}  F_{k-1}^T +  Q_{k-1}\f]
///
/// update step:
///
/// \f[K_k = \hat{P}_{k}  H_{k}^T * (H_k  \hat{P}_{k} * H_{k}^T +  R_k )^{-1}\f]
///
/// \f[\mathbf{x}_k = \hat{\mathbf{x}}_{k} + K_k  (z_k - h( \hat{x}_{k}, 0))\f]
///
/// \f[P_k = (I - K_k  H_k)  \hat{P}_{k}\f]
///
/// where \f$w_k\f$ and \f$v_k\f$  represent process and measurement noise respectively.
/// They are assumed independent and normally distributed:
///
/// \f[p(w) \sim N(0,Q)\f]
///
/// \f[p(v) \sim N(0,R)\f]
///
/// The gain matrix K says how much the predictions should be corrected
/// The following matrices dimensions are assumed:
///
/// state vector:   x n x 1
/// control vector: u l x 1
/// meas. vector:   y m x 1
///
/// \f[ \mathbf{F} n \times n \f]
/// \f[ \mathfb{P} n \times n \f]
/// \f[ \mathbf{B} n \times l \f]
/// \f[ \mathbf{H} m \times n \f]
/// \f[ \mathbf{K} n \times m \f]
/// \f[ \mathbf{Q} n \times n \f]
/// \f[ \mathbf{R} m \times m \f]
///
template<typename MotionModelType, typename ObservationModelType>
class KalmanFilter: private boost::noncopyable
{

public:
	
	typedef KalmanFilterConfig<MotionModelType, ObservationModelType>  config_type;
	typedef DynMat<real_t> matrix_type;
	
    typedef MotionModelType motion_model_type;
    typedef ObservationModelType observation_model_type;
	typedef typename config_type::motion_model_type::state_type state_type;
    typedef std::map<std::string, std::any> input_type;

    ///
    /// \brief KalmanFilter Constructor
    ///
    KalmanFilter(const KalmanFilterConfig<MotionModelType, ObservationModelType>& config);

    ///
    /// \brief Estimate the state. This function simply
    /// wraps the predict and update steps described by the
    /// functions below
    ///
    state_type& estimate(const input_type& input );

    ///
    /// \brief Predicts the state vector x and the process covariance matrix P using
    /// the given input control u accroding to the following equations
    ///
    /// \f[\hat{x}_{k = F_k* x_{k-1} + B_k *  u_k + w_k\f]
    ///
    /// \f[\hat{P}_{k} = F_{k-1} * P_{k-1} * F_{k-1}^T +  Q_{k-1}\f]
    ///
    /// where \f$x_{k-1}\f$ is the state at the previous step, \f$u_{k}\f$
    /// is the control signal and w_k is the error associated with the
    /// control signal. In input argument passed to the function is meant
    /// to model in a tuple all the arguments needed. F, is the dynamics matrix
    /// and Q is the covariance matrix associate with the control signal
    ///
    /// The control input argument should supply both
    /// \f$u_k\f$ and \f$w_k\f$ vectors
    ///
    ///
    void predict(const input_type& input);

    ///
    /// \brief Updates the gain matrix \f$K\f$, the  state vector \f$x\f$ and covariance matrix P
    /// using the given measurement z_k according to the following equations
    ///
    /// K_k = \hat{P}_{k} * H_{k}^T * (H_k * \hat{P}_{k} * H_{k}^T +  R_k )^{-1}
    /// x_k = \hat{x}_{k} + K_k * (z_k - H * \hat{x}_{k}
    /// P_k = (I - K_k * H_k) * \hat{P}_{k}
    void update(const input_type& input);

    ///
    /// \brief Set the motion model
    ///
    void set_motion_model(motion_model_type& motion_model)
    {motion_model_ptr_ = &motion_model;}

    /// \brief Set the observation model
    void set_observation_model(const observation_model_type& observation_model)
    {observation_model_ptr_ = &observation_model;}
	
	///
	/// \brief Set the matrix and return *this
	///
	KalmanFilter& with_matrix(const std::string& name, const matrix_type& mat);

    ///
    /// \brief Set the matrix used by the filter
    ///
    void set_matrix(const std::string& name, const matrix_type& mat);

    ///
    /// \brief Returns true if the matrix with the given name exists
    ///
    bool has_matrix(const std::string& name)const;

    ///
    /// \brief Returns the state
    ///
    const state_type& get_state()const{return motion_model_ptr_->get_state();}

    ///
    /// \brief Returns the state
    ///
    state_type& get_state(){return motion_model_ptr_->get_state();}

    ///
    /// \brief Returns the state property with the given name
    ///
    real_t get(const std::string& name)const{return motion_model_ptr_->get(name);}

    ///
    /// \brief Returns the name-th matrix
    ///
    const matrix_type& operator[](const std::string& name)const;

    ///
    /// \brief Returns the name-th matrix
    ///
    matrix_type& operator[](const std::string& name);

private:

    ///
    /// \brief pointer to the function that computes f
    ///
    motion_model_type* motion_model_ptr_;

    ///
    /// \brief pointer to the function that computes h
    ///
    const observation_model_type* observation_model_ptr_;

    ///
    /// \brief Matrices used by the filter internally
    ///
    std::map<std::string, matrix_type> matrices_;
};


template<typename MotionModelType, typename ObservationModelType>
KalmanFilter<MotionModelType,
             ObservationModelType>::KalmanFilter(const KalmanFilterConfig<MotionModelType, ObservationModelType>& config)
    :
    motion_model_ptr_(config.motion_model),
    observation_model_ptr_(config.observation_model)
{
    // set the matrices
    set_matrix("B", config.B);
    set_matrix("P", config.P);
    set_matrix("Q", config.Q);
    set_matrix("K", config.K);
    set_matrix("R", config.R);
}


template<typename MotionModelType, typename ObservationModelType>
const DynMat<real_t>&
KalmanFilter<MotionModelType,ObservationModelType>::operator[](const std::string& name)const{

    auto itr = matrices_.find(name);

    if(itr == matrices_.end()){
        throw std::invalid_argument("Matrix: "+name+" does not exist");
    }

    return itr->second;
}

template<typename MotionModelType, typename ObservationModelType>
DynMat<real_t>&
KalmanFilter<MotionModelType,ObservationModelType>::operator[](const std::string& name){

    auto itr = matrices_.find(name);

    if(itr == matrices_.end()){
        throw std::invalid_argument("Matrix: "+name+" does not exist");
    }

    return itr->second;
}

template<typename MotionModelType, typename ObservationModelType>
KalmanFilter<MotionModelType, ObservationModelType>&
KalmanFilter<MotionModelType, 
             ObservationModelType>::with_matrix(const std::string& name, const matrix_type& mat){

		set_matrix(name, mat);
		return *this;
}

template<typename MotionModelType, typename ObservationModelType>
void
KalmanFilter<MotionModelType, 
             ObservationModelType>::set_matrix(const std::string& name,
											   const matrix_type& mat){

    if(name != "Q" && 
       name != "K" && 
       name != "R" && 
       name != "P" &&
       name != "B"){
        throw std::logic_error("Invalid matrix name. Name: "+
                               name+
                               " not in [Q, K, R, P, B]");
    }

    matrices_.insert_or_assign(name, mat);
}

template<typename MotionModelType, typename ObservationModelType>
bool
KalmanFilter<MotionModelType,ObservationModelType>::has_matrix(const std::string& name)const{

    auto itr = matrices_.find(name);
    return itr != matrices_.end();
}

template<typename MotionModelType, typename ObservationModelType>
typename KalmanFilter<MotionModelType, ObservationModelType>::state_type&
KalmanFilter<MotionModelType,
                     ObservationModelType>::estimate(const input_type& input ){
    predict(input);
    update(input);
	
	return get_state();
}

template<typename MotionModelType, typename ObservationModelType>
void
KalmanFilter<MotionModelType,
             ObservationModelType>::predict(const input_type& input ){

    if(!motion_model_ptr_){
        throw std::runtime_error("Motion model has not been set");
    }

    auto u = bitrl::utils::template resolve<DynVec<real_t>>("u", input);
    auto w = bitrl::utils::template resolve<DynVec<real_t>>("w", input);

    // make a state predicion using the
    // motion model
    auto& x = motion_model_ptr_->get_state();
    
    // get the matrix that describes the dynamics
    // of the system
    auto& F = motion_model_ptr_->get_matrix("F");
    auto& B = (*this)["B"];

	// update the state vector
    x = F*x + B*u + w;
    //state.set(x);

    // predict the covariance matrix
    auto& P = (*this)["P"];
    auto& Q = (*this)["Q"];
    auto F_T =  F.transpose();

    P = (F*P*F_T) + Q;
}

template<typename MotionModelType, typename ObservationModelType>
void
KalmanFilter<MotionModelType,
             ObservationModelType>::update(const input_type& input){

    if(!motion_model_ptr_){
        throw std::runtime_error("Motion model has not been set");
    }

    if(!observation_model_ptr_){
        throw std::runtime_error("Observation model has not been set");
    }

    auto& x = motion_model_ptr_->get_state();
    auto& P = (*this)["P"];
    auto& R = (*this)["R"];

    auto& H = observation_model_ptr_->get_matrix("H");
    auto H_T = H.transpose();

    try{

      auto S = H*P*H_T + R;
      auto S_inv = S.inverse(); //inv(S);

	  // compute the gain matrix
      if(has_matrix("K")){
          auto& K = matrices_["K"];
          K = P*H_T*S_inv;
      }
      else{
          auto K = P*H_T*S_inv;
          set_matrix("K", K);
      }

      auto& K = (*this)["K"];
      auto z = bitrl::utils::template resolve<DynVec<real_t>>("z", input);
      auto innovation = z - H*x;

      if(K.cols() != innovation.size()){
          throw std::runtime_error("Matrix columns: "+
                                    std::to_string(K.cols())+
                                    " not equal to vector size: "+
                                    std::to_string(innovation.size()));
      }

      x += K*innovation;
      //state.set(x);

	  auto I = matrix_type::Identity(x.size(), x.size());
      
      // update the error covariance matrix
      P = (I - K*H)*P;
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
