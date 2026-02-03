#ifndef KF_MODEL_BASE_H
#define KF_MODEL_BASE_H

#include <map>
#include <string>
#include <stdexcept> //for std::invalid_argument

namespace bitrl{
namespace estimation{
	
template<typename MatrixType>
class KFModelBase
{
public:
	
	typedef MatrixType matrix_type;
	
	virtual void add_matrix(const std::string& name, matrix_type& mat);
	virtual matrix_type& get_matrix(const std::string& mat);
	virtual const matrix_type& get_matrix(const std::string& mat)const;
	
protected:
	
	std::map<std::string, matrix_type> matrices_;
	
};

template<typename MatrixType>
void 
KFModelBase<MatrixType>::add_matrix(const std::string& name, matrix_type& mat){
	 matrices_.insert_or_assign(name, mat);	
}

template<typename MatrixType>
typename KFModelBase<MatrixType>::matrix_type& 
KFModelBase<MatrixType>::get_matrix(const std::string& name){
	
	auto itr = matrices_.find(name);
	if(itr != matrices_.end()){
		return itr->second;
	}
	
	throw std::invalid_argument("Matrix not found");
}

template<typename MatrixType>
const typename KFModelBase<MatrixType>::matrix_type& 
KFModelBase<MatrixType>::get_matrix(const std::string& name)const{
	
	auto itr = matrices_.find(name);
	if(itr != matrices_.end()){
		return itr->second;
	}
	
	throw std::invalid_argument("Matrix not found");
}
	
template<typename MatrixType, typename StateType>
class KFMotionModelBase: public KFModelBase<MatrixType>
{
public:
	
	typedef MatrixType matrix_type;
	typedef StateType state_type;
	
	virtual state_type& get_state(){return state_;}
	virtual const state_type& get_state()const{return state_;}
	virtual void set_state(const state_type& state){state_ = state;}
	
protected:
	
	// the state 
	state_type state_;
};
	
	
}
}

#endif