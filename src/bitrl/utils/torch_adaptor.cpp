#include "bitrl/base/bitrl_config.h"


#ifdef USE_PYTORCH

#include "bitrl/utils/torch_adaptor.h"
#include <torch/torch.h>
#include <vector>

namespace bitrl{
namespace utils{
namespace pytorch{

	
TorchAdaptor::value_type 
TorchAdaptor::to_torch(const std::vector<bool>& data, DeviceType dtype, bool requires_grad){
	
	auto device_ =  dtype != DeviceType::CPU ? torch::kCUDA : torch::kCPU;
	auto options = torch::TensorOptions()
	                      .dtype(torch::kBool)
						  .device(device_)
						  .requires_grad(requires_grad);
	
	torch::Tensor d = torch::zeros(static_cast<long int>(data.size()), options);
	
	for(uint_t i=0; i < data.size(); ++i){
		d[i] = data[i];
	}
	
	return d;
}

TorchAdaptor::value_type 
TorchAdaptor::to_torch(const std::vector<real_t>& data, 
                       DeviceType dtype, bool requires_grad){
	auto device_ =  dtype != DeviceType::CPU ? torch::kCUDA : torch::kCPU;
	auto options = torch::TensorOptions()
	                      .dtype(torch::kFloat64)
						  .device(device_)
						  .requires_grad(requires_grad);
    torch::Tensor d = torch::from_blob(	const_cast<real_t*>(data.data()),
										{static_cast<long int>(data.size())}, options).clone(); 
	return d;
}

TorchAdaptor::value_type 
TorchAdaptor::to_torch(const std::vector<float_t>& data, 
                       DeviceType dtype, bool requires_grad){
	auto device_ =  dtype != DeviceType::CPU ? torch::kCUDA : torch::kCPU;
	auto options = torch::TensorOptions()
	                      .dtype(torch::kFloat)
						  .device(device_)
						  .requires_grad(requires_grad);
    torch::Tensor d = torch::from_blob(	const_cast<float_t*>(data.data()),
										{static_cast<long int>(data.size())}, options).clone(); 
	return d;
}

 
TorchAdaptor::value_type 
TorchAdaptor::to_torch(const std::vector<int_t>& data, 
                       DeviceType dtype, bool requires_grad){
	auto device_ =  dtype != DeviceType::CPU ? torch::kCUDA : torch::kCPU;
	auto options = torch::TensorOptions()
	                      .dtype(torch::kInt)
						  .device(device_)
						  .requires_grad(requires_grad);
    torch::Tensor d = torch::from_blob(	const_cast<int_t*>(data.data()),
										{static_cast<long int>(data.size())}, options).clone(); 
	return d;
}

TorchAdaptor::value_type
TorchAdaptor::to_torch(const std::vector<uint_t>& data, 
					   DeviceType dtype, 
					   bool requires_grad){

	// cast the data to lint
	std::vector<lint_t> values_(data.size());
								 
	// make the input values floats
	for(uint_t i=0; i<data.size(); ++i){
		values_[i] = static_cast<lint_t>(data[i]);
	}
	
	return TorchAdaptor::to_torch(values_, dtype, requires_grad);
							
}

 
TorchAdaptor::value_type 
TorchAdaptor::to_torch(const std::vector<lint_t>& data, 
                       DeviceType dtype, bool requires_grad){
	auto device_ =  dtype != DeviceType::CPU ? torch::kCUDA : torch::kCPU;
	auto options = torch::TensorOptions()
	                      .dtype(torch::kLong)
						  .device(device_)
						  .requires_grad(requires_grad);
    torch::Tensor d = torch::from_blob(	const_cast<lint_t*>(data.data()),
										{static_cast<long int>(data.size())}, options).clone(); 
	return d;
}

template<>
TorchAdaptor::value_type 
TorchAdaptor::stack(const std::vector<TorchAdaptor::value_type>& values, 
                    DeviceType dtype,
					bool requires_grad){
	auto device_ =  dtype != DeviceType::CPU ? torch::kCUDA : torch::kCPU;
    return torch::stack(values, 0).to(device_).set_requires_grad(requires_grad);
}

template<>
TorchAdaptor::value_type 
TorchAdaptor::stack(const std::vector<std::vector<real_t>>& values, 
					DeviceType dtype,
					bool requires_grad){
	
	std::vector<TorchAdaptor::value_type> t_values;
	t_values.reserve(values.size());
	
	for(auto& v:values){
		TorchAdaptor::value_type t = TorchAdaptor::to_torch(v, dtype);
		t_values.push_back(t);
	}
	
	auto device_ =  dtype != DeviceType::CPU ? torch::kCUDA : torch::kCPU;
    return torch::stack(t_values, 0).to(device_).set_requires_grad(requires_grad);
}

template<>
TorchAdaptor::value_type 
TorchAdaptor::stack(const std::vector<std::vector<float_t>>& values, 
                    DeviceType dtype, bool requires_grad){
	
	std::vector<TorchAdaptor::value_type> t_values;
	t_values.reserve(values.size());
	
	for(auto& v:values){
		TorchAdaptor::value_type t = TorchAdaptor::to_torch(v, dtype);
		t_values.push_back(t);
	}
	
	auto device_ =  dtype != DeviceType::CPU ? torch::kCUDA : torch::kCPU;
    return torch::stack(t_values, 0).to(device_).set_requires_grad(requires_grad);
}


template<>
TorchAdaptor::value_type 
TorchAdaptor::stack(const std::vector<std::vector<lint_t>>& values, 
                    DeviceType dtype, bool requires_grad){
	
	std::vector<TorchAdaptor::value_type> t_values;
	t_values.reserve(values.size());
	
	for(auto& v:values){
		TorchAdaptor::value_type t = TorchAdaptor::to_torch(v, dtype);
		t_values.push_back(t);
	}
	
	auto device_ =  dtype != DeviceType::CPU ? torch::kCUDA : torch::kCPU;
    return torch::stack(t_values, 0).to(device_).set_requires_grad(requires_grad);
}

template<>
TorchAdaptor::value_type 
TorchAdaptor::stack(const std::vector<std::vector<int_t>>& values, 
                    DeviceType dtype, bool requires_grad){
	
	std::vector<TorchAdaptor::value_type> t_values;
	t_values.reserve(values.size());
	
	for(auto& v:values){
		TorchAdaptor::value_type t = TorchAdaptor::to_torch(v, dtype);
		t_values.push_back(t);
	}
	
	auto device_ =  dtype != DeviceType::CPU ? torch::kCUDA : torch::kCPU;
    return torch::stack(t_values, 0).to(device_).set_requires_grad(requires_grad);
}

template<>
TorchAdaptor::value_type 
TorchAdaptor::stack(const std::vector<std::vector<bool>>& values, 
                    DeviceType dtype, bool requires_grad){
	
	std::vector<TorchAdaptor::value_type> t_values;
	t_values.reserve(values.size());
	
	for(auto& v:values){
		TorchAdaptor::value_type t = TorchAdaptor::to_torch(v, dtype);
		t_values.push_back(t);
	}
	
	auto device_ =  dtype != DeviceType::CPU ? torch::kCUDA : torch::kCPU;
    return torch::stack(t_values, 0).to(device_).set_requires_grad(requires_grad);
}

TorchAdaptor::value_type 
TorchAdaptor::cat(const std::vector<real_t>& values, 
				  DeviceType dtype, bool requires_grad){
					  
	auto device_ =  dtype != DeviceType::CPU ? torch::kCUDA : torch::kCPU;
    return torch::cat(TorchAdaptor::to_torch(values, dtype, requires_grad), 0).to(device_).set_requires_grad(requires_grad);				  
}

torch_tensor_t
TorchAdaptor::operator()(real_t value)const{
    return operator()(std::vector<real_t>(1, value));
}

torch_tensor_t
TorchAdaptor::operator()(const std::vector<real_t>& data)const{
    return torch::tensor(data);
}

torch_tensor_t
TorchAdaptor::operator()(const std::vector<float_t>& data)const{
    return torch::tensor(data);
}

torch_tensor_t
TorchAdaptor::operator()(const std::vector<int>& data)const{
    return torch::tensor(data);
};

/*
TorchAdaptor::value_type
TorchAdaptor::stack(const std::vector<value_type>& values, DeviceType dtype)const{

	auto device_ =  dtype != DeviceType::CPU ? torch::kCUDA : torch::kCPU;
    return torch::stack(values, 0).to(device_);
}

*/

template<>
std::vector<int>
TorchAdaptor::to_vector(torch_tensor_t tensor){

    auto cont_t = tensor.contiguous();
    return std::vector<int>(cont_t.data_ptr<int>(),
                              cont_t.data_ptr<int>() + cont_t.numel());
}

template<>
std::vector<uint_t>
TorchAdaptor::to_vector(torch_tensor_t tensor){

    auto cont_t = tensor.contiguous();
    return std::vector<uint_t>(cont_t.data_ptr<long int>(),
                               cont_t.data_ptr<long int>() + cont_t.numel());
}

template<>
std::vector<lint_t>
TorchAdaptor::to_vector(torch_tensor_t tensor){

    auto cont_t = tensor.contiguous();
    return std::vector<lint_t>(cont_t.data_ptr<long int>(),
                              cont_t.data_ptr<long int>() + cont_t.numel());
}

template<>
std::vector<float_t>
TorchAdaptor::to_vector(torch_tensor_t tensor){

    auto cont_t = tensor.contiguous();
    return std::vector<float>(cont_t.data_ptr<float>(),
                              cont_t.data_ptr<float>() + cont_t.numel());
}

template<>
std::vector<real_t>
TorchAdaptor::to_vector(torch_tensor_t tensor){

    auto cont_t = tensor.contiguous();
    return std::vector<real_t>(cont_t.data_ptr<double>(),
                              cont_t.data_ptr<double>() + cont_t.numel());
}

}
}
}
#endif


