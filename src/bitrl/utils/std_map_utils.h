#ifndef STD_MAP_UTILS_H
#define STD_MAP_UTILS_H

///
/// \file math_utils.h
///
/// Various utilities for std::map
///

#include <any>
#include <map>
#include <stdexcept>


namespace bitrl{
namespace utils{
	
///
/// \fn resolve
/// \brief Given the name of the argument return std::any_cast<OutT>(itr->second)
/// where itr is itr = input.find(name)
///
template<typename OutT>
OutT
resolve(const std::string& name,
        const std::map<std::string, std::any>& input){

    auto itr = input.find(name);

    if(itr == input.end()){
        throw std::logic_error("Property: " + name + " not in input");
    }

    return std::any_cast<OutT>(itr->second);

}

	}
}

#endif 