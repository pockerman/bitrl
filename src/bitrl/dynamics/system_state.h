#ifndef SYSTEM_STATE_H
#define SYSTEM_STATE_H

#include "bitrl/bitrl_types.h"
#include "bitrl/extern/nlohmann/json/json.hpp"

#include <string>
#include <array>
#include <utility>
#include <algorithm>
#include <vector>
#include <ostream>
#include <stdexcept>
#include <iomanip> // std::setprecision


namespace bitrl{
namespace dynamics{

///
/// \brief SysState utility class describing the state of a system
///
template<int dim>
class SysState
{

public:

    ///
    /// \brief The dimension of the state
    ///
    static const int dimension = dim;

    ///
    /// \brief The type of the stored values
    ///
    typedef std::array<std::pair<std::string, real_t>, dim> value_type;

    ///
    /// \brief Extract a state of different dimension
    ///
    template<int dim1, int dim2>
    static void extract(const SysState<dim1>& state, SysState<dim2>& other);

    ///
    /// \brief Constructor. Initialize the state with no names
    ///
    SysState();

    ///
    /// \brief Constructor. Initialize the state with the given names and values
    ///
    SysState(std::array<std::pair<std::string, real_t>, dim>&& values);

    ///
    /// \brief Constructor. Initialize the state with the given names
    /// all variables will be initialized with val
    ///
    SysState(std::array<std::string, dim>&& names, real_t val);

    ///
    /// \brief Copy constructor
    ///
    SysState(const SysState<dim>& other);

    ///
    /// \brief Copy constructor
    ///
    template<int other_dim>
    SysState(const SysState<other_dim>& other);

    ///
    /// \brief Copy assignement constructor
    ///
    SysState& operator=(const SysState<dim>& other);

    ///
    /// \brief Move copy constructor
    ///
    SysState(SysState&& other);

    ///
    /// \brief Move copy constructor
    ///
    SysState& operator=(SysState&& other);

    ///
    /// \brief Add to this state the entries
    /// of the give vector
    ///
    SysState& operator+=(const DynVec<real_t>& vec);

    ///
    /// \brief Subtract from this state the entries
    /// of the give vector
    ///
    SysState& operator-=(const DynVec<real_t>& vec);

    ///
    /// \brief Scale this state by the given factor
    ///
    SysState& operator*=(real_t val);

    ///
    /// \brief Returns the value for the variable name
    ///
    real_t get(const std::string& name)const;

    ///
    /// \brief Set the value for the variable name
    ///
    void set(const std::string& name, real_t val);

    ///
    /// \brief Set the name and value of the i-th variable
    ///
    void set(uint_t i, const std::pair<std::string, real_t>& value);

    ///
    /// \brief Set the values of state variables
    /// container must be of size dim.
    ///
    void add(const DynVec<real_t>& container);
	
	///
    /// \brief Set the values of state variables
    /// container must be of size dim.
    ///
	void add(const std::vector<real_t>& container);

    ///
    /// \brief Returns the size of the system
    ///
    uint_t size()const{return dim;}

	///
    /// \brief Returns a copy of the state values
	///
    const std::array<real_t, dim> get_values()const;

    ///
    /// \brief Returns a copy of the state names
    ///
    const std::vector<std::string_view> get_names()const;

    ///
    /// \brief Access operator
    ///
    real_t& operator[](uint_t);

    ///
    /// \brief Access operator
    ///
    const real_t& operator[](uint_t)const;

    ///
    /// \brief Access operator
    ///
    real_t& operator[](const std::string& name){return this->operator()(name);}

    ///
    /// \brief Access operator
    ///
    const real_t& operator[](const std::string& name)const{return this->operator()(name);}

    ///
    /// \brief Access operator
    ///
    real_t& operator()(const std::string& name);

    ///
    /// \brief Access operator
    ///
    const real_t& operator()(const std::string& name)const;

    ///
    /// \brief Access operator
    ///
    real_t& operator()(const std::string_view name){return (*this)(std::string(name));}

    ///
    /// \brief Access operator
    ///
    const real_t& operator()(const std::string_view name)const{return (*this)(std::string(name));}

    ///
    /// \brief clear the state
    ///
    void clear();

    ///
    /// \brief Print the state at the given stream
    ///
    std::ostream& print(std::ostream& out)const;

    ///
    /// \brief Returns the entries of this state as a DynVec
    ///
    DynVec<real_t> as_vector()const;

    ///
    /// \brief Return the state as string
    ///
    const std::string as_string()const;

    ///  Convert to json format
    /// @return
    nlohmann::json  as_json() const;

    ///
    /// \brief Scale the values of the state
    ///
    void scale(real_t val);
	
	///
	/// \brief Set the values
	///
	template<typename Container>
    void set(const Container& container);
	
	///
	/// TODO: Add this in the future
	/// \brief Returns a view of the values in [start, end)
	///
	//template<typename Container>
	//Container get_values_view(uint_t start, uint_t end)const;

private:

    ///
    /// \brief values_. Array holding the values of the state
    ///
    std::array<std::pair<std::string, real_t>, dim> values_;

};

template<int dim>
template<int dim1, int dim2>
void
SysState<dim>::extract(const SysState<dim1>& state, SysState<dim2>& other){

    static_assert (dim2 <= dim1, "Invalid dimension dim2 > dim1");

    for(auto& name:other.get_names()){
        auto value = state(name);
        other.set(std::string(name), value);
    }
}

template<int dim>
SysState<dim>::SysState()
    :
   values_()
{
    std::for_each(values_.begin(), values_.end(),
                  [](std::pair<std::string, real_t>& item){

        item.first= "NO NAME";
        item.second = 0.0;
    });
}

template<int dim>
SysState<dim>::SysState(std::array<std::pair<std::string, real_t>, dim>&& values)
    :
    values_(values)
{}

template<int dim>
SysState<dim>::SysState(std::array<std::string, dim>&& names, real_t val)
    :
   values_()
{
    for(uint_t i=0; i<dim; ++i){
        values_[i] = std::pair(names[i], val);
    }
}

template<int dim>
SysState<dim>::SysState(const SysState<dim>& other)
    :
      values_(other.values_)
{}

template<int dim>
template<int other_dim>
SysState<dim>::SysState(const SysState<other_dim>& other)
    :
      values_()
{
    static_assert (dim <= other_dim, "Invalid dimension: dim < other_dim" );

    auto names = other.get_names();
    auto values = other.get_values();

    for(uint_t i=0; i<dim; ++i){
        values_[i] = std::make_pair(names[i], values[i]);
    }
}

template<int dim>
SysState<dim>&
SysState<dim>::operator=(const SysState<dim>& other){

    if(this == &other){
        return *this;
    }

    this->values_ = other.values_;
    return *this;
}

template<int dim>
SysState<dim>::SysState(SysState&& other)
    :
      values_(other.values_)
{
    other.clear();
}

template<int dim>
SysState<dim>&
SysState<dim>::operator=(SysState&& other){

    if(this == &other){
        return *this;
    }

    this->values_ = other.values_;
    other.clear();
    return *this;
}

template<int dim>
SysState<dim>&
SysState<dim>::operator+=(const DynVec<real_t>& vec){
    add(vec);
    return *this;
}

template<int dim>
SysState<dim>&
SysState<dim>::operator-=(const DynVec<real_t>& vec){
    add(-1.0*vec);
    return *this;
}

template<int dim>
SysState<dim>&
SysState<dim>::operator*=(real_t val){
    scale(val);
    return *this;
}

template<int dim>
void
SysState<dim>::scale(real_t val){
    std::for_each(values_.begin(),
                  values_.end(),
                  [=](auto& item){
        item.second *= val;
    });
}

template<int dim>
real_t
SysState<dim>::get(const std::string& name)const{


    auto itr = std::find_if(values_.begin(), values_.end(),
                         [&](const std::pair<std::string, real_t>& item){
        return item.first == name;
    });

    if(itr == values_.end()){
        auto error_msg("Invalid variable name. Name ");
        auto names = get_names();
        std::string name_strs("[");
        for(auto& name:names){
            name_strs += std::string(name);
            name_strs += std::string(",");
        }

        name_strs += std::string("]");
        throw std::invalid_argument(error_msg + 
                                    name+
                                    std::string(" not in: ")+ name_strs);
    }

    return itr->second;
}

template<int dim>
DynVec<real_t>
SysState<dim>::as_vector()const{
    DynVec<real_t> vec(dim);
    for(uint_t v=0; v<values_.size(); ++v){
        vec(v) = values_[v].second;
    }
    return vec;
}

template<int dim>
void
SysState<dim>::set(const std::string& name, real_t val){

    auto itr = std::find_if(values_.begin(), values_.end(),
                         [&](const std::pair<std::string, real_t>& item){
        return item.first == name;
    });

    if(itr == values_.end()){
        auto error_msg("Invalid variable name. Name ");
        auto names = get_names();
        std::string name_strs("[");
        for(auto& name:names){

            name_strs += std::string(name);
            name_strs += std::string(",");
        }

        name_strs += std::string("]");
        throw std::invalid_argument(error_msg + 
                                    name+
                                    std::string(" not in: ")+ name_strs);
    }

    itr->second = val;
}

template<int dim>
template<typename Container>
void
SysState<dim>::set(const Container& container){

    if(container.size() != dim){
        throw std::invalid_argument("Container has incorrect size: "+
                                    std::to_string(container.size()) +
                                    " not equal to "+
                                    std::to_string(dim));
    }

    uint counter = 0;
    std::for_each(values_.begin(),
                  values_.end(),
                  [&](auto& arg){arg.second = container[counter++];});
}

template<int dim>
void
SysState<dim>::set(uint_t i, const std::pair<std::string, real_t>& value){
    values_[i] = value;
}

template<int dim>
real_t&
SysState<dim>::operator[](uint_t i){
    return values_[i].second;
}

template<int dim>
const real_t&
SysState<dim>::operator[](uint_t i)const{
    return values_[i].second;
}

template<int dim>
real_t&
SysState<dim>::operator()(const std::string& name){

    auto itr = std::find_if(values_.begin(), values_.end(),
                         [&](const std::pair<std::string, real_t>& item){
        return item.first == name;
    });

    if(itr == values_.end()){
        auto error_msg("Invalid variable name. Name ");
        auto names = get_names();
        std::string name_strs("[");
        for(auto& name:names){

            name_strs += std::string(name);
            name_strs += std::string(",");
        }

        name_strs += std::string("]");
        throw std::invalid_argument(error_msg + 
                                    name+
                                    std::string(" not in: ")+ name_strs);
    }

    return itr->second;
}

template<int dim>
const real_t&
SysState<dim>::operator()(const std::string& name)const{

    auto itr = std::find_if(values_.begin(), values_.end(),
                         [&](const std::pair<std::string, real_t>& item){
        return item.first == name;
    });

   if(itr == values_.end()){
        auto error_msg("Invalid variable name. Name ");
        auto names = get_names();
        std::string name_strs("[");
        for(auto& name:names){

            name_strs += std::string(name);
            name_strs += std::string(",");
        }

        name_strs += std::string("]");
        throw std::invalid_argument(error_msg + 
                                    name+
                                    std::string(" not in: ")+ name_strs);
    }

    return itr->second;
}

template<int dim>
void
SysState<dim>::clear(){

    std::for_each(values_.begin(), values_.end(),
                  [](std::pair<std::string, real_t>& item){
        item.second = 0.0;
    });
}

template<int dim>
const std::array<real_t, dim>
SysState<dim>::get_values()const{

    std::array<real_t, dim> copy;

    for(uint_t i=0; i<dim; ++i){
        copy[i] = values_[i].second;
    }

    return copy;
}

template<int dim>
const std::vector<std::string_view>
SysState<dim>::get_names()const{

    std::vector<std::string_view> copy(dim);

    for(uint_t i=0; i<dim; ++i){
        copy[i] = values_[i].first;
    }

    return copy;

}

template<int dim>
std::ostream&
SysState<dim>::print(std::ostream& out)const{

    out<<std::fixed << std::setprecision(4);

    std::for_each(values_.begin(), values_.end(),
              [&](const std::pair<std::string, real_t>& vals){
       out<<vals.first<<":"<<vals.second<<std::endl;
    });

    return out;
}

template<int dim>
const std::string
SysState<dim>::as_string()const{

    std::string result;
    bool first = true;

    for (const auto& [name, value] : values_) {
        if (!first)
            result += ",";   // add comma before the next element
        first = false;

        result += name;
        result += ":";
        result += std::to_string(value);
    }

    return result;
}
template<int dim>
nlohmann::json
    SysState<dim>::as_json() const
{
    nlohmann::json j;
    for (const auto& [name, value] : values_) {
        j[name] = value;
    }
    return j;
}

template<int dim>
void
SysState<dim>::add(const DynVec<real_t>& container){
    if(container.size() != dim){
        throw std::logic_error("Invalid container size for update. "+
                               std::to_string(container.size())+
                               " should be"+
                               std::to_string(dim));
    }

    for(uint_t i=0; i<dim; ++i){
        values_[i].second += container[i];
    }
}

template<int dim>
void
SysState<dim>::add(const std::vector<real_t>& container){
    if(container.size() != dim){
        throw std::logic_error("Invalid container size for update. "+
                               std::to_string(container.size())+
                               " should be"+
                               std::to_string(dim));
    }

    for(uint_t i=0; i<dim; ++i){
        values_[i].second += container[i];
    }
}

template<int dim>
inline
std::ostream& operator<<(std::ostream& out, const SysState<dim>& state){
    return state.print(out);
}

}

}

#endif // SYSTEM_STATE_H
