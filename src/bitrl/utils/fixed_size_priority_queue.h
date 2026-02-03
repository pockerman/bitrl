#ifndef FIXED_SIZE_PRIORITY_QUEUE_H
#define FIXED_SIZE_PRIORITY_QUEUE_H

#include "cubeai/base/cubeai_types.h"
#include "cubeai/utils/cubeai_concepts.h"
#include <vector>
#include <functional>

namespace cubeai {
namespace containers {

namespace detail
{


///
///
///
template<typename T, class Container = std::vector<T>>
class priority_queue_common
{
public:

    typedef T value_type;
    typedef Container container_type;

    typedef typename Container::iterator iterator;
    typedef typename Container::const_iterator const_iterator;

    ///
    /// \brief Constructor
    ///
    explicit priority_queue_common(uint_t max_size) noexcept;

    ///
    /// \brief size
    /// \return
    ///
    uint_t size()const noexcept{return pq_.size();}

    ///
    /// \brief capacity
    /// \return
    ///
    uint_t capacity()const noexcept{return capacity_;}

    ///
    /// \brief empty
    /// \return
    ///
    bool empty()const noexcept{return pq_.empty();}

    ///
    /// \brief top
    /// \return
    ///
    const value_type& top()const{return pq_.front();}

    ///
    /// \brief top
    /// \return
    ///
    value_type top(){return pq_.front();}




    iterator begin(){return pq_.begin();}
    iterator end(){return pq_.end();}

    const_iterator begin()const{return pq_.begin();}
    const_iterator end()const{return pq_.end();}

protected:

    ///
    /// \brief push_back
    /// \param value
    ///
    void push_back(const T& value){pq_.push_back(value);}

    ///
    /// \brief pop_back
    ///
    void pop_back(){pq_.pop_back();}

    ///
    /// \brief capacity_
    ///
    uint_t capacity_;

    ///
    /// \brief pq_
    ///
    container_type pq_;

};

template<typename T, class Container>
priority_queue_common<T, Container>::priority_queue_common(uint_t max_size) noexcept
    :
    capacity_(max_size),
    pq_()
{}

}

///
/// \brief FixedSizeMaxPriorityQueue
///
template<typename T, utils::concepts::is_default_constructible Compare = std::less<T>, class Container = std::vector<T>>
class FixedSizeMaxPriorityQueue: public detail::priority_queue_common<T, Container>
{

public:

    typedef T value_type;
    typedef Container container_type;
    typedef Compare value_compare;
    typedef typename Container::iterator iterator;
    typedef typename Container::const_iterator const_iterator;

    ///
    /// \brief Constructor
    ///
    explicit FixedSizeMaxPriorityQueue(uint_t max_size) noexcept;

    ///
    /// \brief push
    /// \param value
    ///
    void push(const value_type& value);

    ///
    /// \brief pop
    ///
    void pop()noexcept;

    ///
    /// \brief top_and_pop
    /// \return
    ///
    value_type top_and_pop();

private:

    ///
    /// \brief value_cp_
    ///
    value_compare value_cp_;
};



template<typename T, utils::concepts::is_default_constructible Compare, class Container>
FixedSizeMaxPriorityQueue<T, Compare, Container>::FixedSizeMaxPriorityQueue(uint_t max_size) noexcept
    :
    detail::priority_queue_common<T, Container>(max_size),
    value_cp_()
{}


template<typename T, utils::concepts::is_default_constructible Compare, class Container>
void
FixedSizeMaxPriorityQueue<T, Compare, Container>::push(const value_type& value){


    if(this->size() >= this->capacity()){

        // we need to differentiate if we
        // implement a max_heap or a min_heap
        auto min = std::min_element(this->begin(), this->end());

        // only if this is better
        if(*min < value){
            *min = value;
        }
    }
    else{

       this->push_back(value);
    }
    std::make_heap(this->begin(), this->end(), value_cp_);
}

template<typename T, utils::concepts::is_default_constructible Compare, class Container>
void
FixedSizeMaxPriorityQueue<T, Compare, Container>::pop()noexcept{

    if(this->empty()){
        return;
    }

    std::pop_heap(this->begin(), this->end(), value_cp_);
    this->pop_back();
}

template<typename T, utils::concepts::is_default_constructible Compare, class Container>
typename FixedSizeMaxPriorityQueue<T, Compare, Container>::value_type
FixedSizeMaxPriorityQueue<T, Compare, Container>::top_and_pop(){
    auto item = this->top();
    pop();
    return item;
}


///
/// \brief FixedSizeMaxPriorityQueue
///
template<typename T, utils::concepts::is_default_constructible Compare = std::greater<T>, class Container = std::vector<T>>
class FixedSizeMinPriorityQueue: public detail::priority_queue_common<T, Container>
{

public:

    typedef T value_type;
    typedef Container container_type;
    typedef Compare value_compare;
    typedef typename Container::iterator iterator;
    typedef typename Container::const_iterator const_iterator;

    ///
    /// \brief Constructor
    ///
    explicit FixedSizeMinPriorityQueue(uint_t max_size) noexcept;

    ///
    /// \brief push
    /// \param value
    ///
    void push(const value_type& value);

    ///
    /// \brief pop
    ///
    void pop()noexcept;

    ///
    /// \brief top_and_pop
    /// \return
    ///
    value_type top_and_pop();

private:

    ///
    /// \brief value_cp_
    ///
    value_compare value_cp_;
};



template<typename T, utils::concepts::is_default_constructible Compare, class Container>
FixedSizeMinPriorityQueue<T, Compare, Container>::FixedSizeMinPriorityQueue(uint_t max_size) noexcept
    :
    detail::priority_queue_common<T, Container>(max_size),
    value_cp_()
{}


template<typename T, utils::concepts::is_default_constructible Compare, class Container>
void
FixedSizeMinPriorityQueue<T, Compare, Container>::push(const value_type& value){


    if(this->size() >= this->capacity()){

        // get the max element
        auto max = std::max_element(this->begin(), this->end());

        // evoke the max element if the given
        // value is smaller than it
        if(*max > value){
            *max = value;
        }
    }
    else{

       this->push_back(value);
    }
    std::make_heap(this->begin(), this->end(), value_cp_);
}

template<typename T, utils::concepts::is_default_constructible Compare, class Container>
void
FixedSizeMinPriorityQueue<T, Compare, Container>::pop()noexcept{

    if(this->empty()){
        return;
    }

    std::pop_heap(this->begin(), this->end(), value_cp_);
    this->pop_back();
}

template<typename T, utils::concepts::is_default_constructible Compare, class Container>
typename FixedSizeMinPriorityQueue<T, Compare, Container>::value_type
FixedSizeMinPriorityQueue<T, Compare, Container>::top_and_pop(){
    auto item = this->top();
    pop();
    return item;
}

}

}

#endif // FIXED_SIZE_PRIORITY_QUEUE_H
