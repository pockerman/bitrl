#ifndef MCTS_NODE_H
#define MCTS_NODE_H

#include "cubeai/base/cubeai_types.h"
#include <memory>

namespace cubeai{
namespace rl{
namespace algos{
namespace mc{
	
///
/// \brief Base class for Nodes in a MC tree search
/// 
template<typename ActionTp, typename StateTp>
class MCTSNodeBase
{
public:

    typedef ActionTp action_type;
    typedef StateTp state_type;

    ///
    /// \brief MCTreeNodeBase
    /// \param parent
    /// \param action
    ///
    MCTSNodeBase(std::shared_ptr<MCTSNodeBase> parent, action_type action);

    ///
    ///
    ///
    virtual ~MCTSNodeBase()=default;

    ///
    /// \brief add_child
    /// \param child
    ///
    void add_child(std::shared_ptr<MCTSNodeBase<ActionTp, StateTp>> child);

    ///
    /// \brief add_child
    /// \param parent
    /// \param action
    ///
    void add_child(std::shared_ptr<MCTSNodeBase<ActionTp, StateTp>> parent, action_type action);

    ///
    /// \brief has_children
    /// \return
    ///
    bool has_children()const noexcept{return children_.empty() != true;}

    ///
    /// \brief get_child
    /// \param cidx
    /// \return
    ///
    std::shared_ptr<MCTSNodeBase> get_child(uint_t cidx){return children_[cidx];}

    ///
    /// \brief n_children
    /// \return
    ///
    uint_t n_children()const noexcept{return children_.size();}

    ///
    /// \brief shuffle_children
    ///
    void shuffle_children()noexcept;

    ///
    /// \brief explored_children
    /// \return
    ///
    uint_t n_explored_children()const noexcept{return explored_children_;}

    ///
    /// \brief update_visits
    ///
    void update_visits()noexcept{total_visits_ += 1;}

    ///
    ///
    ///
    void update_explored_children()noexcept{explored_children_ += 1;}

    ///
    /// \brief update_total_score
    /// \param score
    ///
    void update_total_score(real_t score)noexcept {total_score_ += score;}

    ///
    /// \brief ucb
    /// \param temperature
    /// \return
    ///
    real_t ucb(real_t temperature )const;

    ///
    /// \brief max_ucb_child
    /// \return
    ///
    std::shared_ptr<MCTSNodeBase> max_ucb_child(real_t temperature)const;

    ///
    /// \brief win_pct
    /// \return
    ///
    real_t win_pct()const{return total_score_ / total_visits_ ;}

    ///
    /// \brief total_visits
    /// \return
    ///
    uint_t total_visits()const noexcept{return total_visits_;}

    ///
    /// \brief get_action
    /// \return
    ///
    uint_t get_action()const noexcept{return action_;}

    ///
    /// \brief parent
    /// \return
    ///
    std::shared_ptr<MCTSNodeBase> parent(){return parent_;}

protected:

    ///
    /// \brief total_score_
    ///
    real_t total_score_;

    ///
    /// \brief total_visits_
    ///
    uint_t total_visits_;

    ///
    /// \brief explored_children_
    ///
    uint_t explored_children_;

    ///
    /// \brief action_
    ///
    action_type action_;

    ///
    /// \brief parent_
    ///
    std::shared_ptr<MCTSNodeBase> parent_;

    ///
    /// \brief children_
    ///
    std::vector<std::shared_ptr<MCTSNodeBase>> children_;

};

template<typename ActionTp, typename StateTp>
MCTSNodeBase<ActionTp, StateTp>::MCTSNodeBase(std::shared_ptr<MCTSNodeBase> parent, action_type action)
    :
    total_score_(0.0),
    total_visits_(0),
    explored_children_(0),
    action_(action),
    parent_(parent),
    children_()
{}


template<typename ActionTp, typename StateTp>
void
MCTSNodeBase<ActionTp, StateTp>::add_child(std::shared_ptr<MCTSNodeBase<ActionTp, StateTp>> child){

#ifdef CUBEAI_DEBUG
    assert(child != nullptr && "Cannot add null children");
#endif

   children_.push_back(child);

}

template<typename ActionTp, typename StateTp>
void
MCTSNodeBase<ActionTp, StateTp>::shuffle_children()noexcept{

    if(children_.empty()){
        return;
    }

    std::random_shuffle(children_.begin(), children_.end());
}

template<typename ActionTp, typename StateTp>
void
MCTSNodeBase<ActionTp, StateTp>::add_child(std::shared_ptr<MCTSNodeBase<ActionTp, StateTp>> parent, 
											 action_type action){

    add_child(std::make_shared<MCTSNodeBase<ActionTp, StateTp>>(parent, action));
}

template<typename ActionTp, typename StateTp>
std::shared_ptr<MCTSNodeBase<ActionTp, StateTp>>
MCTSNodeBase<ActionTp, StateTp>::max_ucb_child(real_t temperature)const{

    if(children_.empty()){
        return std::shared_ptr<MCTSNodeBase>();
    }

    auto current_ucb = children_[0]->ucb(temperature);
    auto current_idx = 0;
    auto dummy_idx = 0;
    for(auto node : children_){

        auto node_ucb = node->ucb(temperature);

        if(node_ucb > current_ucb){

            current_idx = dummy_idx;
            current_ucb = node_ucb;
        }

        dummy_idx += 1;
    }

    return children_[current_idx];
}

template<typename ActionTp, typename StateTp>
real_t
MCTSNodeBase<ActionTp, StateTp>::ucb(real_t temperature )const{
    return win_pct() + temperature * std::sqrt(std::log(parent_ -> total_visits()) / total_visits_);
}
	
}
}
}
}
#endif