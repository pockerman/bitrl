#ifndef MESH_ENTITY_H
#define MESH_ENTITY_H

#include "bitrl/bitrl_consts.h"
#include "bitrl/bitrl_types.h"

#include <any>
#include <type_traits>

namespace bitrl
{
namespace utils
{
namespace geom
{

///
/// \brief Helper class that wraps non template dependent
/// parameters and common to every mesh entity
///

class MeshEntity
{

  public:
    ///
    /// \brief Constructor
    ///
    MeshEntity();

    ///
    /// \brief Constructor
    ///
    MeshEntity(uint_t id, uint_t pid, const std::any &data);

    ///
    /// \brief Returns the id of the element
    ///
    uint_t get_id() const { return id_; }

    ///
    /// \brief Set the id of the element
    ///
    void set_id(uint_t id) { id_ = id; }

    ///
    /// \brief Returns the id of the element
    ///
    uint_t get_pid() const { return pid_; }

    ///
    /// \brief Set the id of the element
    ///
    void set_pid(uint_t id) { pid_ = id; }

    ///
    /// \brief
    ///
    bool has_valid_id() const { return true; }

    ///
    /// \brief
    ///
    bool is_active() const { return is_active_; }

    ///
    /// \brief
    ///
    void set_active_flag(bool f) { is_active_ = f; }

  private:
    /// \brief The id of the element
    uint_t id_;

    /// \brief The processor id the element belongs
    uint_t pid_;

    /// \brief flag indicating if the entity is active
    bool is_active_;

    ///
    /// \brief The data held by the the entity
    ///
    std::any entity_data_;
};

inline MeshEntity::MeshEntity()
    : id_(bitrl::consts::INVALID_ID), pid_(bitrl::consts::INVALID_ID), is_active_(true),
      entity_data_()
{
}

inline MeshEntity::MeshEntity(uint_t id, uint_t pid, const std::any &data)
    : id_(id), pid_(pid), is_active_(true), entity_data_(data)
{
}

} // namespace geom
} // namespace utils
} // namespace bitrl
#endif // MESH_ENTITY_H
