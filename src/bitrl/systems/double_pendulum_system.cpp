//
// Created by alex on 11/15/25.
//

#include "bitrl/bitrl_config.h"

#ifdef BITRL_CHRONO

#include "bitrl/bitrl_consts.h"
#include "bitrl/systems/double_pendulum_system.h"

#include <chrono/physics/ChBodyEasy.h>
#include <chrono/physics/ChLinkRevolute.h>

#include <chrono/core/ChQuaternion.h>
#include <chrono/utils/ChUtilsCreators.h>

namespace bitrl
{
namespace systems
{
DoublePendulumSystem::DoublePendulumSystem() : system_() {}

void DoublePendulumSystem::enable_gravity()
{
    system_.SetGravitationalAcceleration(chrono::ChVector3d(0, -9.81, 0));
    gravity_enabled_ = true;
}

void DoublePendulumSystem::build_system(const DoublePendulumProperties &props)
{
    if (is_initialized_)
    {
        throw std::runtime_error("DoublePendulumSystem already initialized");
    }

    // -------------------------
    // Create ground (fixed)
    // -------------------------
    auto ground = chrono_types::make_shared<chrono::ChBody>();
    ground->SetFixed(true);
    ground->SetName("Ground");
    system_.AddBody(ground);

    // -------------------------
    // Create first pendulum body
    // -------------------------
    auto pend1 = chrono_types::make_shared<chrono::ChBodyEasyCylinder>(
        chrono::ChAxis::Z, props.r1, props.l1, props.density, true, true);

    // hanging from origin downward
    pend1->SetPos(chrono::ChVector3d(0.0, 0.0, props.l1 * 0.5));

    {
        // align cylinder along Y
        chrono::ChQuaterniond q;
        chrono::ChVector3d axis(0, 0, 1);
        axis.Normalize();
        q.SetFromAngleAxis(consts::maths::PI * 0.5, axis);
        pend1->SetRot(q);
        pend1->SetName("Pendulum-1");
    }

    system_.AddBody(pend1);

    // -------------------------
    // Create second pendulum body
    // -------------------------
    auto pend2 = chrono_types::make_shared<chrono::ChBodyEasyCylinder>(
        chrono::ChAxis::Z, props.r2, props.l2, props.density, true, true);
    {
        pend2->SetPos(chrono::ChVector3d(0, -props.l1 - props.l2 * 0.5, 0));

        chrono::ChQuaterniond q;
        chrono::ChVector3d axis(0, 0, 1);
        axis.Normalize();
        q.SetFromAngleAxis(consts::maths::PI * 0.5, axis);
        pend2->SetRot(q);
        pend2->SetName("Pendulum-2");
    }
    system_.AddBody(pend2);

    // -------------------------
    // Joint: ground → pendulum 1
    // -------------------------
    auto joint1 = chrono_types::make_shared<chrono::ChLinkRevolute>();
    joint1->Initialize(
        ground, pend1,
        chrono::ChFrame<>(chrono::ChVector3d(0, 0, 0), chrono::QUNIT) // pivot at origin
    );
    system_.AddLink(joint1);

    // -------------------------
    // Joint: pendulum 1 → pendulum 2
    // -------------------------
    auto joint2 = chrono_types::make_shared<chrono::ChLinkRevolute>();
    joint2->Initialize(pend1, pend2,
                       chrono::ChFrame<>(chrono::ChVector3d(0, -props.l1, 0), chrono::QUNIT));
    system_.AddLink(joint2);
    is_initialized_ = true;
}
} // namespace systems
} // namespace bitrl

#endif