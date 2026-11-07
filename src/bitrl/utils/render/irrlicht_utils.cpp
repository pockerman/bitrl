#include "bitrl/bitrl_config.h"

#if defined(BITRL_IRRLICHT) && defined(BITRL_CHRONO)
#include "bitrl/utils/render/irrlicht_utils.h"
#include "bitrl/bitrl_types.h"
#include <chrono/core/ChFrame.h>
#include <chrono_irrlicht/ChIrrTools.h>
#include <chrono_irrlicht/ChVisualSystemIrrlicht.h>

#include <string>

namespace bitrl
{
namespace utils
{
namespace render
{

void irr_draw_world_axes(chrono::irrlicht::ChVisualSystemIrrlicht& vis,
                     real_t scale) {
    auto* driver = vis.GetVideoDriver();

    // X axis (red)
    driver->draw3DLine(
        {0, 0, 0},
        {static_cast<float>(scale), 0, 0},
        irr::video::SColor(255, 255, 0, 0));

    // Y axis (green)
    driver->draw3DLine(
        {0, 0, 0},
        {0, static_cast<float>(scale), 0},
        irr::video::SColor(255, 0, 255, 0));

    // Z axis (blue)
    driver->draw3DLine(
        {0, 0, 0},
        {0, 0, static_cast<float>(scale)},
        irr::video::SColor(255, 0, 0, 255));
}

void irr_draw_line(chrono::irrlicht::ChVisualSystemIrrlicht& vis, const chrono::ChVector3d& start,
                   const chrono::ChVector3d& end, const chrono::ChColor& color, real_t thickness)
{
    auto* driver = vis.GetVideoDriver();
    driver->setTransform(irr::video::ETS_WORLD, irr::core::matrix4());

    irr::video::SMaterial material;
    material.ZBuffer = false;
    material.Lighting = false;
    material.Thickness = static_cast<float>(thickness);
    driver->setMaterial(material);

    driver->draw3DLine(irr::core::vector3df(start.x(), start.y(), start.z()),
                       irr::core::vector3df(end.x(), end.y(), end.z()),
                       chrono::irrlicht::tools::ToIrrlichtSColor(color));
}

void irr_draw_label(chrono::irrlicht::ChVisualSystemIrrlicht& vis, const std::string& text,
                    const chrono::ChVector3d& position, const chrono::ChColor& color)
{
    const irr::core::position2di on_screen =
        vis.GetSceneManager()->getSceneCollisionManager()->getScreenCoordinatesFrom3DPosition(
            irr::core::vector3df(position.x(), position.y(), position.z()));

    vis.GetGUIEnvironment()->getBuiltInFont()->draw(
        irr::core::stringw(text.c_str()),
        irr::core::rect<irr::s32>(on_screen.X, on_screen.Y, on_screen.X + 150, on_screen.Y + 15),
        chrono::irrlicht::tools::ToIrrlichtSColor(color));
}

void irr_draw_frame(chrono::irrlicht::ChVisualSystemIrrlicht& vis, const chrono::ChFrame<>& frame,
                    real_t scale, const std::string& label)
{
    const auto& origin = frame.GetPos();
    const auto& R = frame.GetRotMat();

    irr_draw_line(vis, origin, origin + R.GetAxisX() * scale, chrono::ChColor(1.0f, 0.0f, 0.0f));
    irr_draw_line(vis, origin, origin + R.GetAxisY() * scale, chrono::ChColor(0.0f, 1.0f, 0.0f));
    irr_draw_line(vis, origin, origin + R.GetAxisZ() * scale, chrono::ChColor(0.2f, 0.5f, 1.0f));

    if (!label.empty())
    {
        irr_draw_label(vis, label, origin - chrono::ChVector3d(0.0, 0.0, 0.08));
    }
}

}
}
}

#endif

