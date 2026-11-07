#ifndef IRRLICHT_UTILS_H
#define IRRLICHT_UTILS_H

#include "bitrl/bitrl_config.h"

#if defined(BITRL_IRRLICHT) && defined(BITRL_CHRONO)
#include "bitrl/bitrl_types.h"
#include <chrono/core/ChFrame.h>
#include <chrono_irrlicht/ChVisualSystemIrrlicht.h>

#include <string>

namespace bitrl
{
namespace utils
{
namespace render
{

///
/// Draw the global axis on  the given ChVisualSystemIrrlicht system
///
void irr_draw_world_axes(chrono::irrlicht::ChVisualSystemIrrlicht& vis, real_t scale = 1.0);

///
/// Draw a line in world coordinates. The line is drawn on top of the scene, so
/// that bodies do not hide it. Call this between Render() and EndScene()
///
void irr_draw_line(chrono::irrlicht::ChVisualSystemIrrlicht& vis, const chrono::ChVector3d& start,
                   const chrono::ChVector3d& end, const chrono::ChColor& color,
                   real_t thickness = 2.0);

///
/// Draw a label at a point given in world coordinates. Unlike
/// chrono::irrlicht::tools::drawLabel3D this does not need the Chrono data
/// directory to be available
///
void irr_draw_label(chrono::irrlicht::ChVisualSystemIrrlicht& vis, const std::string& text,
                    const chrono::ChVector3d& position,
                    const chrono::ChColor& color = chrono::ChColor(1.0f, 1.0f, 1.0f));

///
/// Draw the axes of the given frame with x in red, y in green and z in blue
/// and write the label at the origin of the frame
///
void irr_draw_frame(chrono::irrlicht::ChVisualSystemIrrlicht& vis, const chrono::ChFrame<>& frame,
                    real_t scale = 1.0, const std::string& label = "");

}
}
}

#endif
#endif //IRRLICHT_UTILS_H
