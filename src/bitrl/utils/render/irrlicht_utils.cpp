#ifndef IRRLICHT_UTILS_H
#define IRRLICHT_UTILS_H

#include "bitrl/bitrl_config.h"

#if defined(BITRL_IRRLICHT) && defined(BITRL_CHRONO)
#include "bitrl/utils/render/irrlicht_utils.h"
#include "bitrl/bitrl_types.h"
#include <chrono_irrlicht/ChVisualSystemIrrlicht.h>

namespace bitrl
{
namespace utils
{
namespace render
{

void irr_draw_world_axes(chrono::irrlicht::ChVisualSystemIrrlicht& vis,
                     real_t scale = 1.0) {
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

}
}
}

#endif
#endif //IRRLICHT_UTILS_H

