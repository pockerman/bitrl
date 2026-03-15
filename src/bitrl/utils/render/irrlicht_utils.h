#ifndef IRRLICHT_UTILS_H
#define IRRLICHT_UTILS_H

#include "bitrl/bitrl_config.h"

#if defined(BITRL_IRRLICHT) && defined(BITRL_CHRONO)
#include "bitrl/bitrl_types.h"
#include <chrono_irrlicht/ChVisualSystemIrrlicht.h>

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

}
}
}

#endif
#endif //IRRLICHT_UTILS_H
