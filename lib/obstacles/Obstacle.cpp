#include "obstacles/Obstacle.hpp"

// System includes
#include <cmath>

// Project includes
#include "trigonometry.h"

namespace cogip {

namespace obstacles {

Obstacle::Obstacle(
    const cogip_defs::Coords &center, double radius)
    : center_(center), radius_(radius), enabled_(true)
{
}

void Obstacle::set_center(cogip_defs::Coords &center)
{
    center_ = center;
}

} // namespace obstacles

} // namespace cogip
