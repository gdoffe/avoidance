// Copyright (C) 2021 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup     lib_obstacles
/// @{
/// @file
/// @brief       Rectangle obstacle class declaration
/// @author      Eric Courtois <eric.courtois@gmail.com>

#pragma once

#include "obstacles/ObstaclePolygon.hpp"

namespace cogip {

namespace obstacles {

/// A rectangle obstacle which inherits from Polygon.
/// Rectangle is very similar to Polygon, but it is defined by its center,
/// angle and lengths to be easier to display in the simulator.
class ObstacleRectangle : public ObstaclePolygon {
public:
    /// Constructor
    ObstacleRectangle(
        const cogip_defs::Coords &center, ///< [in] center of the rectangle
        double angle,                     ///< [in] angle of the rectangle
        double length_x,                  ///< [in] length on X of the rectangle
        double length_y                   ///< [in] length on Y of the rectangle
        );

private:
    double angle_;                        ///< angle of the rectangle
    double length_x_;                     ///< length on X axis when angle = 0
    double length_y_;                     ///< length on Y axis when angle = 0
};

} // namespace obstacles

} // namespace cogip

/// @}
