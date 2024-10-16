// Copyright (C) 2021 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup     lib_cogip_defs
/// @{
/// @file
/// @brief       Pose class declaration
/// @author      Eric Courtois <eric.courtois@gmail.com>

#pragma once

// System includes
#include <math.h>

// Project includes
#include "cogip_defs/Coords.hpp"
#include "cogip_defs/Polar.hpp"
#include "trigonometry.h"

namespace cogip {

namespace cogip_defs {

/// A robot position
class Pose : public Coords {
public:
    /// Constructor.
    Pose(
        double x=0.0,         ///< [in] X coordinate
        double y=0.0,         ///< [in] Y coordinate
        double O=0.0          ///< [in] 0-orientation
        ) : Coords(x, y), O_(O) {};

    /// Return coordinates.
    Coords coords(void) const { return Coords(x_, y_); };

    /// Set coordinates.
    void set_coords(
        const Coords &coords  ///< [in] new coordinates
        ) { x_ = coords.x(); y_ = coords.y();};

    /// Return 0-orientation.
    double O(void) const { return O_; };

    /// Set 0-orientation.
    void set_O(
        double O              ///< [in] new 0-orientation
        ) { O_ = O; };

    /// Check if this pose is equal to another.
    /// @return true if poses are equal, false otherwise
    bool operator == (
        const Pose other      ///< [in] pose to compare
        ) const { return x_ == other.x_ && y_ == other.y_ && O_ == other.O_; };

    Polar operator-(const Pose& p) {
        double error_x = x_ - p.x();
        double error_y = y_ - p.y();

        double error_O = limit_angle_rad(atan2(error_y, error_x) - DEG2RAD(p.O()));

        return Polar(
            sqrt(square(error_x) + square(error_y)),
            RAD2DEG(error_O)
        );
    };

protected:
    double O_;                ///< 0-orientation
};

} // namespace cogip_defs

} // namespace cogip

/// @}
