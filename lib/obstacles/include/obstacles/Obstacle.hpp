// Copyright (C) 2021 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup     lib_obstacles
/// @{
/// @file
/// @brief       Polygon obstacle class declaration
/// @author      Eric Courtois <eric.courtois@gmail.com>

#pragma once

// System includes
#include <cstdint>

// Project includes
#include "cogip_defs/Coords.hpp"
#include "cogip_defs/Polygon.hpp"

#ifndef OBSTACLE_BOUNDING_BOX_VERTICES
#  define OBSTACLE_BOUNDING_BOX_VERTICES    6  /**< number of bounding box vertices */
#endif

#ifndef OBSTACLE_BOUNDING_BOX_MARGIN
#  define OBSTACLE_BOUNDING_BOX_MARGIN    0.2  /**< bounding box margin in percent of the radius */
#endif

namespace cogip {

namespace obstacles {

using BoundingBox = cogip_defs::Polygon;

/// An obstacle used to detect and avoid collisions.
class Obstacle : public cogip_defs::Polygon  {
public:
    /// Constructor
    Obstacle(
        const cogip_defs::Coords &center = {0, 0}, ///< [in] obstacle center
        double radius = 0.0                        ///< [in] obstacle circumscribed circle radius
        );

    /// Destructor
    virtual ~Obstacle() {};

    /// Check if the given point is inside the obstacle.
    /// @return true if point is inside, false otherwise
    virtual bool is_point_inside(
        const cogip_defs::Coords &p       ///< [in] point to check
        ) const = 0;

    /// Check if a segment defined by two points A,B is crossing an obstacle.
    /// @return true if [AB] crosses obstacle, false otherwise
    virtual bool is_segment_crossing(
        const cogip_defs::Coords &a,      ///< [in] point A
        const cogip_defs::Coords &b       ///< [in] point B
        ) const = 0;

    /// Find the nearest point of obstacle perimeter from given point.
    /// @return position of nearest point
    virtual cogip_defs::Coords nearest_point(
        const cogip_defs::Coords &p       ///< [in] point to check
        ) const = 0;

    /// Return obstacle center.
    const cogip_defs::Coords &center() const { return center_; };

    /// Set obstacle center.
    void set_center(cogip_defs::Coords &center);

    /// Return obstacle circumscribed circle radius.
    double radius() const { return radius_; };

    /// Return true if obstacle is enabled, false otherwise.
    bool enabled() const { return enabled_; };

    /// Enable or disable obstacle.
    void enable(bool enabled) { enabled_ = enabled; };

protected:
    cogip_defs::Coords center_;           ///< obstacle center
    double radius_;                       ///< obstacle circumscribed circle radius
    BoundingBox bounding_box_;            ///< Precomputed bounding box for avoidance
    bool enabled_;                        ///< Obstacle enabled or not
};

} // namespace obstacles

} // namespace cogip

/// @}
