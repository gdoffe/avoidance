// Project includes
#include "obstacles/ObstacleCircle.hpp"
#include "utils.hpp"

// System includes
#include <cmath>

namespace cogip {

namespace obstacles {

ObstacleCircle::ObstacleCircle(const cogip_defs::Coords &center, double radius)
    : Obstacle(center, radius)
{
    update_bounding_box();
}

bool ObstacleCircle::is_point_inside(const cogip_defs::Coords &p) const {
    double d = center_.distance(p);

    if (d * d > radius_ * radius_) {
        return false;
    }
    else {
        return true;
    }
}

bool ObstacleCircle::is_segment_crossing(const cogip_defs::Coords &a, const cogip_defs::Coords &b) const
{
    const cogip_defs::Coords &c = center_;

    if (!is_line_crossing_circle(a, b)) {
        return false;
    }

    if (is_point_inside(a)) {
        return true;
    }
    if (is_point_inside(b)) {
        return true;
    }

    cogip_defs::Coords vect_ab(b.x() - a.x(), b.y() - a.y());
    cogip_defs::Coords vect_ac(c.x() - a.x(), c.y() - a.y());
    cogip_defs::Coords vect_bc(c.x() - b.x(), c.y() - b.y());

    double scal1 = vect_ab.x() * vect_ac.x() + vect_ab.y() * vect_ac.y();
    double scal2 = (-vect_ab.x()) * vect_bc.x() + (-vect_ab.y()) * vect_bc.y();
    if (scal1 >= 0 && scal2 >= 0) {
        return true;
    }

    return false;
}

cogip_defs::Coords ObstacleCircle::nearest_point(const cogip_defs::Coords &p) const
{
    cogip_defs::Coords vect(
        p.x() - center_.x(),
        p.y() - center_.y()
    );

    double vect_norm = sqrt(vect.x() * vect.x() + vect.y() * vect.y());

    return cogip_defs::Coords(
        center_.x() + (vect.x() / vect_norm) * radius_,
        center_.y() + (vect.y() / vect_norm) * radius_
    );
}

bool ObstacleCircle::is_line_crossing_circle(const cogip_defs::Coords &a, const cogip_defs::Coords &b) const
{
    const cogip_defs::Coords &c = center_;

    cogip_defs::Coords vect_ab(b.x() - a.x(), b.y() - a.y());
    cogip_defs::Coords vect_ac(c.x() - a.x(), c.y() - a.y());

    // Norm of vector V
    double numerator = vect_ab.x() * vect_ac.y() - vect_ab.y() * vect_ac.x();
    if (numerator < 0) {
        numerator = -numerator;
    }

    // Norm of vector U
    double denominator = sqrt(vect_ab.x() * vect_ab.x() + vect_ab.y() * vect_ab.y());

    // Norm of vector CI where I is the nearest point of the line
    double ci = numerator / denominator;

    // If CI norm is less or equal to the circle radius, point I is inside the
    // circle
    if (ci < radius_) {
        return true;
    }
    else {
        return false;
    }
}

void ObstacleCircle::update_bounding_box()
{
    if (radius_) {
        double radius = radius_ * (1 + OBSTACLE_BOUNDING_BOX_MARGIN);

        clear();

        for (uint8_t i = 0; i < OBSTACLE_BOUNDING_BOX_VERTICES; i++) {
            push_back(cogip_defs::Coords(
                center_.x() + radius * cos(((double)i * 2 * M_PI) / (double)OBSTACLE_BOUNDING_BOX_VERTICES),
                center_.y() + radius * sin(((double)i * 2 * M_PI) / (double)OBSTACLE_BOUNDING_BOX_VERTICES)));
        }
    }
}

} // namespace obstacles

} // namespace cogip
