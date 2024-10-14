#include <iostream>
#include <vector>
#include "avoidance/Avoidance.hpp"  // Include the Avoidance class and other dependencies
#include <cmath> // For calculating hexagon and octagon points

int main() {
    // Create a fictitious area for the obstacle borders (size of the table 2000mm x 3000mm)
    std::vector<cogip::cogip_defs::Coords> bordersPoints = {
        cogip::cogip_defs::Coords(0, 0),
        cogip::cogip_defs::Coords(2000, 0),
        cogip::cogip_defs::Coords(2000, 3000),
        cogip::cogip_defs::Coords(0, 3000)
    };
    cogip::obstacles::ObstaclePolygon borders(bordersPoints);  // Table borders
    Avoidance avoidance(borders);  // Create the Avoidance object with the borders

    // Create a square of 100mm side length
    cogip::obstacles::ObstaclePolygon square({
        cogip::cogip_defs::Coords(500, 500),
        cogip::cogip_defs::Coords(600, 500),
        cogip::cogip_defs::Coords(600, 600),
        cogip::cogip_defs::Coords(500, 600)
    });

    // Create a regular hexagon with a 100mm radius
    std::vector<cogip::cogip_defs::Coords> hexagon;
    double hexRadius = 100;
    double hexCenterX = 1000, hexCenterY = 1000;
    for (int i = 0; i < 6; ++i) {
        double angle = M_PI / 3 * i;
        hexagon.push_back(cogip::cogip_defs::Coords(
            hexCenterX + hexRadius * std::cos(angle),
            hexCenterY + hexRadius * std::sin(angle)
        ));
    }
    cogip::obstacles::ObstaclePolygon hexagonObstacle(hexagon);

    // Create a regular octagon with a 100mm radius
    std::vector<cogip::cogip_defs::Coords> octagon;
    double octRadius = 100;
    double octCenterX = 1000, octCenterY = 1500;
    for (int i = 0; i < 8; ++i) {
        double angle = M_PI / 4 * i;
        octagon.push_back(cogip::cogip_defs::Coords(
            octCenterX + octRadius * std::cos(angle),
            octCenterY + octRadius * std::sin(angle)
        ));
    }
    cogip::obstacles::ObstaclePolygon octagonObstacle(octagon);

    // Adding dynamic obstacles
    std::cout << "Adding a square, hexagon, and octagon..." << std::endl;
    avoidance.addDynamicObstacle(square);
    avoidance.addDynamicObstacle(hexagonObstacle);
    avoidance.addDynamicObstacle(octagonObstacle);

    // Test the buildGraph() method
    cogip::cogip_defs::Coords start(250, 250);   // Example start position
    cogip::cogip_defs::Coords finish(1750, 2750); // Example finish position
    std::cout << "Building the avoidance graph..." << std::endl;
    bool graphBuilt = avoidance.buildGraph(start, finish);
    if (graphBuilt) {
        std::cout << "Graph built successfully!" << std::endl;
    } else {
        std::cout << "Failed to build the graph." << std::endl;
    }

    // Removing the hexagon
    std::cout << "Removing the hexagon..." << std::endl;
    avoidance.removeDynamicObstacle(hexagonObstacle);

    // Clearing the dynamic obstacles list
    std::cout << "Resetting dynamic obstacles..." << std::endl;
    avoidance.clearDynamicObstacles();

    std::cout << "Test complete." << std::endl;

    return 0;
}
