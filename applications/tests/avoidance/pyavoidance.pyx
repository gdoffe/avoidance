# pyavoidance.pyx

# cython: profile=True

from libcpp.vector cimport vector
from libcpp cimport bool
from cython.operator cimport dereference as deref

cdef extern from "avoidance/Avoidance.hpp" namespace "":
    cdef cppclass Avoidance:
        Avoidance(const ObstaclePolygon& borders)
        size_t getPathSize()
        Coords getPathPose(unsigned char index) const
        void addDynamicObstacle(Obstacle& obstacle)
        void removeDynamicObstacle(Obstacle& obstacle)
        void clearDynamicObstacles()
        bool buildGraph(const Coords& start, const Coords& finish)

cdef extern from "obstacles/Obstacle.hpp" namespace "cogip::obstacles":
    cdef cppclass Obstacle:
        Obstacle()  # Constructor
        bool is_point_inside(const Coords& p) const
        bool is_segment_crossing(const Coords& a, const Coords& b) const
        Coords nearest_point(const Coords& p) const

cdef extern from "obstacles/ObstaclePolygon.hpp" namespace "cogip::obstacles":
    cdef cppclass ObstaclePolygon(Obstacle):
        ObstaclePolygon()  # Default constructor
        ObstaclePolygon(const vector[Coords]& points)

cdef extern from "cogip_defs/Coords.hpp" namespace "cogip::cogip_defs":
    cdef cppclass Coords:
        Coords()
        Coords(double x, double y)
        double x() const
        double y() const

# Wrapping Obstacle class for Python
cdef class PyObstacle:
    cdef Obstacle* c_obstacle

    def __cinit__(self):
        pass

    def __dealloc__(self):
        del self.c_obstacle

    def is_point_inside(self, double x, double y):
        """
        Check if a point (x, y) is inside the obstacle.
        """
        cdef Coords p = Coords(x, y)
        return self.c_obstacle.is_point_inside(p)

    def nearest_point(self, double x, double y):
        """
        Get the nearest point on the obstacle to the given point (x, y).
        """
        cdef Coords p = Coords(x, y)
        cdef Coords nearest = self.c_obstacle.nearest_point(p)
        return (nearest.x(), nearest.y())

# Wrapping ObstaclePolygon for Python
cdef class PyObstaclePolygon(PyObstacle):
    cdef ObstaclePolygon* c_obstacle_polygon

    def __cinit__(self, list points):
        """
        Initialize an obstacle polygon with a list of points (tuples of x, y).
        """
        cdef vector[Coords] polygon_points
        for point in points:
            x, y = point
            polygon_points.push_back(Coords(x, y))
        self.c_obstacle_polygon = new ObstaclePolygon(polygon_points)
        self.c_obstacle = self.c_obstacle_polygon

    def is_point_inside(self, double x, double y):
        """
        Check if a point (x, y) is inside the polygon.
        """
        return super().is_point_inside(x, y)

    def nearest_point(self, double x, double y):
        """
        Get the nearest point on the polygon to the given point (x, y).
        """
        return super().nearest_point(x, y)

# Wrapping Avoidance class for Python
cdef class PyAvoidance:
    cdef Avoidance* c_avoidance

    def __cinit__(self, PyObstaclePolygon borders):
        """
        Initialize Avoidance object with the borders of the area.
        """
        self.c_avoidance = new Avoidance(deref(borders.c_obstacle_polygon))

    def __dealloc__(self):
        del self.c_avoidance

    def get_path_size(self):
        """
        Get computed avoidance path size including start and stop pose.
        """
        return self.c_avoidance.getPathSize()

    def get_path_pose(self, unsigned int index):
        """
        Get pose in computed avoidance path.
        """
        cdef Coords pose = self.c_avoidance.getPathPose(index)
        return (pose.x(), pose.y())

    def add_dynamic_obstacle(self, PyObstacle obstacle):
        """
        Add a dynamic obstacle to the avoidance system.
        """
        self.c_avoidance.addDynamicObstacle(deref(obstacle.c_obstacle))

    def remove_dynamic_obstacle(self, PyObstacle obstacle):
        """
        Remove a dynamic obstacle from the avoidance system.
        """
        self.c_avoidance.removeDynamicObstacle(deref(obstacle.c_obstacle))

    def clear_dynamic_obstacles(self):
        """
        Clear all dynamic obstacles from the avoidance system.
        """
        self.c_avoidance.clearDynamicObstacles()

    def build_graph(self, double start_x, double start_y, double finish_x, double finish_y):
        """
        Build a graph between the start and finish points in the avoidance area.
        """
        cdef Coords start = Coords(start_x, start_y)
        cdef Coords finish = Coords(finish_x, finish_y)
        return self.c_avoidance.buildGraph(start, finish)
