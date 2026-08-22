#pragma once

/* Includes ----------------------------------------------------------------*/

#include <vector>
#include "mini_nav_core/costmap_2d.hpp"

/* Namespace ---------------------------------------------------------------*/
namespace mini_nav_core
{

/* Structs definition -------------------------------------------------------*/
    struct OpenNode
    {
        unsigned int index;
        unsigned int f_score;
    };

    struct CompareOpenNode
    {
        bool operator()(const OpenNode & left, const OpenNode & right) const
        {
            return left.f_score > right.f_score;
        }
    };

/* Class definition --------------------------------------------------------*/
    class AStarPlanner
    {
    public:
        std::vector<MapLocation> Plan(
            const Costmap2D   & costmap,
            const MapLocation & start,      
            const MapLocation & goal) const;

    private:
        unsigned int GetIndex(unsigned int mx, unsigned int my, unsigned int size_x) const;
        unsigned int ManhattanDistance(const MapLocation & from, const MapLocation & to) const;
    };

/* Local constants ---------------------------------------------------------*/
    constexpr unsigned char kLethalObstacle = 254;
}