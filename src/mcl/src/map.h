#ifndef __MAP_H__
#define __MAP_H__

#include "base.h"
#include <fstream>

typedef struct grid_cell{
    int occupancy; // 0=empty, 1=occupied, -1=unknown
    std::vector<double> expectedRange; // don't have to compute this in realtime when applying sensor model
                                       // Increments of 
    grid_cell()
        : occupancy(-1), expectedRange({}){}
    grid_cell(int occ)
        : occupancy(occ), expectedRange({}){}
    grid_cell(int occ, std::vector<double> exp_ranges)
        : occupancy(occ), expectedRange(exp_ranges){}
} grid_cell_t;

//This class should obtain the occupancy grid from a csv file
class Map{
private:
    std::vector<std::vector<grid_cell_t>> occupancyGridMap;
    double rangeSensorAngleIncrement;
    double rangeSensorMaxRange;
public:
    Map(double raneg_sensor_max_range, double range_sensor_angle_increment);
    Map(double raneg_sensor_max_range, double range_sensor_angle_increment, std::vector<std::vector<grid_cell_t>>& grid) ;
    virtual ~Map();

    void loadMapFromFile(std::string& path_to_file, std::string& file_name, bool find_angular_ranges);

    void computeAngularRanges();

};
#endif // __MAP_H__