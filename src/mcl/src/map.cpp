#include "map.h"

Map::Map(double raneg_sensor_max_range, double range_sensor_angle_increment) 
    : rangeSensorMaxRange(raneg_sensor_max_range), 
        rangeSensorAngleIncrement(range_sensor_angle_increment), 
        occupancyGridMap({}){}

Map::Map(double raneg_sensor_max_range, double range_sensor_angle_increment, 
        std::vector<std::vector<grid_cell_t>>& grid) 
    : rangeSensorMaxRange(raneg_sensor_max_range), 
        rangeSensorAngleIncrement(range_sensor_angle_increment), 
        occupancyGridMap(grid){}

Map::~Map() {}

void Map::loadMapFromFile(std::string& path_to_file, std::string& file_name, 
                        bool find_angular_ranges) 
{
    std::string file = path_to_file + "/" + file_name;
    std::ifstream in_file(file);
    
    // Make sure the file is open
    if(!in_file.is_open()) throw std::runtime_error("Could not open file");

    std::string line;
    int val;

    while (std::getline(in_file, line)){
        std::stringstream ss(line);
        std::vector<grid_cell_t> vals;

        while (ss >> val){
            vals.push_back(grid_cell(val));
            if (ss.peek() == ',') ss.ignore();
        }
        this->occupancyGridMap.push_back(vals);
    }

    if (find_angular_ranges)
        this->computeAngularRanges();

}

void Map::computeAngularRanges() 
{
    for (int i = 0; i < this->occupancyGridMap.size(); i++){
        for (int j = 0; j < this->occupancyGridMap[0].size(); j++){
            if (this->occupancyGridMap[i][j].occupancy == -1)
                continue;
        }
    }
}
