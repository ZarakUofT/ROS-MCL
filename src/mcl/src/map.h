#ifndef __MAP_H__
#define __MAP_H__

#include "base.h"
#include <fstream>
#include <matplot/matplot.h>

using namespace matplot;

typedef struct grid_cell{
    int8_t occupancy; // 0=empty, 1=occupied, -1=unknown
    std::vector<float> expectedRange; // don't have to compute this in realtime when applying sensor model
                                       // Increments of 2 degrees
    grid_cell()
        : occupancy(-1), expectedRange({}){}
    grid_cell(int occ)
        : occupancy(occ), expectedRange({}){}
    grid_cell(int occ, std::vector<float> exp_ranges)
        : occupancy(occ), expectedRange(exp_ranges){}
} grid_cell_t;

//This class should obtain the occupancy grid from a csv file
class Map{
    friend class Particle;
private:
    std::vector<grid_cell_t> occupancyGridMap;
    uint mapWidth, mapHeight;
    uint16_t laserBeams;
    double gridCellSize;
    double rangeSensorAngleIncrement;
    double rangeSensorMaxRange;

    figure_handle figure;
    axes_handle axes;

    // Held for plotting for graph on a different thread
    std::future<bool> futurePlotThread;
    bool threadCreatedForPlotting;
    bool stillPlotting;

    /* 
        Private Functions
    */
    void init_figure();
    void lowGradient(int x1, int y1, int x2, int y2, Coordinates2D& res);
    void highGradient(int x1, int y1, int x2, int y2, Coordinates2D& res);
    Coordinates2D bresenhams(int x1, int y1, int x2, int y2); 
    double computeEuclidian(uint x1, uint y1, uint x2, uint y2);

    // thread functions private
    bool computeAngularRangesThread_func(uint start, uint end);

public:
    Map(uint16_t laser_beams, double range_sensor_max_range, double range_sensor_angle_increment);
    Map(uint16_t laser_beams, double range_sensor_max_range, double range_sensor_angle_increment, 
        std::vector<grid_cell_t>& grid, uint map_width, uint map_height, double grid_cell_size);
    
    virtual ~Map();

    void loadMapFromFile(std::string& path_to_file, std::string file_name, bool find_angular_ranges=false);

    inline uint ARR_2D_to_1D(uint i, uint j);
    inline void ARR_1D_to_2D(uint n, uint& row, uint& col);

    void computeAngularRanges();

    void update_image();
    bool saveAngularRanges(std::string filename, uint pos_x, uint pos_y);
};

// Prototypes for functions

// This function is called in a different thread to plot heatmap for probability of 
// occupancy of the map
bool display_image_thread_func(std::shared_ptr<std::vector<std::vector<int>>> plot_data, axes_handle& axes);

#endif // __MAP_H__`