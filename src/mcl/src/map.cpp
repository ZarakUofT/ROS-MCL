#include "map.h"

Map::Map(u_int16_t laser_beams, double range_sensor_max_range, double range_sensor_angle_increment) 
    :   gridCellSize(0.01), laserBeams(laser_beams),
        rangeSensorMaxRange(range_sensor_max_range), 
        rangeSensorAngleIncrement(range_sensor_angle_increment),
        occupancyGridMap({}), mapWidth(0), mapHeight(0),
        stillPlotting(false), threadCreatedForPlotting(false){

    this->init_figure();
}

Map::Map(u_int16_t laser_beams, double range_sensor_max_range, double range_sensor_angle_increment, 
        std::vector<grid_cell_t>& grid, uint map_width, uint map_height, double grid_cell_size) 
    :   gridCellSize(grid_cell_size), laserBeams(laser_beams), 
        rangeSensorMaxRange(range_sensor_max_range), 
        rangeSensorAngleIncrement(range_sensor_angle_increment), 
        occupancyGridMap(grid), mapWidth(map_width), mapHeight(map_height),
        stillPlotting(false), threadCreatedForPlotting(false)
{
    this->init_figure();
}

Map::~Map() {}

void Map::init_figure() 
{
    this->figure = gcf();
    this->figure->ioff();
    this->axes = gca();
    this->axes->grid(false);
}

void Map::loadMapFromFile(std::string& path_to_file, std::string file_name, 
                        bool find_angular_ranges) 
{
    std::string file = path_to_file + "/" + file_name;
    std::ifstream in_file(file);
    
    // Make sure the file is open
    if(!in_file.is_open()) throw std::runtime_error("Could not open file");

    std::string line;
    char c;
    int val;

    // extracting Cell Size from file
    std::getline(in_file, line);
    std::stringstream sss(line);
    sss >> this->gridCellSize >> c >> this->mapWidth >> c >> this->mapHeight;

    while (std::getline(in_file, line)){
        std::stringstream ss(line);

        while (ss >> val){
            this->occupancyGridMap.push_back(grid_cell(val));
            if (ss.peek() == ',') ss.ignore();
        }
    }
    
    if (find_angular_ranges)
        this->computeAngularRanges();

}

inline uint Map::ARR_2D_to_1D(uint i, uint j){
    return (i * this->mapHeight) + j;
}

inline void Map::ARR_1D_to_2D(uint n, uint& row, uint& col){
    row = n / this->mapHeight;
    col = n % this->mapHeight;
}

void Map::lowGradient(int x1, int y1, int x2, int y2, Coordinates2D& res)
{
    int dx = x2 - x1;
    int dy = y2 - y1;
    int yi = 1;
    
    if (dy < 0){
        yi = -1;
        dy = -dy;
    }

    if (dy == 0)
        yi = 0;

    int D = 2 * dy - dx;
    int y = y1;
    int incr = (x1 > x2) ? -1 : 1;

    for(int i = x1; i <= x2;){
        if (this->occupancyGridMap[ARRAY_2D_to_1D(i, y, this->mapHeight)].occupancy == 1){
            res.x = i;
            res.y = y;
            return;
        }

        if (D > 0){
            y = y + yi;
            D = D + (2 * (dy - dx));
        }else{
            D = D + 2*dy;
        }
        i += incr;
    }
}

void Map::highGradient(int x1, int y1, int x2, int y2, Coordinates2D& res)
{
    int dx = x2 - x1;
    int dy = y2 - y1;
    int xi = 1;

    if (dx < 0){
        xi = -1;
        dx = -dx;
    }

    if (dx == 0)
        xi = 0;

    int D = 2 * dx - dy;
    int x = x1;
    int incr = (y1 > y2) ? -1 : 1;

    for(int i = y1; i <= y2;){
        if (this->occupancyGridMap[ARRAY_2D_to_1D(x, i, this->mapHeight)].occupancy == 1){
            res.x = x;
            res.y = i;
            return;
        }
        if (D > 0){
            x = x + xi;
            D = D + (2 * (dx - dy));
        }else{
            D = D + 2*dx;
        }
        i += incr;
    }
}

Coordinates2D Map::bresenhams(int x1, int y1, int x2, int y2) 
{
    Coordinates2D res; // initialized to infinity by default
    if (abs(y2 - y1) < abs(x2 - x1)){
        if (x1 > x2)
            this->lowGradient(x2, y2, x1, y1, res);
        else
            this->lowGradient(x1, y1, x2, y2, res);
    }else{
        if (y1 > y2)
            this->highGradient(x2, y2, x1, y1, res);
        else
            this->highGradient(x1, y1, x2, y2, res);
    }

    return res;
}

double Map::computeEuclidian(uint x1, uint y1, uint x2, uint y2){
    double distance = Math::euclidian_distance(x1, y1, x2, y2);

    return distance * this->gridCellSize;
}

bool Map::computeAngularRangesThread_func(uint start, uint end){
    float phi;
    Coordinates2D coords, position;
    double dist;

    for (int i = start; i < end; i++) {
        if (this->occupancyGridMap[i].occupancy == -1)
            continue;
        phi = 0.0;
        ARRAY_1D_to_2D(i, this->mapHeight, coords.x, coords.y);

        while (phi < DEG2RAD(this->laserBeams)) {
            position = Math::pos_in_2d_array(this->mapWidth, this->mapHeight, coords.x, coords.y, 0.0, phi, 
                this->rangeSensorMaxRange / this->gridCellSize, this->rangeSensorMaxRange / this->gridCellSize);

            position = this->bresenhams(coords.x, coords.y, static_cast<int>(position.x), static_cast<int>(position.y));
            dist = (position.isInfinity()) ? INFINITY : this->computeEuclidian(coords.x, coords.y, position.x, position.y);
            this->occupancyGridMap[i].expectedRange.push_back(static_cast<float>(dist));

            phi += 2 * (this->rangeSensorAngleIncrement);
        }
    }

    return true;
}


void Map::computeAngularRanges() 
{
    auto start_time = std::chrono::system_clock::now();
    
    int num_threads = 8;
    uint increm = this->occupancyGridMap.size() / num_threads;
    uint start = 0, end = increm;

    std::vector<std::thread> threads;
    for (int i = 0; i < num_threads; i++){
        threads.push_back(std::thread(&Map::computeAngularRangesThread_func, this, start, end));
        start = end;
        end += increm;
    }

    for (auto& thread: threads){
        if (thread.joinable())
            thread.join();
    }

    uint64_t secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start_time).count();
    std::cout << "Time Taken to compute Angular Ranges: " << secondsElapsed << std::endl;
}

void Map::update_image()
{
    if (this->threadCreatedForPlotting)
        this->stillPlotting = this->futurePlotThread.wait_for(std::chrono::seconds(0))!=std::future_status::ready;
    if (this->stillPlotting)
        return;

    // Pass vectors of vector of int as data
    std::vector<std::vector<int>> plot_data (this->mapWidth, std::vector<int>(this->mapHeight));

    for(int i = 0; i < this->mapWidth; i++){
        for(int j = 0; j < this->mapHeight; j++){
            plot_data[i][j] = this->occupancyGridMap[ARRAY_2D_to_1D(i, j, this->mapHeight)].occupancy;
        }
    }

    auto plot_data_ptr = std::make_shared<std::vector<std::vector<int>>>(plot_data);

    // Start a new thread
    this->futurePlotThread = std::async(std::launch::async, display_image_thread_func, plot_data_ptr, std::ref(this->axes));
    this->threadCreatedForPlotting = true;

    std::cout << "Continuing to main thread from Plotting thread" << std::endl;
}

bool Map::saveAngularRanges(std::string filename, uint pos_x, uint pos_y){
    std::ofstream outdata;
    const auto& data = this->occupancyGridMap[ARRAY_2D_to_1D(pos_x, pos_y, this->mapHeight)].expectedRange;

    std::cout << "Saving Angular Ranges..." << std::endl;
    
    outdata.open(filename, std::ios::trunc);

    if (!outdata){ // file couldn't be opened
        std::cerr << "Error: file could not be opened" << std::endl;
        return false;
    }

    // saving map pos x and y
    // outdata << pos_x << ", " << pos_y << "\n";

    for(int i = 0;  i < data.size(); i++){
        outdata << data[i] << ",";
    }
    outdata << "\n";

    std::cout << "Saved successfully Angular Ranges..." << std::endl;
    outdata.close();

    return true;
}

/* 
    Non-class Functions
*/
bool display_image_thread_func(std::shared_ptr<std::vector<std::vector<int>>> plot_data, axes_handle& axes) {
    if (!plot_data)
        return false;
    axes->image(*plot_data, true);
    axes->color_box(true);
    axes->draw();

    return true;
}