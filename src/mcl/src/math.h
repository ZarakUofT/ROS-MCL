#ifndef MATH_H
#define MATH_H

#include <iostream>
#include <chrono>
#include <vector>
#include <deque>
#include <math.h>
#include <random>
#include <limits>
#include <limits.h>
#include <algorithm>
#include <memory>
#include <future>

#define RAD2DEG(rad)((rad) * 180 / M_PI)
#define DEG2RAD(deg)((deg) * M_PI/180)
// #define ARRAY_2D_to_1D(i, j, mcol)((i * mcol) + j)
// #define ARRAY_1D_to_2D(i, mcol, row, col) {row = i / mcol; col = i % mcol;}

struct Coordinates2D{
    uint x, y;

    Coordinates2D()
        :x(UINT_MAX), y(UINT_MAX)   {}

    Coordinates2D(uint xx, uint yy)
        :x(xx), y(yy)   {}

    bool isInfinity(){
        return this->x == UINT_MAX && this->y == UINT_MAX;
    }       
};

namespace Math
{

//Make sure to only pass in Numeric Types to this
//This does not support string literals
//For straight lines, this should return infinity
//We return double in order to handle the case of ints being passed
template<typename T>
double compute_slope(T x1, T y1, T x2, T y2)
{
    if (y2 - y1 == 0.)
        return 0.;

    //Check for division by zero
    if ((x2 - x1) == 0.){
        (y2 > y1) ? std::numeric_limits<double>::infinity(): -std::numeric_limits<double>::infinity();
    }else{
        return (y2 - y1) / (x2 - x1);
    }

    return 0.;  
}

// compute Euclidian distance between two points
template<typename T>
double euclidian_distance(T x1, T y1, T x2, T y2)
{
    return sqrt(((x2 - x1) * (x2 - x1))  + ((y2 - y1) * (y2 - y1)));
}

void lowGradient(int x1, int y1, int x2, int y2, std::vector<Coordinates2D>& res)
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
        res.push_back(Coordinates2D(i, y));
        if (D > 0){
            y = y + yi;
            D = D + (2 * (dy - dx));
        }else{
            D = D + 2*dy;
        }
        i += incr;
    }
}

void highGradient(int x1, int y1, int x2, int y2, std::vector<Coordinates2D>& res)
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
        res.push_back(Coordinates2D(x, i));
        if (D > 0){
            x = x + xi;
            D = D + (2 * (dx - dy));
        }else{
            D = D + 2*dx;
        }
        i += incr;
    }
}

//This function takes in integer coordinates and returns a vector of pairs containing x and y coordinates of the line
//joining the two points
std::vector<Coordinates2D> bresenhamsAlgo(int x1, int y1, int x2, int y2) 
{
    std::vector<Coordinates2D> res;
    if (abs(y2 - y1) < abs(x2 - x1)){
        if (x1 > x2)
            lowGradient(x2, y2, x1, y1, res);
        else
            lowGradient(x1, y1, x2, y2, res);
    }else{
        if (y1 > y2)
            highGradient(x2, y2, x1, y1, res);
        else
            highGradient(x1, y1, x2, y2, res);
    }

    //Make sure it's returned in the right format
    if (res[0].x != x1 && res[0].y != x2)
        std::reverse(res.begin(), res.end());

    return res;
}

// Determine the destination pos in a 2D array given the orientation and range value
Coordinates2D pos_in_2d_array(const uint map_width, const uint map_height, 
    const uint pos_x, const uint pos_y, 
    const float yaw, const float phi, float range, const float r_max)
{
    if (!map_width || !map_height)
        return Coordinates2D(0, 0);

    int row = pos_x, col = pos_y;
    if (range > r_max){
        range = r_max;
    }
    
    row += round(range * (float)cos((double)(yaw + phi))); 
    col += round(range * (float)sin((double)(yaw + phi))); 

    if (row < 0)
        row = 0;
    else if (row > map_width - 1)
        row = map_width - 1;
    
    if (col < 0)
        col = 0;
    else if (col > map_height - 1)
        col = map_height - 1;
    
    return Coordinates2D((uint)row, (uint)col);
}

//Converts probability into logit function
inline float logit(float p)
{
    return log(p / (1 - p));
}

//Converts logit back to probability
inline float logit_inverse(float l)
{
    return exp(l) / (1 + exp(l));
}

// returns a vector of n evenly spaced point 
std::vector<float> linspace(const float x1, const float x2, const int n)
{
    if (n <= 0)
        return std::vector<float>(0);
   
    std::vector<float> retval (n);

    float diff = (x2 - x1) / (n - 1);
    float val = x1;
    for (int i = 0; i < n; i++){
        retval[i] = val;
        val += diff;
    }
    return retval;
}

// this resizes the passed vector and keeps ints current contents in the center
// resizes the front and back by the given amount
template<typename T>
bool resizeDeq(std::deque<std::deque<T>>& deq, uint16_t resize_by_width, uint16_t resize_by_height) 
{
    if (deq.empty())
        deq = {{T()}};
    // insert at the front
    deq.insert(deq.begin(), resize_by_width, std::deque<T>(deq[0].size()));
    deq.insert(deq.end(), resize_by_width, std::deque<T>(deq[0].size()));
    for (int i = 0; i < deq.size(); i++){
        deq[i].insert(deq[i].begin(), resize_by_height, T());
    }
    for (int i = 0; i < deq.size(); i++){
        deq[i].insert(deq[i].end(), resize_by_height, T());
    }

    return true;
}

inline double normalize(double z)
{
  return atan2(sin(z),cos(z));
}

template<typename T>
double sum(std::vector<T>& vec){
    double sum = 0.0;
    for (auto& e: vec){
        sum += e;
    }
    return sum;
}

template<typename T>
bool scale(std::vector<T>& vec, double scale){
    for (auto& e: vec){
        e *= scale;
    }
    return true;
}

template<typename T>
bool normalize_vec(std::vector<T>& vec){
    double summ = sum(vec);
    if (summ <= 1)
        return false;
    return scale(vec, 1 / summ);
}


// c is assumed to be empty
template<typename T>
void cumalitive_vector(std::vector<T>& vec, std::vector<T>& c){
    c.reserve(vec.size());
    double n = 0.f;
    for (int i = 0; i < vec.size(); i++){
        n += vec[i];
        c.push_back(n);
    }
}

// Angle difference in radians
double angle_diff(double a, double b)
{
  double d1, d2;
  a = normalize(a);
  b = normalize(b);
  d1 = a - b;
  d2 = 2 * M_PI - fabs(d1);
  if(d1 > 0)
    d2 *= -1.0;
  if(fabs(d1) < fabs(d2))
    return(d1);
  else
    return(d2);
}

// Draw randomly from a zero-mean Gaussian distribution, with standard
// deviation sigma.
// We use the polar form of the Box-Muller transformation, explained here:
//   http://www.taygeta.com/random/gaussian.html
double sample_gaussian_dist(double sigma)
{
  double x1, x2, w, r;

  do
  {
    do { r = drand48(); } while (r==0.0);
    x1 = 2.0 * r - 1.0;
    do { r = drand48(); } while (r==0.0);
    x2 = 2.0 * r - 1.0;
    w = x1*x1 + x2*x2;
  } while(w > 1.0 || w==0.0);

  return(sigma * x2 * sqrt(-2.0*log(w)/w));
}

double sample_normal_dist(double mean, double sigma)
{

     // random device class instance, source of 'true' randomness for initializing random seed
    std::random_device rd; 

    // Mersenne twister PRNG, initialized with seed from previous random device instance
    std::mt19937 gen(rd()); 
    
    // instance of class std::normal_distribution with specific mean and stddev
    std::normal_distribution<float> d(mean, sigma); 

    // get random number with normal distribution using gen as random source
    return d(gen);
}

template<typename T>
void random_integers_in_range(T r1, T r2, T n,std::vector<T>& nums)
{   
    std::random_device rd; // obtain a random number from hardware
    std::mt19937 gen(rd()); // seed the generator
    std::uniform_int_distribution<> distr(r1, r2); // define the range

    for (int i=0; i < n; i++){
        nums.push_back(distr(gen));
    }   
}

inline double random_double(double x, double y){
    return x + static_cast <double> (rand()) /( static_cast <double> (RAND_MAX/(y-x)));
}

template<typename T>
static inline double model_normal_distribution(T x, T mean, T sigma)
{
    return (1 / sqrt(2 * M_PI * sigma * sigma)) * exp(-(pow((x - mean), 2)) / (2 * sigma * sigma));
}

}

#endif
