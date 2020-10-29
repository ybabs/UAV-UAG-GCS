#ifndef UTILS_H
#define UTILS_H

#include <cmath>
#include <iostream>
#include <random>



#define PI (double) 3.141592653589793
#define C_EARTH (double)6378137.0

std::default_random_engine& randomEngine();

template <typename T>
T getRandomInt(T first, T second)
{
    std::uniform_int_distribution<T> uniform_dist(first, second);
    return uniform_dist(randomEngine());
}


template <typename T>
T getRandomDouble(T first, T second)
{
    std::uniform_real_distribution<T> uniform_dist(first, second);
    return uniform_dist(randomEngine());
}

template <typename T>
std::vector<T> flatten(const std::vector<std::vector<T>>& v)
{
    std::size_t total_size =0;
    for(const auto &sub :v)
    {
        total_size+=sub.size();
    }
    std::vector<T> result;
    result.reserve(total_size);
    for(const auto& sub: v)
    {
        result.insert(result.end(), sub.begin(), sub.end());
    }
    return result;
}


static double DegToRad(double degree)
{
    return degree * (PI/180.0);
}

static double RadToDeg(double rad)
{
    return rad *  (180.0/PI);
}


#endif //UTILS_H
