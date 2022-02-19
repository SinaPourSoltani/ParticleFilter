//
// Created by Sina P. Soltani on 20/11/2019.
//

#ifndef PARTICLEFILTER_PARTICLE_H
#define PARTICLEFILTER_PARTICLE_H

#include <cmath>
#include <iostream>

#define FOV (2.27 - (-2.27))
#define RANDOM_CHANGE_IN_VISION_RAY 6
#define NUM_VISION_DATA_PTS 54
#define NUM_LIDAR_DATA_PTS 39
#define ANGULAR_PREC (float) (FOV / NUM_LIDAR_DATA_PTS)
#define RAY_LENGTH 100
#define NUM_PARTICLES 2000
#define DELTA_T (float) (1/10)
#define MAP_TO_SIM_SCLR 10
#define ETA_LOWER_BOUND (NUM_PARTICLES * NUM_LIDAR_DATA_PTS * 0.3)
#define ETA_UPPER_BOUND (NUM_PARTICLES * NUM_LIDAR_DATA_PTS * 0.6)

using namespace std;

class Particle {
public:
    Particle();
    Particle(int _x, int _y, int _indexOfRightmostRay);

    void calculateOrientation();

    void setPosX(int _x);
    void setPosY(int _y);
    void setIndexOfRightmostRay(int _iLR);
    void setWeight(float _weight);
    void setPosition(int _x, int _y);

    int getPosX();
    int getPosY();
    int getIndexOfRightmostRay();
    float getWeight();
    float getOrientation();

    ~Particle();
protected:
    int x;
    int y;
    int indexOfRightmostRay;
    float orientation;
    float weight;
};


#endif //PARTICLEFILTER_PARTICLE_H
