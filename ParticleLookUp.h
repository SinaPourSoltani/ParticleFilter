//
// Created by Sina P. Soltani on 27/11/2019.
//

#ifndef PARTICLEFILTER_PARTICLELOOKUP_H
#define PARTICLEFILTER_PARTICLELOOKUP_H

#include <cmath>
#include <iostream>

#define LARGEST_ERROR ((10 - 0.08) * 200)
#define FOV (2.27 - (-2.27))
#define NUM_VISION_DATA_PTS 90
#define NUM_LIDAR_DATA_PTS 65
#define ANGULAR_PREC (float) (FOV / NUM_LIDAR_DATA_PTS)
#define RAY_LENGTH 100
#define NUM_PARTICLES 1000

using namespace std;

class ParticleLookUp {
public:
    ParticleLookUp();
    ParticleLookUp(float *** _visionMap);

    void onSetDataByParticleWithPos(int x, int y);

    void calculateSimilitudesForParticle(int x, int y);
    void determineBestSimilitude();

    float getResemblanceFactor();
    float getOrientationOfBestSimilitude();

    float normal_pdf(float x, float m);

    ~ParticleLookUp();
protected:
    float *** visionMap;
    float *lidarData;
    float *resemblanceFactors;
    int indexOfBestSimilitude;
    float resemblanceFactorOfBestSimilitude;
};


#endif //PARTICLEFILTER_PARTICLELOOKUP_H
