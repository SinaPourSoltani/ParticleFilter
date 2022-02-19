//
// Created by Sina P. Soltani on 27/11/2019.
//

#include "ParticleLookUp.h"

ParticleLookUp::ParticleLookUp() {}

ParticleLookUp::ParticleLookUp(float *** _visionMap) {
    visionMap = _visionMap;
    resemblanceFactors = new float[NUM_VISION_DATA_PTS];
    resemblanceFactorOfBestSimilitude = 1;
}

void ParticleLookUp::onSetDataByParticleWithPos(int x, int y) {
    //calculateSimilitudesForParticle(x, y);
    //determineBestSimilitude();
}

void ParticleLookUp::calculateSimilitudesForParticle(int x, int y) {
    float sumOfNDiffs = 0;
    for (int i = 0; i < NUM_VISION_DATA_PTS; ++i) {
        for (int j = 0; j < NUM_LIDAR_DATA_PTS; ++j) {
            sumOfNDiffs += normal_pdf(lidarData[j],visionMap[y][x][((i + j >= NUM_VISION_DATA_PTS) ? i + j - NUM_VISION_DATA_PTS : i + j)]);
        }
        resemblanceFactors[i] = sumOfNDiffs / NUM_LIDAR_DATA_PTS;
        sumOfNDiffs = 0;
    }
}

void ParticleLookUp::determineBestSimilitude() {
    int index = 0;
    float largestResemblance = resemblanceFactors[index];
    for (int i = 0; i < NUM_VISION_DATA_PTS; ++i) {
        if(resemblanceFactors[i] > largestResemblance){
            largestResemblance = resemblanceFactors[i];
            index = i;
        }
    }
    indexOfBestSimilitude = index;
    resemblanceFactorOfBestSimilitude = largestResemblance;
}

float ParticleLookUp::getResemblanceFactor() {
    return resemblanceFactorOfBestSimilitude;
}

float ParticleLookUp::getOrientationOfBestSimilitude() {
    return indexOfBestSimilitude * ANGULAR_PREC;
}

float ParticleLookUp::normal_pdf(float x, float m) {
    static const float inv_sqrt_2pi = 0.3989423;
    float a = (x - m) / 1;

    return inv_sqrt_2pi / 1 * exp(-0.5f * a * a) * (float) 2.5066;
}

ParticleLookUp::~ParticleLookUp() {}