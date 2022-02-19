//
// Created by Sina P. Soltani on 20/11/2019.
//

#include "Particle.h"

Particle::Particle() {}

Particle::Particle(int _x, int _y, int _indexOfRightmostRay) {
    x = _x;
    y = _y;
    indexOfRightmostRay = _indexOfRightmostRay;
    calculateOrientation();
    weight = (float) 1 / NUM_PARTICLES;
}

void Particle::calculateOrientation() {
    //cout << "IRR: " << indexOfRightmostRay << endl;
    //cout << "CR: " << (indexOfRightmostRay + (NUM_LIDAR_DATA_PTS - 1)/2) % NUM_VISION_DATA_PTS << endl;
    //cout << "ILR: " << (indexOfRightmostRay + NUM_LIDAR_DATA_PTS) % NUM_VISION_DATA_PTS << endl;
    orientation = ((indexOfRightmostRay + (NUM_LIDAR_DATA_PTS - 1)/2) % NUM_VISION_DATA_PTS) * ANGULAR_PREC;
    //cout << "O: " << orientation << endl;
}

void Particle::setPosX(int _x) {
    x = _x;
}

void Particle::setPosY(int _y) {
    y = _y;
}

void Particle::setIndexOfRightmostRay(int _iRR) {
    indexOfRightmostRay = _iRR;
    calculateOrientation();
}

void Particle::setWeight(float _weight) {
    weight = _weight;
}

void Particle::setPosition(int _x, int _y) {
    x = _x;
    y = _y;
}

int Particle::getPosX() {
    return x;
}

int Particle::getPosY() {
    return y;
}

int Particle::getIndexOfRightmostRay() {
    return indexOfRightmostRay;
}

float Particle::getWeight() {
    return weight;
}

float Particle::getOrientation() {
    return orientation;
}

Particle::~Particle() {}