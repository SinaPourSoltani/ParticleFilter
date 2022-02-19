//
// Created by Sina P. Soltani on 20/11/2019.
//

#ifndef PARTICLEFILTER_PARTICLEFILTER_H
#define PARTICLEFILTER_PARTICLEFILTER_H

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <cmath>
#include <random>
#include "Particle.h"

#define RESEMBLANCE_THRESHOLD 0.9

using namespace std;
using namespace cv;

class ParticleFilter {
public:
    ParticleFilter();
    ParticleFilter(Mat *_image);
    void setData(float _speed, float _direction ,float *_lidarData);
    void onSetData();

    void updateRobotPos(float _robX, float _robY);

    void generateInitialSetOfParticles();
    void locateROI();
    void generateParticleLookUpMap();
    void updateWeights();
    void resampleParticles();
    void updateStateOfParticlesBasedOnModel();

    float calculateWeightForParticle(int x, int y, int indexOfLeftmostRay);

    float normal_pdf(float x, float m, float s);
    bool isInBounds(int x, int y);
    bool isObstacle(int x, int y);
    void createLidarVisionForPoint(int x, int y, float *lidarVision);
    void startOver();

    void updateImage();
    void showImage();

    void drawLine(Point *from, Point *to);
    void drawCircle(Point *center, int r, bool isRobot);

    ~ParticleFilter();
protected:
    int rows;
    int cols;

    Mat *image;
    Mat *imageToShow;

    int numMapCoveringParticles;
    Particle ** initialParticles;
    Particle ** particles;

    float speed;
    float direction;
    float *lidarData;
    float *** visionMap;

    Point robotPose;
    bool hasStabilized;
    bool isInitial;
    string windowName;
    Vec3b particleColor;
};


#endif //PARTICLEFILTER_PARTICLEFILTER_H
