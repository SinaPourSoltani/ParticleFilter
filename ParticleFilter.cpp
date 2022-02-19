//
// Created by Sina P. Soltani on 20/11/2019.
//

#include "ParticleFilter.h"

ParticleFilter::ParticleFilter() {}

ParticleFilter::ParticleFilter(Mat *_image) {
    image = _image;
    rows = image->rows;
    cols = image->cols;

    robotPose = Point(cols/2,rows/2);
    hasStabilized = false;
    isInitial = true;
    windowName = "ParticleMap";
    particleColor = {0,255,0};

    imageToShow = new Mat(rows,cols,image->type());
    *imageToShow = image->clone();

    visionMap = new float**[rows];
    for (int y = 0; y < rows; ++y) {
        visionMap[y] = new float*[cols];
        for (int x = 0; x < cols; ++x) {
            visionMap[y][x] = new float[NUM_VISION_DATA_PTS];
        }
    }
    generateParticleLookUpMap();

    particles = new Particle*[NUM_PARTICLES];
}

void ParticleFilter::setData(float _speed, float _direction, float *_lidarData) {
    speed = _speed;
    direction = _direction;
    lidarData = _lidarData;
    cout << "S: " << speed << endl;
    cout << "D: " << direction << endl;

    //cout << calculateWeightForParticle(430,277,58) << endl;
    //cout << calculateWeightForParticle(430,277,13) << endl;
    //cout << calculateWeightForParticle(230,277,58) << endl;
    //cout << calculateWeightForParticle(829,277,58) << endl;

    onSetData();
}

void ParticleFilter::onSetData() {
    if(isInitial){
        generateInitialSetOfParticles();
        locateROI();
        isInitial = false;
    } else {
        updateWeights();
        resampleParticles();
        updateStateOfParticlesBasedOnModel();
        updateImage();
        showImage();
    }
}

void ParticleFilter::updateRobotPos(float _robDX, float _robDY) {
    //cout << "RDX: " << _robDX << " - " << "RDY: " << _robDY << endl;
    robotPose = Point(cols/2 + _robDX * MAP_TO_SIM_SCLR, rows/2 - _robDY * MAP_TO_SIM_SCLR);
}

void ParticleFilter::generateInitialSetOfParticles() {
    initialParticles = new Particle*[rows * cols];

    numMapCoveringParticles = 0;
    for (int y = 20; y < rows - 20; y += 3) {
        for (int x = 20; x < cols - 20; x += 3) {
            if(!isObstacle(x,y)){
                int randomOrientation = rand() % NUM_VISION_DATA_PTS;
                initialParticles[numMapCoveringParticles++] = new Particle(x,y,randomOrientation);
                imageToShow->at<Vec3b>(y,x) = particleColor;
            }
        }
    }
    showImage();
}

void ParticleFilter::locateROI() {
    cout << "Locating ROI" << endl;
    for (int i = 0; i < numMapCoveringParticles; ++i) {
        Particle * p = initialParticles[i];
        float weight = calculateWeightForParticle(p->getPosX(),p->getPosY(),p->getIndexOfRightmostRay());
        p->setWeight(weight);
    }

    Particle ** tmpArray = new Particle*[NUM_PARTICLES];
    vector<float> weightVec(numMapCoveringParticles);

    double eta = 0;
    for (int i = 0; i < numMapCoveringParticles; ++i) {
        weightVec[i] = initialParticles[i]->getWeight();
        eta += weightVec[i];
    }
    for (int j = 0; j < numMapCoveringParticles; ++j) {
        weightVec[j] /= eta;
    }

    random_device rd;
    mt19937 gen(rd());
    discrete_distribution<> distribution(weightVec.begin(), weightVec.end());

    for (int i = 0; i < NUM_PARTICLES; ++i) {
        Particle * p = initialParticles[distribution(gen)];
        int randomOrientation = (p->getIndexOfRightmostRay() + ((rand() % RANDOM_CHANGE_IN_VISION_RAY) - RANDOM_CHANGE_IN_VISION_RAY/2)) % NUM_VISION_DATA_PTS;
        randomOrientation = (randomOrientation < 0) ? randomOrientation + NUM_VISION_DATA_PTS : randomOrientation;
        tmpArray[i] = new Particle(p->getPosX(),p->getPosY(),randomOrientation);
    }

    delete [] initialParticles;
    delete [] particles;
    particles = tmpArray;

    updateImage();
    showImage();
}

void ParticleFilter::generateParticleLookUpMap() {
    for (int y = 0; y < rows; ++y) {
        cout << "Row# " << y << endl;
        for (int x = 0; x < cols; ++x) {
            if(!isObstacle(x,y)){
                auto *lidarVision = new float[NUM_VISION_DATA_PTS];
                createLidarVisionForPoint(x,y,lidarVision);
                visionMap[y][x] = lidarVision;
            }
        }
    }
}

void ParticleFilter::updateWeights() {
    cout << "UPDATING WEIGHTS" << endl;
    for (int i = 0; i < NUM_PARTICLES; ++i) {
        //cout << "Particle #" << i << ": ";
        Particle * p = particles[i];
        //cout << "[" << p->getPosX() << ", " << p->getPosY() << "]" << " - iRR: " << p->getIndexOfRightmostRay() << " - O: " << p->getOrientation() << " - W: ";
        float weight = calculateWeightForParticle(p->getPosX(),p->getPosY(),p->getIndexOfRightmostRay());
        //cout << weight << endl;
        p->setWeight(weight);
    }
}

void ParticleFilter::resampleParticles() {
    cout << "RESAMPLING PARTICLES" << endl;
    Particle ** tmpArray = new Particle*[NUM_PARTICLES];
    vector<double> weightVec(NUM_PARTICLES);

    double eta = 0;
    for (int i = 0; i < NUM_PARTICLES; ++i) {
        weightVec[i] = particles[i]->getWeight();
        eta += weightVec[i];
    }
    cout << "ETA: " << eta << endl;
    for (int j = 0; j < NUM_PARTICLES; ++j) {
        weightVec[j] /= eta;
    }

    if(eta > ETA_UPPER_BOUND){
        hasStabilized = true;
    }

    if(eta < ETA_LOWER_BOUND && hasStabilized){
        cout << "L: " << ETA_LOWER_BOUND << " " << "U: " << ETA_UPPER_BOUND << "Eta: " << eta << endl;
        startOver();
    }

    random_device rd;
    mt19937 gen(rd());
    discrete_distribution<> distribution(weightVec.begin(),weightVec.end());

    for (int i = 0; i < NUM_PARTICLES; ++i) {
        Particle * p = particles[distribution(gen)];
        int randomOrientation = (p->getIndexOfRightmostRay() + ((rand() % RANDOM_CHANGE_IN_VISION_RAY) - RANDOM_CHANGE_IN_VISION_RAY/2)) % NUM_VISION_DATA_PTS;
        randomOrientation = (randomOrientation < 0) ? randomOrientation + NUM_VISION_DATA_PTS : randomOrientation;
        tmpArray[i] = new Particle(p->getPosX(),p->getPosY(),randomOrientation);
    }
    
    delete [] particles;
    particles = tmpArray;
}

void ParticleFilter::updateStateOfParticlesBasedOnModel() {
    cout << "UPDATING STATE" << endl;
    for (int i = 0; i < NUM_PARTICLES; ++i) {
        Particle * p = particles[i];
        p->setIndexOfRightmostRay(round(((float) (2 * M_PI / NUM_VISION_DATA_PTS) * p->getIndexOfRightmostRay() + direction * DELTA_T) * (float) (NUM_VISION_DATA_PTS /(2*M_PI))));

        random_device rd;
        mt19937 gen(rd());
        normal_distribution<float> distribution(2*speed,2);

        int dx = (distribution(gen)) * cos(p->getOrientation());
        int dy = (distribution(gen)) * sin(p->getOrientation());

        int newX = p->getPosX() + dx;
        int newY = p->getPosY() - dy;

        if(isInBounds(newX,newY) && !isObstacle(newX,newY))
            p->setPosition(newX, newY);
    }
}

float ParticleFilter::calculateWeightForParticle(int x, int y, int iRRay) {

    //cout << "[" << x << ", " << y << "]" << endl;
    float sumOfNDiffs = 0;
    for (int i = 0; i < NUM_LIDAR_DATA_PTS; ++i) {
        float scoreOfSingleDataPt = normal_pdf(lidarData[i] * MAP_TO_SIM_SCLR,visionMap[y][x][((iRRay + i >= NUM_VISION_DATA_PTS) ? iRRay + i - NUM_VISION_DATA_PTS : iRRay + i)], 3);
        //cout << lidarData[i] * MAP_TO_SIM_SCLR << " ··· " << visionMap[y][x][((iLRay + i >= NUM_VISION_DATA_PTS) ? iLRay + i - NUM_VISION_DATA_PTS : iLRay + i)] << " : " << scoreOfSingleDataPt << endl;
        sumOfNDiffs += scoreOfSingleDataPt;
    }
    //cout << endl << endl;
    return sumOfNDiffs;
}

float ParticleFilter::normal_pdf(float x, float m, float s) {
    static const float inv_sqrt_2pi = 0.3989423;
    float a = (x - m) / s;

    return inv_sqrt_2pi / s * exp(-0.5f * a * a) * s / inv_sqrt_2pi;
}

bool ParticleFilter::isInBounds(int x, int y) {
    return 0 < x && x < cols && 0 < y && y < rows;
}

bool ParticleFilter::isObstacle(int x, int y) {
    if(isInBounds(x,y))
        return image->at<Vec3b>(y,x)[0] == 0;
    else
        return true;
}

void ParticleFilter::createLidarVisionForPoint(int x, int y, float *lidarVision) {
    Point particle = Point(x, y);
    //Point testParticle = Point(430,277);

    for (int i = 0; i < NUM_VISION_DATA_PTS; ++i) {
        double angle = i * 2 * M_PI / NUM_VISION_DATA_PTS;
        bool wallInDaWay = false;

        Point furthestPointInSight = Point(x + round(RAY_LENGTH * cos(angle)), y - round(RAY_LENGTH * sin(angle)));
        LineIterator lineIterator(*image, particle, furthestPointInSight);

        for (int k = 0; k < lineIterator.count; k++, lineIterator++) {
            if (isObstacle(lineIterator.pos().x, lineIterator.pos().y)) {
                wallInDaWay = true;
                break;
            }
        }

        lidarVision[i] = (float) norm(particle - ((wallInDaWay) ? lineIterator.pos() : furthestPointInSight));

        /*if(testParticle.x == particle.x && testParticle.y == particle.y){
            cout << lidarVision[i] << endl;
            line(*imageToShow, particle, ((wallInDaWay) ? lineIterator.pos() : furthestPointInSight), Scalar{0, 200, 0});
            showImage();
            waitKey();
        }*/

    }
    //imshow("A",*tegneBillede);
    //waitKey(1);
}

void ParticleFilter::startOver() {
    hasStabilized = false;
    isInitial = true;
}

void ParticleFilter::updateImage() {
    *imageToShow = image->clone();

    cout << "UPDATING IMAGE" << endl;
    for (int i = 0; i < NUM_PARTICLES; ++i) {
        Point particle(particles[i]->getPosX(),particles[i]->getPosY());
        float orientation = particles[i]->getOrientation();
        int dirArrowX = 5 * cos(orientation);
        int dirArrowY = 5 * sin(orientation);
        Point arrow(particle.x + dirArrowX,particle.y - dirArrowY);
        if(isInBounds(arrow.x,arrow.y)){
            line(*imageToShow, particle, arrow, Scalar{200,0, 150});
        }
        imageToShow->at<Vec3b>(particle.y,particle.x) = particleColor;
    }
    drawCircle(&robotPose,1,true);
}

void ParticleFilter::showImage() {
    cout << "SHOWING IMAGE" << endl;
    imshow(windowName,*imageToShow);
    waitKey(1);
}

void ParticleFilter::drawLine(Point *from, Point *to) {
    line(*image,*from,*to,Scalar{0,0,0});
}

void ParticleFilter::drawCircle(Point *center, const int r, bool isRobot) {
    Scalar color = ((isRobot) ? Scalar(164,66,245) : Scalar(70,189,65));
    circle(*imageToShow,*center,r,color,10);
}

ParticleFilter::~ParticleFilter() {}