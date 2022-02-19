#include "ParticleFilter.h"
#include <time.h>

#define IMG_SCL_FCTR (double) 10/1.41735

void showImage(string *name, Mat *image){
    namedWindow(*name,WINDOW_NORMAL);
    imshow(*name,*image);
}

void onMouse(int event, int x, int y, int flags, void* param){
    if(event == EVENT_LBUTTONDOWN){
        cout << "[" << x << "," << y << "]" << endl;
    }
}

int main() {
    srand(time(NULL));


    Mat image = imread("../floor_plan.png");
    string imageName = "ParticleMap";
    resize(image,image,Size(847,564),IMG_SCL_FCTR,IMG_SCL_FCTR,INTER_NEAREST);

    showImage(&imageName, &image);
    setMouseCallback(imageName, onMouse);
    waitKey();

    /*
    vector<double> vec = {1.5,2.2,3.4,4.2,5.0,6.0};

    random_device rd;
    mt19937 gen(rd());
    discrete_distribution<> distribution(vec.begin(),vec.end());

    int * numOccurences = new int[6];

    for (int k = 0; k < 6; ++k) {
        numOccurences[k] = 0;
    }

    for (int i = 0; i < 22300; ++i) {
        numOccurences[distribution(gen)]++;
    }

    for (int j = 0; j < 6; ++j) {
        cout << j << ": " << (double) numOccurences[j]/10000 << endl;
    }*/



    float speed = 1.49996;
    float direction = 1.66222e-17;
    float lidarData[65] = {6.83487, 6.45959, 6.11825, 5.93261, 5.72775, 5.60303, 5.44365, 5.42622, 5.23998, 5.2504,
                        5.21772, 5.23751, 5.2756, 5.29091, 5.37708, 5.53584, 5.704, 5.81438, 6.09055, 6.38394,
                        6.65575, 7.06101, 7.59235, 8.17386, 7.84224, 7.4553, 7.19287, 7.671, 9.05979, 10,
                        10, 10, 10, 10, 10, 10, 10, 10, 10, 10,
                        8.65837, 7.67899, 6.92141, 6.36424, 5.84679, 5.48156, 5.12841, 4.80371, 4.66028, 4.44314,
                        4.32946, 4.25785, 4.1314, 4.03212, 4.04525, 3.95719, 3.89755, 3.90744, 4.01774, 4.08356,
                        4.10161, 4.21872, 4.29417, 4.42965, 4.60744};

    ParticleFilter particleFilter = ParticleFilter(&image);
    particleFilter.setData(speed,direction,lidarData);

    showImage(&imageName, &image);
    setMouseCallback(imageName, onMouse);
    waitKey();

    return 0;
}