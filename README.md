# ParticleFilter

An implementation of a [Particle Filter](https://en.wikipedia.org/wiki/Particle_filter) that was written to run as part of a larger project in a Gazebo simulation. 

The implementation is environment independent as the `ParticleFilter` accepts lidar data as an array of floats.

A simulated robot car moves around in a known environment. Connected to the car is a `lidar sensor` as well as a monochrome camera.

The purpose of this implementation is to determine the location of the robot car using only the lidar data.

This implementation of the `ParticleFilter` also supports *transportation*. If the robot car suddenly finds itself in a different position or if the filter drifts too much, the entire map is resampled anew. This can be seen at 01:04 in the demo video.

## Map
A map is given by a simple bw image, with black pixels representing wall.

<img src=floor_plan.png>

## Demo
https://user-images.githubusercontent.com/42042078/154821891-b562690c-0267-4171-9299-c70776025068.mp4

## Usage

```bash
$ git clone [repo] 
# update files to match project
$ mkdir build && cd build
$ cmake ..
$ make
$ ./Particlefilter
```

## More info
For more information check the excerpt from the associated [paper](pf_paper.pdf).
