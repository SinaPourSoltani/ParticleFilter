# ParticleFilter

An implementation of a [Particle Filter](https://en.wikipedia.org/wiki/Particle_filter) that was written to run as part of a larger project in a Gazebo simulation. 

The implementation is environment independent as the `ParticleFilter` accepts lidar data as an array of floats.

A simulated robot car moves around in a known environment. Connected to the car is a `lidar sensor` as well as a monochrome camera.

The purpose of this implementation is to determine the location of the robot car using only the lidar data.

## Map
A map is given by a simple bw image, with black pixels representing wall.

<img src=floor_plan.png>

## Demo


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