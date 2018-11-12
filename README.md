#### ROS wrapper for dense_flow.

work in progress..

#### TODO:

- remove easylogging.
- also CLUE seems not to be used at all, remove that and test
- remove matlab and python wrappers as well: the interface is with ros: matlab can read rostopics as well as python.
- make same modifications to warp flow, cpu flow
- remove/ fix saving of images (not functional with the many videos stream)
- fix the change video image transition error
- fix the normalization function (the CAST compiler def was not working and you changed for something else that does not make the same kinds of images - they used to look much grayer so the trained network might not work)



Extracting dense flow field given a video.

#### Depencies:
- LibZip:
to install on ubuntu ```apt-get install libzip-dev``` on mac ```brew install libzip```

#### For OpenCV 3 Users
Please see the [opencv-3.1](https://github.com/yjxiong/dense_flow/tree/opencv-3.1) branch. Many thanks to @victorhcm for the contributions!

### Install
```
git clone --recursive http://github.com/yjxiong/dense_flow
mkdir build && cd build
cmake .. && make -j
```

### Usage
```
./extract_gpu -f test.avi -x tmp/flow_x -y tmp/flow_y -i tmp/image -b 20 -t 1 -d 0 -s 1 -o dir
```
- `test.avi`: input video
- `tmp`: folder containing RGB images and optical flow images
- `dir`: output generated images to folder. if set to `zip`, will write images to zip files instead.

### Warp Flow
The warp optical flow is used in the following paper

```
@inproceedings{TSN2016ECCV,
  author    = {Limin Wang and
               Yuanjun Xiong and
               Zhe Wang and
               Yu Qiao and
               Dahua Lin and
               Xiaoou Tang and
               Luc {Van Gool}},
  title     = {Temporal Segment Networks: Towards Good Practices for Deep Action Recognition},
  booktitle   = {ECCV},
  year      = {2016},
}
```

To extract warp flow, use the command
```
./extract_warp_gpu -f test.avi -x tmp/flow_x -y tmp/flow_y -i tmp/image -b 20 -t 1 -d 0 -s 1 -o dir
```
