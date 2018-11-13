#### ROS wrapper for dense_flow.

work in progress..

#### TODO:

- remove easylogging.
- also CLUE seems not to be used at all, remove that and test
- make same modifications to warp flow, cpu flow
- remove/ fix saving of images (not functional with the many videos stream)
- fix the change video image transition error
- fix the normalization function (the CAST compiler def was not working and you changed for something else that does not make the same kinds of images - they used to look much grayer so the trained network might not work)

Extracting dense flow field given a video.
