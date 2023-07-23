# final
Compressed image raw of realsense --> pointcloud data -> octomap -> flood fill -> control 

```Bash
rosrun image_transport republish in:=/camera/color/image_raw compressed raw out:=/camera/color/image_raw _image_transport:=compressed
```
