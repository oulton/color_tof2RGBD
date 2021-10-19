# color_tof2RGBD

```bash
cd ~/rosworkspace/src/
git clone git@github.com:oulton/color_tof2RGBD.git
cd ..
catkin_make

source devel/setup.bash
roslaunch imagepub imagepub.launch
```
##  input:

    topic_img      topic_tof_pcd

    color_image   （bgr8）

    pcd_cloud      (pcl::PointXYZ)

##  output:
    
    topic_color2depth    topic_depth

    color2depth_img  （bgr8）

    depth_img         (mono16)
