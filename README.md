# edge-plan-odometry

This work propose a matlab framwork for a feature based lidar Odometry.

It is highly inspired by Tixiao Shan and Brendan Englot's work: Lego LOAM (https://github.com/RobustFieldAutonomyLab/LeGO-LOAM)

# recommended parameters

This framework can be used with Velodyne VLP-16 and Velodyne HDL-64. Here are the recommended parameters for both configurations:

- VLP-16 : c_edge = 1, c_plane = 0.025, dist_threshold = 0.5
- HDL-64 : c_edge = 0.2, c_plane = 0.025, dist_threshold = 0.2

# current results 

This is a plot on a KITTI dataset trajectory
<p align='center'>
    <img src="/results/result.png" alt="drawing" width="400"/>
</p>
