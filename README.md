# Pose Optimization Using G2O Library: an Easy Example

## A small sample for pose optimization using g2o to minimize reprojection errors.

### Usage:

Requirement:

1. OpenCV 3.1.0 or newer version.

2. LAPACK && Suitesparse && Eigen3.

3. (optional) Viz module in opencv (needs compiling with VTK). Disable the display in the macro DISPLAY_3D.


Example:

There is an example given on optimizing a pose calculated from EPnP to give intuitions on the performance of g2o. 

You may write extra code to implement other optimization based on g2o.


