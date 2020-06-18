# Orthogonal Plane-based Visual Odometry (OPVO)
This package provides a MATLAB implementation of BMVC 2017 paper: "Visual Odometry with Drift-Free Rotation Estimation Using Indoor Scene Regularities" for the purpose of research and study only.
Note that this repository only includes simplified proposed visual odometry example codes to understand how the OPVO works in structured environments.

![OPVO](https://github.com/PyojinKim/OPVO/blob/master/overview.png)


# 1. Goal
Our goal is to estimate 6-DoF camera motion with respect to the indoor structured environments.
For reducing drift of the rotation estimate, which is the main source of position inaccuracy in visual odometry, the Manhattan frame tracking is performed to estimate the absolute camera orientation.
Given drift-free rotation estimates in Manhattan World, translational motion is estimated by minimizing de-rotated reprojection with the tracked features.

![OPVO](https://github.com/PyojinKim/OPVO/blob/master/result.png)


# 2. Prerequisites
This package depends on [mexopencv](https://github.com/kyamagu/mexopencv) library for keypoint processing, KLT tracking, and translation estimation.
cv.* functions in this package cannot run without mexopencv install in the MATLAB environment.
Please, build [mexopencv](https://github.com/kyamagu/mexopencv) in your OS first, and then run this package.


# 3. Usage
* Download the ICL-NUIM dataset from https://www.doc.ic.ac.uk/~ahanda/VaFRIC/iclnuim.html, 'of kt3' is recommended.

* Or, Use the ICL-NUIMdataset/of_kt3/ included in this package.

* Define 'datasetPath' correctly in your directory at setupParams_ICL_NUIM.m file.

* Run OPVO_core/main_script_ICL_NUIM.m, which will give you the 3D motion estimation result. Enjoy! :)


# 4. Publications
The approach is described and used in the following publications:

* **Linear RGB-D SLAM for Planar Environments** (Pyojin Kim, Brian Coltin, and H. Jin Kim), ECCV 2018.

* **Indoor RGB-D Compass from a Single Line and Plane** (Pyojin Kim, Brian Coltin, and H. Jin Kim), CVPR 2018.

* **Low-Drift Visual Odometry in Structured Environments by Decoupling Rotational and Translational Motion** (Pyojin Kim, Brian Coltin, and H. Jin Kim), ICRA 2018.

You can find more related papers at http://pyojinkim.com/_pages/pub/index.html.


# 5. License
The package is licensed under the MIT License, see http://opensource.org/licenses/MIT.

if you use OPVO in an academic work, please cite:

    @inproceedings{kim2017visual,
      author = {Kim, Pyojin and Coltin, Brian and Kim, H Jin},
      title = {Visual Odometry with Drift-Free Rotation Estimation Using Indoor Scene Regularities},
      year = {2017},
      booktitle = {British Machine Vision Conference (BMVC)},
     }

