# Orthogonal Plane-based Visual Odometry (OPVO)
This package provides a MATLAB implementation of BMVC 2017 paper: "Visual Odometry with Drift-Free Rotation Estimation Using Indoor Scene Regularities" for the purpose of research and study only.

<a href="https://www.youtube.com/embed/sC3iiaxBhdw" target="_blank"><img src="http://img.youtube.com/vi/sC3iiaxBhdw/0.jpg" 
alt="OPVO" width="240" height="180" border="10" /></a>


# 1. License
The package is licenced under the MIT License, see http://opensource.org/licenses/MIT.

if you use OPVO in an academic work, please cite:

    @inproceedings{kim2017visual,
	  author = {Kim, Pyojin and Coltin, Brian and Kim, H Jin},
	  title = {Visual Odometry with Drift-Free Rotation Estimation Using Indoor Scene Regularities},
      year = {2017},
	  booktitle = {British Machine Vision Conference (BMVC)},
     }


# 2. Prerequisites
This package depends on [mexopencv](https://github.com/kyamagu/mexopencv) library for keypoint processing, KLT tracking, and translation estimation.
cv.* functions in this package cannot run without mexopencv install in Matlab environment.
Please, build [mexopencv](https://github.com/kyamagu/mexopencv) in your OS first, and then run this package.


# 3. Usage
* Download the ICL-NUIM dataset from https://www.doc.ic.ac.uk/~ahanda/VaFRIC/iclnuim.html, 'of kt3' is recommanded.

* Or, Use the ICL-NUIMdataset/of_kt3/ included in this package.

* Define 'datasetPath' correctly in your directory at setupParams_ICL_NUIM.m file.

* Run OPVO_core/main_script_ICL_NUIM.m which will give you the 3D motion estimation result. Enjoy! :)


# 4. Publications
The approach is descirbed and used in the following publications:

* **Linear RGB-D SLAM for Planar Environments** (Pyojin Kim, Brian Coltin, and H. Jin Kim), ECCV 2018.

* **Indoor RGB-D Compass from a Single Line and Plane** (Pyojin Kim, Brian Coltin, and H. Jin Kim), CVPR 2018.

* **Low-Drift Visual Odometry in Structured Environments by Decoupling Rotational and Translational Motion** (Pyojin Kim, Brian Coltin, and H. Jin Kim), ICRA 2018.

You can find more related papers at http://pyojinkim.com/_pages/pub/index.html.
