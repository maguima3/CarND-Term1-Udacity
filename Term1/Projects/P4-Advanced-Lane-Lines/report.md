# Advanced Lane Finding Project

The goals / steps of this project are the following:

* Compute the camera calibration matrix and distortion coefficients given a set of chessboard images.
* Apply a distortion correction to raw images.
* Use color transforms, gradients, etc., to create a thresholded binary image.
* Apply a perspective transform to rectify binary image ("birds-eye view").
* Detect lane pixels and fit to find the lane boundary.
* Determine the curvature of the lane and vehicle position with respect to center.
* Warp the detected lane boundaries back onto the original image.
* Output visual display of the lane boundaries and numerical estimation of lane curvature and vehicle position.

[//]: # (Image References)

[undistorted]: ./output_images/undistorted.png
[undistorted2]: ./output_images/undistorted2.png 
[color_channels]: ./output_images/color_channels.png 
[yellow_white_masks]: ./output_images/yellow_white_masks.png 
[gradient_thesholds]: ./output_images/gradient_thesholds.png∫
[final_mask]: ./output_images/final_mask.png
[warped]: ./output_images/warped.png
[histogram]: ./output_images/histogram.png
[final_result]: ./output_images/example_output.png

## [Rubric](https://review.udacity.com/#!/rubrics/571/view) Points

### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---

### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  [Here](https://github.com/udacity/CarND-Advanced-Lane-Lines/blob/master/writeup_template.md) is a template writeup for this project you can use as a guide and a starting point.  

You're reading it!

### Camera Calibration

#### 1. Briefly state how you computed the camera matrix and distortion coefficients. Provide an example of a distortion corrected calibration image.

The code for this step is contained in the first code cell of the `p4-advanced-line-finding` IPython notebook.

Camera calibration is the process of estimating the parameters of lens and image sensor of an image or video camera. One can use these parameters to correct for lens distortion, measure the size of an object in word units, or determine the location of the camera in the scene. Camera parameters include intrinsic, extrinsic, and distortion coefficients.

To estimate the camera parameters, it is necessary to have 3D world points and their corresponding 2D image points. To get these correspondences, I used multiple images of chessboards.

I started by preparing "object points", which are the (x, y, z) coordinates of the chessboard corners in the world. Here I am assuming the chessboard is fixed on the (x, y) plane at z=0, such that the object points are the same for each calibration image.  Thus, `objp` is just a replicated array of coordinates, and `objpoints` will be appended with a copy of it every time I successfully detect all chessboard corners in a test image.  `imgpoints` are appended with the (x, y) pixel position of each of the corners in the image plane with each successful chessboard detection.  

I then used the output `objpoints` and `imgpoints` to compute the camera calibration and distortion coefficients using the `cv2.calibrateCamera()` function.  I applied this distortion correction to the test image using the `cv2.undistort()` function and obtained this result: 

![alt text][undistorted]

### Pipeline (single images)

#### 1. Provide an example of a distortion-corrected image.

Here is an example of an undistorted image:
![alt text][undistorted2]

Although the distortion is not very high, one can see the distortion effects on the edges of the original image: objects appear more curved than they actually are.

#### 2. Describe how (and identify where in your code) you used color transforms, gradients or other methods to create a thresholded binary image.  Provide an example of a binary image result.

Color and gradient thresholds generate binary images that keep pixels belonging to lane lines and remove the irrelevant elements (noise) as much as possible.

The transformations I found more helpful for finding lane lines on images are:

* Color channel thresholds (7th cell code): As the image bellow shows, Red (R) and Saturation (S) binary channel masks are very good for filtering out lane lines.
![alt text][color_channels]
* Color masks (9th cell code): white and yellow color masks really help in detecting pixels of the images.
![alt text][yellow_white_masks]
* Gradient thresholds (11th cell code): as lane lines tend to be close to vertical, the Sobel operator on the X axis is suitable for finding lane lines. More particularly, we are interested on lines with a particular orientation (lines usually have 30 or 120 degrees approximately) and applying a threshold on the direction of the gradient also seems to help in filtering out lane lines.
![alt text][gradient_thesholds]

After testing several combinations of transformations, I decided to apply color thresholds to identify the lane lines on the images: I use the R and S binary channels as well as the yellow and white color masks. I decided not to use gradient thresholds as they decreased the algorithm performance on the `project_video.mp4`. However, I believe that in situations with worst light conditions (like in the challenge videos), gradient thresholds will be really useful.

This is how it looks like after applying color thresholds on test images:

![alt text][final_mask]

#### 3. Describe how (and identify where in your code) you performed a perspective transform and provide an example of a transformed image.

On the 3rd cell code of the IPhython notebook, I performed a perspective transform to get a bird's eye view of the line. Having a bird's eye view of the lane makes easier to fit a polynomio to the lane lines. This is a necessary step in order to determine the curvature of the lane.

To perform a perspective transform, it is needed to select four points that define a rectangle on a plane on the original image and where we want those same four points to appear on the transformed (warped) image.

The source (`src`) and destination (`dst`) points are created in the function `get_src_dst_pts()` with the following hardcoded values:


| Source        | Destination   | 
|:-------------:|:-------------:| 
| 200, 720      | 300, 720        | 
| 575, 465      | 300, 0      |
| 710, 465     | 1000, 0      |
| 1100, 729      | 1000, 720        |

I verified that my perspective transform was working as expected by drawing the `src` and `dst` points onto a test image and its warped counterpart to verify that the lines appear parallel in the warped image.

![alt text][warped]

#### 4. Describe how (and identify where in your code) you identified lane-line pixels and fit their positions with a polynomial?

The process of finding lane lines on binary, warped images consists on:

* Determine where the lines start. To answer this question, I take a histogram along all the columns in the lower half of the image. The peaks of the histogram determine the starting position of the lines.

![alt text][histogram]

* Follow the lines all way up to the top of the image to detect the pixels belonging to them. To accomplish this, I used sliding windows as suggested in the lectures.

* Fit a second degree polynomial to the lines


This process is performed by the `find_lanes()` function on the 20th cell code. Moreover, the function `draw_lines()` makes possible to visualize the result: it shows the detected lane and lines. 

#### 5. Describe how (and identify where in your code) you calculated the radius of curvature of the lane and the position of the vehicle with respect to center.

I calculate the radius of curvature of the lane with the function `get_radius_of_curvature()`, on the 22nd cell. It is possible to calculate the radius of curvature of a line from its parameters using the following formula:

`R_curvature = (1+(2Ay+B)^2)^1.5) / abs(2A)`

To determine the curvature of the lane, I computed the average of the left and right radius of curvature at y=540 (which is the 75% of the total height of the image) 

The vehicle offset with respect to the lane is computed by calculating the position (in meters) of the left and line lanes at the bottom of the image, relative to the center of the image. This is implemented in the function `get_lane_offset()`.


#### 6. Provide an example image of your result plotted back down onto the road such that the lane area is identified clearly.

Here is an example of my result on a test image:

![alt text][final_result]

---

### Pipeline (video)

#### 1. Provide a link to your final video output.  Your pipeline should perform reasonably well on the entire project video (wobbly lines are ok but no catastrophic failures that would cause the car to drive off the road!).

Here's a [link to my video result](./project_result.mp4)

---

### Discussion

#### 1. Briefly discuss any problems / issues you faced in your implementation of this project.  Where will your pipeline likely fail?  What could you do to make it more robust?

Although my pipeline recognizes the lane lines in the project video very well, the result on the challenging videos is not as good as expected for several reasons:

* There are worse light conditions (e.g shadows)
* Curves are sharper
* Road has more inclination, which can cause the perspective transformation not to work as expected

I believe the following improvements can make my algorithm more robust:

* Use gradient thresholds. This might help at bad light conditions, where the color masks are not enough for finding lines
* Include sanity checks, to verify that the detected lines make sense before displaying them
* Include look-ahead filters, to avoid blindly searching lines on each video frame
