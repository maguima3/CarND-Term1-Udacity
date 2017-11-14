# Finding Lane Lines on the Road

### Reflection

In this section, I describe the steps performed by `find_lanes_pipeline` function to dentify and draw the lane lines on images and videos (which are just a serie of images).


#### Context of the test images
* Images are taken from a front facing camera mounted in a fixed position on the car, such that the lane lines always appear in the same general region of the image. Approximately, the center of the lane lines is located at the center of the images.
* Images show good visibility and light conditions.
* Lane lines vary in colour (white or yellow) and length (continuos or discontinuos). 
* Other cars do not cross the lane lines in any moment.

#### Pipeline description
Below, I describe the steps necessary to identify and draw the lane lines on a picture. As an example, I use an image of a solid white curve lane.
![alt text][start]
* First, it is necessary to convert the image to grayscale [`grayscale`].
![alt text][gray]
* Use the Canny algorithm to detect edges. Along with the previous grayscale image, it is also necessary to define a low and hight threshold values, which determine the pixels that are labeled as edges on the imsge. In this project, I've chosen a 1:3 low to hight ratio, being the lower and threshold 50 and 150 resoectivly [`canny`].
![alt text][edges]
* The next step is to detect the lane lines from the edges of the images. To obtain an accuarate result, it is first necessary to filter out the edges in other areas of the image (e.g trees, other cars, etc) that may introduce noise. I've accomplished it using a quadrilateral region of interest mask, which borders the region where lane lines appear. Everything else outside this regios is wiped out [`region_of_interest`]. 
![alt text][mask]
* Next, the Hough Transform is applied on the filtered edge-detected image to detect the segments present on the image [`hough_lines`]. To identify the lane lines, some steps more are needed [`draw_lines`]:
    * Separate line segments by their slope to decide which segments are part of the left line vs. the right line. At first, I only considered the sign of the slope: segments with positive slope were part of the left lane, and segments with negaive slope of the right lane. To make the algorithm more robust, I added another restriction: segments of lane lines must be inclined. I defined that the minumum slope that segments must have is 0.5.
    * Separate line segments by their location: segments of the right line are located on the right half side of the image, and segments of the left line on tft side.  
    * Finally, I use the line segments to extrapolate the full extent of the lines.
    ![alt text][lines]
* The last step consists on drawing the detected lane lines on the initial image [`weighted_img`].
![alt text][end]

[//]: # (Image References)

[start]: ./test_images/solidWhiteCurve.jpg "Initial image: Solid White Curve"
[gray]: ./examples/solidWhiteCurve_grayscale.jpg "Gray scale"
[edges]: ./examples/solidWhiteCurve_edges.jpg "Edges"
[mask]: ./examples/solidWhiteCurve_mask.jpg "Mask"
[lines]: ./examples/solidWhiteCurve_lines.jpg "Detected lane lines"
[end]: ./examples/solidWhiteCurve_end.jpg "Result"

---

###  Submission Videos

1. Finding lane lines: solid white lines

[![Solid white lines](http://img.youtube.com/vi/M_dnxvHdv6A/0.jpg)](https://youtu.be/M_dnxvHdv6A "Solid white lines - Click to Watch!")


2. Finding lane lines: solid yellow left

[![Solid yellow left](http://img.youtube.com/vi/q3Dp-TF8yYs/0.jpg)](https://youtu.be/q3Dp-TF8yYs "Solid yellow left - Click to Watch!")

3. Challenge

[![Challenge](http://img.youtube.com/vi/YwUlD8wuwM4/0.jpg)](https://youtu.be/YwUlD8wuwM4 "Challenge - Click to Watch!")


---

###  Shortcomings
Some potential shortcoming would be:
* Images must be taken from a front facing camera mounted in a fixed position on the car, such that the lane lines always appear in the same central general region of the image. If not, the region of interest mask used to filter out everything but the lane lines could wipe out the lanes.
* Cars crossing the lane lines.
* Sharp curves, as lane lines are extrapolated from the line segments using a linear function.

---

### Improvements

* A possible improvement for videos would be to consider the position of the lane lines in the previous image to make better predictions.