# Vehicle Detection Project
---
The goals / steps of this project are the following:

* Perform a Histogram of Oriented Gradients (HOG) feature extraction on a labeled training set of images and train a classifier Linear SVM classifier
* Optionally, you can also apply a color transform and append binned color features, as well as histograms of color, to your HOG feature vector. 
* Note: for those first two steps don't forget to normalize your features and randomize a selection for training and testing.
* Implement a sliding-window technique and use your trained classifier to search for vehicles in images.
* Run your pipeline on a video stream (start with the test_video.mp4 and later implement on full project_video.mp4) and create a heat map of recurring detections frame by frame to reject outliers and follow detected vehicles.
* Estimate a bounding box for vehicles detected.

[//]: # (Image References)
[image1]: ./output_images/dataset_overview.jpg
[image2]: ./output_images/hog_visualization_HLS.jpg
[image3]: ./output_images/hog_visualization_HSV.jpg
[image4]: ./output_images/hog_visualization_LUV.jpg
[image5]: ./output_images/features_HOG_HLS.jpg
[image6]: ./output_images/features_HOG_HSV.jpg
[image7]: ./output_images/features_HOG_LUV.jpg
[image8]: ./output_images/features_Color_Hist.jpg
[image9]: ./output_images/features_Spatial.jpg
[image10]: ./output_images/multiscaleWindows_far.jpg
[image11]: ./output_images/multiscaleWindows_intermediate1.jpg
[image12]: ./output_images/multiscaleWindows_intermediate2.jpg
[image13]: ./output_images/multiscaleWindows_near.jpg
[image15]: ./output_images/prediction.jpg
[image16]: ./output_images/multiscale_sliding_windows_1.jpg
[image17]: ./output_images/multiscale_sliding_windows_2.jpg
[image18]: ./output_images/multiscale_sliding_windows_3.jpg
[image19]: ./output_images/multiscale_sliding_windows_4.jpg
[image20]: ./output_images/multiscale_sliding_windows_5.jpg
[image21]: ./output_images/multiscale_sliding_windows_6.jpg
[image22]: ./output_images/heatmap1.jpg
[image23]: ./output_images/heatmap2.jpg
[image24]: ./output_images/heatmap3.jpg
[image25]: ./output_images/heatmap4.jpg
[image26]: ./output_images/heatmap5.jpg
[image27]: ./output_images/heatmap6.jpg




## [Rubric](https://review.udacity.com/#!/rubrics/513/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  [Here](https://github.com/udacity/CarND-Vehicle-Detection/blob/master/writeup_template.md) is a template writeup for this project you can use as a guide and a starting point.  

You're reading it!
All project code is contained in the Jupyter notebook `vehicle_detection.ipynb`

### Histogram of Oriented Gradients (HOG)

#### 1. Explain how (and identify where in your code) you extracted HOG features from the training images.

I first had a look into the training data. I used the `vehicles` and `non-vehicles` datasets provided by Udacity, which contain 8.792 and 8.968 64x64 pictures respectively. Here is an example of some of the vehicle and non-vehicle images:

![alt text][image1]

I then used the function `get_hog_features()` to explore different color spaces and different `skimage.hog()` parameters (`orientations`, `pixels_per_cell`, and `cells_per_block`). I grabbed random images from each of the two classes and displayed them to get a feel for what the `skimage.hog()` output looks like.

Here is an example using the `LUV`, `HLS` and `HSV` color spaces and HOG parameters of `orientations=9`, `pixels_per_cell=(8, 8)`, `cells_per_block=(2, 2)`. I decided to apply a normalization scheme by setting `transf_sqrt=True`, which helps to reduce the effect of shadows or other illumination variation.


![alt text][image2]
![alt text][image5]

![alt text][image3]
![alt text][image6]

![alt text][image4]
![alt text][image7]

#### 2. Explain how you settled on your final choice of HOG parameters.

I tried various combinations of parameters and compared the performance (accuracy and train time) between them. I finally chose the following parameters:

* `color_space`: LUV
* `hog_channel`: ALL
* `orientation`: 9
* `pix_per_cell`: (8,8)
* `cells_per_block`: (2,2)
* `transf_sqrt`: True


#### 3. Describe how (and identify where in your code) you trained a classifier using your selected HOG features (and color features if you used them).

The code where the classifier is trained is contained in the cell titled "Train classifier".

I trained a linear SVM using HOG and color features, which were extracted from the images using the `extract_features()` function, contained in a cell titles "HOG and color features extraction" 

*  To extract the spatial features, I used the `bin_spatial()` function with the `spatial_size` parameter set to (16, 16).
*  To extract the color histogram of the images I used the `color_hist()` 
function. 

Here is an example of how do the spatial and color histogram features of two random images look like:

![alt text][image8]
![alt text][image9]


After training the classifier with the parameters mentioned above, it reached a test accuracy of 99.27%. The image below shows the prediction the classifier (successfully) made about two random images.

![alt text][image15]

### Sliding Window Search

#### 1. Describe how (and identify where in your code) you implemented a sliding window search. How did you decide what scales to search and how much to overlap windows?

I used the sliding window technic to identify vehicles in the images: I step across an image in a grid pattern and extract the features of each window, run the classifier and make a prediction on each step.

As discussed in the lectures, the size of the vehicles depends on their position on the image: vehicles close to the horizon appear smaller than vehicles closer to our car. I tried different window scales at different image positions, till I reached my final implementation, written in `find_cars_extended()` function.

Here I show the multi-scale sliding windows I use to identify vehicles on images:

![alt text][image10]
![alt text][image11]
![alt text][image12]
![alt text][image13]

#### 2. Show some examples of test images to demonstrate how your pipeline is working.  What did you do to optimize the performance of your classifier?

Ultimately I searched on four scales using LUV 3-channel HOG features plus spatially binned color and histograms of color in the feature vector, which provided a nice result. Here there are six example images:

![alt text][image16]
![alt text][image17]
![alt text][image18]
![alt text][image19]
![alt text][image20]
![alt text][image21]

---

### Video Implementation

#### 1. Provide a link to your final video output.  Your pipeline should perform reasonably well on the entire project video (somewhat wobbly or unstable bounding boxes are ok as long as you are identifying the vehicles most of the time with minimal false positives.)

Here's a [link to my video result](./result_project_video.mp4)


#### 2. Describe how (and identify where in your code) you implemented some kind of filter for false positives and some method for combining overlapping bounding boxes.

The code for combining overlapping bounding boxes and minimizing false positives is contained in the cell titled "Multiple detections and false positives".

##### Combining overlapping bounding boxes
I recorded the positions of positive detections in each frame of the video.  From the positive detections I created a heatmap and then threshold that map to identify vehicle positions. I then used `scipy.ndimage.measurements.label()` to identify individual blobs in the heatmap and then assumed each blob corresponded to a vehicle. I constructed bounding boxes to cover the area of each blob detected.  


Here are five frames, their corresponding heat maps and the resulting bounding boxes formed by `scipy.ndimage.measurements.label()` :

![alt text][image22]
![alt text][image23]
![alt text][image24]
![alt text][image25]
![alt text][image26]
![alt text][image27]

##### Filtering out false positives
I filter out false positives by determining which detections appear in one frame but not in the next ones.  This is done using the `DetectionHistory` class.

---

###  Submission Video

[![Vehicle Detection](http://img.youtube.com/vi/3FxIi_PnrIY/0.jpg)](https://youtu.be/3FxIi_PnrIY "ehicle Detection - Click to Watch!")


---

### Discussion

#### 1. Briefly discuss any problems / issues you faced in your implementation of this project.  Where will your pipeline likely fail?  What could you do to make it more robust?

In Computer Vision approaches, finding the correct trade-off of parameters is always a very challenging process that takes a lot of time. I would rather like to try a Deep Learning approach, with networks like YOLO (You Only Look Once) or SSD (Single Shot Detector), to solve this problem.

From my pipeline, I would like to reduce the tame it takes to process a video frame. The current frame-rate is around 0.7 sec/frame, which is not fast enough for real-time vehicle detection processes. Reducing the number of features or the number or windows to search in might improve the frame-rate, even though the detection accuracy could be affected.

