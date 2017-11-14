**Behavioral Cloning Project**

The goals / steps of this project are the following:

* Use the simulator to collect data of good driving behavior
* Build, a convolution neural network in Keras that predicts steering angles from images
* Train and validate the model with a training and validation set
* Test that the model successfully drives around track one without leaving the road
* Summarize the results with a written report


[//]: # (Image References)

[tiny-1]: ./submission/tiny-dataset/tiny-1.jpg 
[tiny-2]: ./submission/tiny-dataset/tiny-2.jpg 
[tiny-3]: ./submission/tiny-dataset/tiny-3.jpg
[tiny-4]: ./submission/tiny-dataset/tiny-4.jpg
[tiny-5]: ./submission/tiny-dataset/tiny-5.jpg
[balanced]: ./submission/balanced.png
[data-augmentation]: ./submission/data-augmentation.png 
[rec-1]: ./submission/rec-1.jpg 
[rec-2]: ./submission/rec-2.jpg 
[rec-3]: ./submission/rec-3.jpg 
[curv-1]: ./submission/curv-1.jpg 
[curv-2]: ./submission/curv-2.jpg 
[curv-3]: ./submission/curv-3.jpg

## Rubric Points
### Here I will consider the [rubric points](https://review.udacity.com/#!/rubrics/432/view) individually and describe how I addressed each point in my implementation.  

---
### Files Submitted & Code Quality

#### 1. Submission includes all required files and can be used to run the simulator in autonomous mode

My project includes the following files:

* model.py containing the script to create and train the model
* drive.py for driving the car in autonomous mode
* model.h5 containing a trained convolution neural network 
* writeup.md summarizing the results

#### 2. Submission includes functional code
Using the Udacity provided simulator and my drive.py file, the car can be driven autonomously around the track by executing 
```sh
python drive.py model.h5
```

#### 3. Submission code is usable and readable

The model.py file contains the code for training and saving the convolution neural network. The file shows the pipeline I used for training and validating the model, and it contains comments to explain how the code works.

### Model Architecture and Training Strategy

#### 1. An appropriate model architecture has been employed

In this project, my model is based on the convolutional neural network developed by NVIDIA on "End to End Learning for Self-Driving Cars", which has proved to be very successful in mapping raw pixels from a single front-facing camera directly to steering commands.

My network consists of 9 layers: 1 lambda layer to normalize the data, 5 convolutional layers, that perform feature extraction, and 3 fully connected layers, that function as a controller for steering.

The table bellow shows the architecture of the CNN in detail:

| Layer | Description|
| :---: | | :---: |
| Input | 3@66x200|
| Convolution | kernel: 5x5, stride: 2x2, output: 24@31x98|
| Convolution | kernel: 5x5, stride: 2x2, output: 36@14x47|
| Convolution | kernel: 5x5, stride: 2x2, output: 48@5x22|
| Convolution | kernel: 3x3, stride: 2x2, output: 64@3x20|
| Flatten | output: 3840|
| Fully Connected | output: 1164|
| Fully Connected | output: 100|
| Fully Connected | output: 50|
| Output | output: 1|

Before entering in the network, images caputred by the front cameras are preprocessed (model.py function ``preprocess_input``):

* First, images are cropped so as the elements that might obstruct the learning process are removed (such as trees, birds, mountains, etc)
* Afterwards, images are resized from 160x320 to 66x200 pixels, to meet the input size of the NVIDIA CNN
* Finally, images are converted from RGB to YUV 

#### 2. Attempts to reduce overfitting in the model

On each iteration of the training process, the training data is randomly augmented on the fly: images are randomly flipped and translated. This helps to prevent overfitting, as the model is trained with "new" data every epoch.

Moreover, the model was trained and validated on different data sets to ensure that the model was not overfitting, which has been successfully tested by running it through the simulator and ensuring that the vehicle stays on the track.

#### 3. Model parameter tuning

The learning rate to firstly train the model was 0.0001. In the following "refinement" processes, the learning rate was one order of magnitude smaller, 0.00001, as I used the refinement mode to correct the model from little mistakes, such as not turning enough on curves.

The ideal number of epochs to train the model was 5, and the batch size 128. More epochs didn't seem to improve the behavior of the network. In refinement mode, 10-15 epochs were enough to fine tune the model, and I usually set the batch size to 32 or 64, according to the size of the refinement dataset.


#### 4. Appropriate training data

The training data I used to train the model was the dataset provided by Udacity, which mostly contains center lane driving data. However, only this dataset was not enough to "teach" the vehicle to stay driving on the road the whole time. I tried multiple combination of parameters and data augmentation processes, but nothing seemed to be perfect even though the vehicle was performing well on most of the track.

Following the suggestion of other students of the Nanodegree, I decided to "refine" my model: instead of trying to find the perfect model every time from scratch, I selected a fairly good model and fine tuned it, making small modifications to it in order to improve it and reduce its mistakes. So, in addition to the dataset provided by Udacity, I recorded some other data to train the network to better curve to the right and recover from the sides of the road.

For details about how I created the training data, see the next section. 

### Model Architecture and Training Strategy

#### 1. Solution Design Approach

The first thing I do to find out a suitable architecture for a model, is to check out if there is already one that performs something similar to what I need. In this case, NVIDIA already developed a model that was able to find out the correct steering angles from the raw pixels of a front-facing camera, which is exactly what I wanted. Therefore, I've tried to follow NVIDIA's approach as much as possible in this project.

To be sure that the approach was correct, I first trained the network with very few data. I carefully selected the images so that the dataset contained meaningful data: one image represented center lane driving, two images sharp curving and two soft curving. The result was a dataset of just 15 images (5 images per each of the three front cameras). Then I duplicated the data and trained the model. The result was surprisingly good! The vehicle, although quite unsmoothly, was able to drive around the racetrack without leaving the road! So I decided to keep using the NVIDIA architecture and tried to develop a more general model, as I believe that a good model should not memorize the training data, but be able to generalize it and make good predictions in new scenarios.

These are the 5 images captured from the center camera used as first try to check my model:

![alt text][tiny-1]
![alt text][tiny-3]
![alt text][tiny-4]
![alt text][tiny-2]
![alt text][tiny-5]

I then trained the model with the dataset provided by Udacity and, as there were a few spots where the vehicle fell off the track, I refined it till the behavior was as expected: the vehicle is able to drive smoothly around the track without leaving the road.

#### 2. Final Model Architecture

The final model architecture is described above.

#### 3. Creation of the Training Set & Training Process

I've mainly used the dataset provided by Udacity to train my network. It contains (good) center lane driving behavior. The vehicle drives 3 times clockwise and 3 times counterclockwise arond Track 1, which makes this dataset balanced with respect to of steering angles.

![alt text][balanced]

Data is then randomly shuffled and splitted into training (80%) and validation (20%) data sets.

To augment the training dataset, images and angles are randomly flipped and translated (horizontally and vertically) during the training process, thinking that this would help the model to be more general (e.g vertical translation can help it to deal with slopes in other tracks)

![alt text][data-augmentation]


To refine the model, I recorded the vehicle recovering from the left side and right sides of the road back to center so that the vehicle would learn how to do it. These images show what a recovery looks like:

![alt text][rec-1]
![alt text][rec-2]
![alt text][rec-3]

I also recorded the vehicle driving along sharp curves, to improve its behavior:

![alt text][curv-1]
![alt text][curv-2]
![alt text][curv-3]

In the validation set, I just include images recorded from the center camera, and I do not perform any data augmentation process. The validation accuracy helps me determine the behavior the vehicle will have on the track.