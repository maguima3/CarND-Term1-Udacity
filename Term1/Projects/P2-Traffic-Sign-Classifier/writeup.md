#**Traffic Sign Recognition** 

---

**Build a Traffic Sign Recognition Project**

The goals / steps of this project are the following:
* Load the data set (see below for links to the project data set)
* Explore, summarize and visualize the data set
* Design, train and test a model architecture
* Use the model to make predictions on new images
* Analyze the softmax probabilities of the new images
* Summarize the results with a written report


[//]: # (Image References)

[dataset]: ./submission/dataset “Random dataset sample”
[barchart]: ./submission/distribution "Distribution of traffic signs by class"
[jitteredImage]: ./examples/random_noise.jpg “Random jittered image"
[uniform_barchart]: ./submission/uniform_dataset.jpg "Distribution of the training set after the extension process"
[grayscale]: ./submission/jittered.jpg “Processed Jittered Image”
[test1]: ./traffic-signs-test/test1.jpg "Traffic Sign 1”
[test2]: ./traffic-signs-test/test2.jpg "Traffic Sign 2”
[test3]: ./traffic-signs-test/test3.jpg "Traffic Sign 3”
[test4]: ./traffic-signs-test/test4.jpg "Traffic Sign 4”
[test5]: ./traffic-signs-test/test5.jpg "Traffic Sign 5"
[prob_test1]: ./submission/prob_test1.jpg “Softmax Probabilities for Traffic Sign 1”
[prob_test2]: ./submission/prob_test2.jpg "Softmax Probabilities for Traffic Sign 2”
[prob_test3]: ./submission/prob_test3.jpg "Softmax Probabilities for Traffic Sign 3”
[prob_test4]: ./submission/prob_test4.jpg "Softmax Probabilities for Traffic Sign 4”
[prob_test5]: ./submission/prob_test5.jpg "Softmax Probabilities for Traffic Sign 5"
[conv1]: ./submission/conv1.jpg “16 Feature Maps - First Conv Layer”
[conv2]: ./submission/conv2.jpg “32 Feature Maps - Second Conv Layer”
---



###Data Set Summary & Exploration

####1. Summary of the data set

I used the numpy library to calculate summary statistics of the traffic
signs data set:

* The size of training set is 34.799
* The size of the validation set is 4.410
* The size of test set is 12.630
* The shape of a traffic sign image is 32x32 pixels
* The number of unique classes/labels in the data set is 43

####2. Visualization of the dataset.

The dataset contains traffic signals of the German Traffic Signs dataset. As mentioned before, the traffic signs are categorized into 43 different classes, according to what their represent. Here are some examples of the images:

![alt text][dataset]


The bar charts bellow show the number of traffic sign samples of each class presented in each of the data sets.

![alt text][barchart]


###Design and Test a Model Architecture

####1. Preprocessing the image data

ConvNets architectures have built-in invariance to small translations, scaling and rotations. It has been proven that adding these transformations synthetically to the data set yields more robust learning to potential deformations in the test set.

Moreover, one can observe in the chart above that the number of samples of each class is fairly unbalanced, and some classes are represented to significantly lower extend than other, which might difficult their classification.

For these reasons, I decided to extend the training data set with jittered images. Images are randomly perturbed in:
* Position ([-2,2] pixels)
* Rotation ([-15,+15] degrees)

Here is an example of an original image and an augmented image:
![alt text][jitteredImage]

After the jittered process,  each class of the training set was composed by 2.500 samples.

![alt text][uniform_barchart]

Afterwards, I decided to convert the images to grayscale because as Pierre Sermanet and Yann LeCun mentioned in their paper, using no color is often better than using color.

As a last step, I normalized the image data to remove amplitude variation and only focus on the underlying distribution shape.

Here is an example of a jittered traffic sign image before and after grayscaling and normalization.

![alt text][grayscale]

To sum up, the difference between the original data set and the augmented data set is the following:
* Images are uniformly distributed (2.500 traffic sign samples per class)
* Data is normalized: zero mean and unit variance
* Images are in grayscale: only one channel is needed to represent the images (instead of three)


####2. Model Architecture


My final model consisted of 3 convolutional layers and 2 fully connected layers, inspired by LeNet and Pierre Sermanet/Yann LeCun model.

| Layer         		|     Description	        				| 
|:--------------------- :|:---------------------------------------------:	| 
| Input         		| 32x32x1 gray image   					| 
| Convolution1 5x5     	| 1x1 stride, same padding, outputs 30x30x16 	|
| RELU				|								|
| Max pooling	      	| 2x2 stride,  outputs 16x16x16 			|
| Dropout		      	| 								|
| Convolution2 5x5		| 1x1 stride, same padding, outputs 14x14x32	|
| RELU				|								|
| Max pooling	      	| 2x2 stride,  outputs 8x8x32				|
| Dropout		      	| 								|
| Convolution3 5x5     	| 1x1 stride, same padding, outputs 6x6x64		|
| RELU				|								|
| Max pooling	      	| 2x2 stride,  outputs 4x4x64				|
| Dropout		      	| 								|
| Fully connected1		| input 1024, output: 256        			|
| RELU				|								|
| Dropout		      	| 								|
| Fully connected2		| input 256, output: 84        				|
| RELU				|								|
| Dropout		      	| 								|
| Output layer		| input 84, output: 43        				|
| Softmax			| 	        						|


####3.Training

To train the model, I used the Adam optimizer, which is more sophisticated than the SGD and it has proven to perform well in this kind of situations.

The number of epochs of my model is 10 because it offers a good compromise between the performance of the algorithm and the time it takes to train it. The batch size is 128.

I chose the keep probability in the dropout layers to be 0.85. Less keep probability resulted in low accuracy (as the training data only runs 10 times through the net), and higher keep probability in overfitting.

Finally, I used a learning rate of 0.001 to train the model.


####4. Description of the approach taken for finding a solution and getting the validation set accuracy to be at least 0.93

My model architecture was first based on LeNet: 2 convolutional layers + 2 fully connected layer + 1 output layer. After training the model with different combination of hyperparameters, the validation accuracy was never higher than 87%. Inspired by Sermanet/Yann LeCun paper, I decided to add one more convolutional layer, and the performance of the network notably increased.

However, although the accuracy obtained on the  training and validation sets was fairly high (around 97-99%), the accuracy on the test set wasn’t: I was overfitting my model. I solved this issue by adding dropout layers.

After this process, my final model results are:
* validation set accuracy of 96.6%
* test set accuracy of 95.0%
 

###Test a Model on New Images

####1. Description of Possible Difficulties

Here are five German traffic signs that I found on the web:

![alt text][test1] ![alt text][test2] ![alt text][test3] 
![alt text][test4] ![alt text][test5]

As a first try, I wanted to evaluate the “happy path” performance of my model. That’s why I chose 5 images that should not present any problem on the classification.


####2. Model's predictions on new traffic signs
Discuss the model's predictions on these new traffic signs and compare the results to predicting on the test set. At a minimum, discuss what the predictions were, the accuracy on these new predictions, and compare the accuracy to the accuracy on the test set (OPTIONAL: Discuss the results in more detail as described in the "Stand Out Suggestions" part of the rubric).

Here are the results of the prediction:

| Image					|     Prediction	        		| 
|:------------------------------:	|:----------------------------------:| 
| Yield      				| Yield	   				| 
| Priority Sign     			| Priority Sign 				|
| End of all speed and passing limits	| End of speed limit (80km/h)		|
| Road Work	      				| Road Work					|
| No entry					| No entry		      			|


The model was able to correctly guess 4 of the 5 traffic signs, which gives an accuracy of 80%. I believe this result is not as relevant as the accuracy obtained on the test set (95.0%), since it has been obtained from the classification of only five images (in comparison with the 12.630 of the test set), but it has helped me to reach an interesting conclusion: 

I believe that the model did not correctly classify the “End of all speed and passing limits” because the original number of samples provided in the original data set was low (only 210 samples). Even though the training set was extended to 2.500 samples with rotated and shifted transformations of the original images, it seems not have been sufficient. I am confident that improving the data augmentation process  will lead to better results in the future.

####3. Certainty of the model - Softmax probabilities


![alt text][prob_test1]
![alt text][prob_test2] 
![alt text][prob_test3] 
![alt text][prob_test4]
![alt text][prob_test5]


### Visualizing the Neural Network
####1. Discuss the visual output of your trained network's feature maps. What characteristics did the neural network use to make classifications?

![alt text][conv1]
![alt text][conv2]

The first layer contains 16 filters that can detect very basic pixel patterns, like edges and lines. These basic filters are then used by subsequent layers as building bricks to construct detectors of more complicated patterns and figures.

#### More Information

For more details about the implementation, you can check the jupyter notebook with the code [here] (https://github.com/maguima3/Udacity-Self-Driving-Car-Nanodegree/blob/master/Term1/Projects/P2-Traffic-Sign-Classifier/Traffic_Sign_Classifier.ipynb)

