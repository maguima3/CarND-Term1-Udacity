import csv
import cv2
import numpy as np
from sklearn.utils import shuffle
from sklearn.model_selection import train_test_split
from random import randint, uniform
import math

from keras.models import load_model, Sequential
from keras.layers import Flatten, Dense, Lambda, Cropping2D
from keras.layers.convolutional import Conv2D
from keras.layers.pooling import MaxPooling2D
from keras.optimizers import Adam

from preprocess import preprocess_input

import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt


REFINEMENT_MODE = False

STEERING_ANGLE_CORRECTION = 0.2 

CENTER_CAMERA = 0
LEFT_CAMERA = 1
RIGHT_CAMERA = 2

BATCH_SIZE = 128
DATA_PATH = '../../../../data/' 
EPOCHS = 5

def read_training_data():
    samples = []
    with open(DATA_PATH + 'driving_log.csv', 'r') as csvfile:
            reader = csv.reader(csvfile)
            next(reader)
            for line in reader:
                    samples.append(line)
    return samples

def random_camera(line):
    camera = randint(0, 2)
    if (camera == 0): 
            image = get_image_from_local_storage(CENTER_CAMERA, line)
            angle = float(line[3])
    elif (camera == 1):
            image = get_image_from_local_storage(LEFT_CAMERA, line)
            angle = float(line[3]) + STEERING_ANGLE_CORRECTION
    elif (camera == 2):
            image = get_image_from_local_storage(RIGHT_CAMERA, line)
            angle = float(line[3]) - STEERING_ANGLE_CORRECTION
    return image, angle

### n_camera refers to one of the three cameras located on the front of the car
### n_camera values can be. 0, 1 or 2, refering to the centered, left and right cameras respectively
def get_image_from_local_storage(n_camera, line):
    filename = line[n_camera].split('/')[-1]
    path = DATA_PATH + 'IMG/' + filename
    image = cv2.imread(path)
    return image

### Flips image randomly. Angle changes accordingly
def random_flip(image, angle):
    if (randint(0,1)):
            return np.fliplr(image), -angle
    else:
            return image, angle   

### Random horizontal translation
def random_h_translation(image, angle):
    if (randint(0,1)):
        h_translation_range = 25

        translation = uniform(-h_translation_range, h_translation_range)
        M = np.float32([[1,0,translation],[0,1,0]])
        rows, cols = image.shape[0], image.shape[1]

        image_translated = cv2.warpAffine(image, M, (cols,rows))
        angle_corrected = angle + (translation/h_translation_range) * 0.1

        return image_translated, angle_corrected, translation

    else:
        return image, angle, 0

### Random vertical translation
def random_v_translation(image, angle):
    if (randint(0,1)):
        v_translation_range = 10

        translation = uniform(-v_translation_range, v_translation_range)
        M = np.float32([[1,0,0],[0,1,translation]])
        rows, cols = image.shape[0], image.shape[1]

        image_translated = cv2.warpAffine(image, M, (cols, rows))
        angle_corrected = angle + (translation/v_translation_range) * 0.05

        return image_translated, angle_corrected, translation
    else:
        return image, angle, 0

### Crops image after translation. Does not include black masks
def crop(image, h_translation, v_translation):
    rows, cols = image.shape[0], image.shape[1]

    if h_translation >= 0:
        h_init = int(math.ceil(h_translation))
        h_end = cols
    else:
        h_init = 0
        h_end = cols + int(math.floor(h_translation))

    if v_translation >= 0:
        v_init = int(math.ceil(v_translation))
        v_end = rows
    else:
        v_init = 0
        v_end = rows + int(math.floor(v_translation))
    
    image = image[v_init:v_end, h_init:h_end]
    return image


### Generator to train the model
### Data is randomly augmented on the fly - helps to generalize the model
def train_generator(lines):
    num_lines = len(lines)
    while(1):
        shuffle(lines)
        for offset in range(0, num_lines, BATCH_SIZE):
            batch_lines = lines[offset:offset+BATCH_SIZE]

            images = []
            measurements = []
            for batch_line in batch_lines:
                image, measurement = random_camera(batch_line)
                image, measurement = random_flip(image, measurement)
                (image, measurement, h_translation) = random_h_translation(image, measurement)
                (image, measurement, v_translation) = random_v_translation(image, measurement)
                image = crop(image, h_translation, v_translation)
                image = preprocess_input(image)
                images.append(image)
                measurements.append(measurement)

            X_train = np.array(images)
            y_train = np.array(measurements)
            yield shuffle(X_train, y_train)

### Generator to validate the model
def val_generator(lines):
    num_lines = len(lines)
    while(1):
        shuffle(lines)
        for offset in range(0, num_lines, BATCH_SIZE):
            batch_lines = lines[offset:offset+BATCH_SIZE]

            images = []
            measurements = []
            for batch_line in batch_lines:
                filename = batch_line[CENTER_CAMERA].split('/')[-1]
                path = DATA_PATH + 'IMG/' + filename
                image = cv2.imread(path)
                image = preprocess_input(image)
                images.append(image)
                measurement = float(batch_line[3])
                measurements.append(measurement)

            X_val = np.array(images)
            y_val = np.array(measurements)
            yield shuffle(X_val, y_val)

### Model developed by NVIDIA in "End to End Learning for Self-Driving Cars"
def create_NVIDIA_model():
    model = Sequential()
    model.add(Lambda(lambda x: (x / 255.0) - 0.5, input_shape=(66, 200, 3)))
    model.add(Conv2D(24, (5, 5), activation='relu', strides=(2, 2)))
    model.add(Conv2D(36, (5, 5), activation='relu', strides=(2, 2)))
    model.add(Conv2D(48, (3, 3), activation='relu'))
    model.add(Conv2D(64, (3, 3), activation='relu'))
    model.add(Flatten())
    model.add(Dense(1164))
    model.add(Dense(100))
    model.add(Dense(50))
    model.add(Dense(1))
    return model


if REFINEMENT_MODE:
    # Performs small improvements to the model
    print('-------- REFINEMENT MODE --------')
    model = load_model('model.h5')
    learning_rate = 0.00001
    test_size = 0.5

else:
    model = create_NVIDIA_model()
    learning_rate = 0.0001
    test_size = 0.2

samples = read_training_data()
train_samples, validation_samples = train_test_split(samples, test_size=test_size)

model.compile(loss='mse', optimizer=Adam(lr=learning_rate))
history_object = model.fit_generator(train_generator(train_samples), \
                steps_per_epoch = math.ceil(len(train_samples)/BATCH_SIZE), \
                validation_data = val_generator(validation_samples), \
                validation_steps = math.ceil(len(validation_samples)/BATCH_SIZE), \
                epochs=EPOCHS, verbose=1)

model.save('model.h5')
print('Model saved')


### print the keys contained in the history object
print(history_object.history.keys())

### plot the training and validation loss for each epoch
plt.plot(history_object.history['loss'])
plt.plot(history_object.history['val_loss'])
plt.title('model mean squared error loss')
plt.ylabel('mean squared error loss')
plt.xlabel('epoch')
plt.legend(['training set', 'validation set'], loc='upper right')
plt.show()

exit()