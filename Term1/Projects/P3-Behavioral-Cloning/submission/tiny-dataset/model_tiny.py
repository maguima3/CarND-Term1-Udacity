
import csv
import cv2
import numpy as np
from sklearn.utils import shuffle
from sklearn.model_selection import train_test_split
from random import randint
from math import ceil

from keras.models import Sequential
from keras.layers import Flatten, Dense, Lambda, Cropping2D
from keras.layers.convolutional import Conv2D
from keras.layers.pooling import MaxPooling2D
from keras.optimizers import Adam

from settings import preprocess_input

BATCH_SIZE = 5
STEERING_ANGLE_CORRECTION = 0.25

CENTER_CAMERA = 0
LEFT_CAMERA = 1
RIGHT_CAMERA = 2

###########
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt


DATA_PATH = '../../../../tiny/'

def read_training_data():
	samples = []
	with open(DATA_PATH + 'driving_log.csv', 'r') as csvfile:
	        reader = csv.reader(csvfile)
	        next(reader)
	        for line in reader:
	                samples.append(line)
	return samples

# n_camera refers to one of the three cameras located on the front of the car
# n_camera values can be. 0, 1 or 2, refering to the centered, left and right cameras respectively
def get_image_from_local_storage(n_camera, line):
        filename = line[n_camera].split('/')[-1]
        path = DATA_PATH + 'IMG/' + filename
        image = cv2.imread(path)
        return image

def random_camera(line):
        camera = randint(0, 3)
        if (camera == 0 or camera == 1): 
                image = get_image_from_local_storage(CENTER_CAMERA, line)
                angle = float(line[3])

        elif (camera == 2):
                image = get_image_from_local_storage(LEFT_CAMERA, line)
                angle = float(line[3]) + STEERING_ANGLE_CORRECTION
        else:
                image = get_image_from_local_storage(RIGHT_CAMERA, line)
                angle = float(line[3]) - STEERING_ANGLE_CORRECTION

        return image, angle

def random_flip(image, angle):
        if (randint(0,1)):
                return np.fliplr(image), -angle
        else:
                return image, angle

def random_translation(image, angle):
        translation_range = 30 # Pixels
        translation = random.uniform(-translation_range, translation_range)

        angle_correction = translation


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
                                image = preprocess_input(image)
                                images.append(image)
                                measurements.append(measurement)

                        X_train = np.array(images)
                        y_train = np.array(measurements)
                        yield shuffle(X_train, y_train)

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

samples = read_training_data()
train_samples, validation_samples = train_test_split(samples, test_size=0.5)

model = create_NVIDIA_model()

model.compile(loss='mse', optimizer=Adam(lr=0.0001))
model.fit_generator(train_generator(train_samples), \
                steps_per_epoch = ceil(len(train_samples)/BATCH_SIZE), \
                validation_data = val_generator(validation_samples), \
                validation_steps = ceil(len(validation_samples)/BATCH_SIZE), \
                epochs=30)


model.save('model_test.h5')
print('Model saved')
exit()