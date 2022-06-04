#Libraries to import for function calls
import csv
import os
import cv2
import numpy as np
import sklearn
import matplotlib.pyplot as plt


#Code to read in the driving_log.csv file from the simulation
#Will probably need to read in mutiple driving logs, as I have 4 different driving logs from different events
car_angles = []
steering_angles = []
samples = []
with open('./Simulation_Recordings/Course_1_Simulation/driving_log.csv' , 'r') as csvfile: #First CSV FIle path
    reader = csv.reader(csvfile)
    for line in reader:
            samples.append(line)

with open('./Simulation_Recordings/Course_2_Simulation/driving_log.csv', 'r') as csvfile: #Second CSV FIle path
    reader = csv.reader(csvfile)
    for line in reader:
        samples.append(line)

with open('./Simulation_Recordings/Lane_Correct_Simulation/driving_log.csv' , 'r') as csvfile: #Third CSV FIle path
    reader = csv.reader(csvfile)
    for line in reader:
        samples.append(line)

with open('./Simulation_Recordings/Smooth_Curve_Simulation/driving_log.csv' , 'r') as csvfile: #Fourth CSV FIle path
    reader = csv.reader(csvfile)
    for line in reader:
            samples.append(line)

#split up training and validation samples
from sklearn.model_selection import train_test_split
train_samples, validation_samples = train_test_split(samples, test_size=0.2) #need to correct variable used "samples"


def generator(samples, batch_size=32, filepath= './Simulation_Recordings):
    num_samples = len(samples)
    correction = 0.2 # this is a parameter to tune
    while 1: # Loop forever so the generator never terminates
        shuffle(samples)
        for offset in range(0, num_samples, batch_size):
            batch_samples = samples[offset:offset+batch_size]
            images = []
            angles = []
            for batch_sample in batch_samples:
        #Collect Center Images & Center Image Steering Angle
                name = filepath + batch_sample[0].split('/Simulation_Recordings')[-1]
                center_image = cv2.imread(name)
                center_angle = float(batch_sample[3])
                images.append(center_image)
                angles.append(center_angle)
        #Create and collect inverted center images & center image steering angles for more data
        images.append(cv2.flip(center_image,1))
        angles.append(center_angle*-1.0)
        #Collect left images & left image steering angles
                name = filepath + batch_sample[1].split('/Simulation_Recordings')[-1]
                left_image = cv2.imread(name)
                left_angle = float(batch_sample[3] + correction) 
                images.append(left_image)
                angles.append(left_angle)
        #Create and collect inverted left images & left image steering angles for more data
        images.append(cv2.flip(left_image,1))
        angles.append(left_angle*-1.0)
        #Collect right images & right image steering angles
                name = filepath + batch_sample[2].split('/Simulation_Recordings')[-1]
                right_image = cv2.imread(name)
                right_angle = float(batch_sample[3] - correction)
                images.append(right_image)
                angles.append(right_angle)
        #Create and collect inverted right images & right image steering angles for more data
        images.append(cv2.flip(right_image,1))
        angles.append(center_angle*-1.0)

            # trim image to only see section with road
            X_train = np.array(images)
            y_train = np.array(angles)
            yield sklearn.utils.shuffle(X_train, y_train)


# compile and train the model using the generator function
train_generator = generator(train_samples, batch_size=192) #do i need to change the batch size back to 32?
validation_generator = generator(validation_samples, batch_size=192) #do i need to change the batch size back to 32?

#Import Keras libraries to utilize easy classifier functions
from keras.models import Sequential, Model
from keras.layers import Flatten, Dense, Lambda, Convolution2D, Cropping2D
from keras.layers.convolutional import Convolution2D
from keras.layers.pooling import MaxPooling2D


#Create Regression Model
model = Sequential()
#Lambda Layer to process the image before hand, i.e. normalize the pixel values
model.add(Lambda(Lambda x: (x/255.0) - 0.5, input_shape=(3,160,320))) #They did x/127.5 -1 in another example
#Cropping Layer, crops down 70 pixels from top, crops 25 pixels from bottom, and zero pixels from either side
model.add(Cropping2D(cropping=((70,25), (0,0)), input_shape=(160,320,3)))
#Regression steps
#Use dropout or pooling layers to reduce overfitting
#Use fewer convolutions or fewer fully connected layers to reduce overfitting
model.add(Convolution2D(24,5,5,subsample=(2,2),activation="relu"))
model.add(Convolution2D(36,5,5,subsample=(2,2),activation="relu"))
model.add(Convolution2D(48,5,5,subsample=(2,2),activation="relu"))
model.add(Convolution2D(64,3,3,activation="relu"))
model.add(Convolution2D(64,3,3,activation="relu"))
model.add(Flatten())
model.add(Dense(100))
model.add(Dense(50))
model.add(Dense(10))


#Validate the model
model.compile(loss = 'mse', optimizer = 'adam')

#Plot the results
history_object = model.fit_generator(train_generator, steps_per_epoch =
    len(train_samples)/batch_size, validation_data = 
    validation_generator,
    validation_steps = len(validation_samples)/batch_size, 
    nb_epoch=5, verbose=1)

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

#save model and exit
#must save model with name "model.h5", so the video file can find this specific file and run it
model.save('model.h5')
exit()
