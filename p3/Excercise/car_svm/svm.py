#!/usr/bin/env python
import matplotlib
matplotlib.use('Qt5Agg')

import matplotlib.image as mpimg
import matplotlib.pyplot as plt
import numpy as np
import cv2
import glob
import time
from sklearn.svm import SVC
from sklearn.preprocessing import StandardScaler
from sklearn.model_selection import train_test_split


def color_hist(img, nbins=32, bins_range=(0, 256)):
    # Convert from RGB to HSV using cv2.cvtColor()
    hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    # Compute the histogram of the HSV channels separately
    h_hist = np.histogram(hsv[:,:,0], bins=nbins, range=bins_range)
    s_hist = np.histogram(hsv[:,:,1], bins=nbins, range=bins_range)
    v_hist = np.histogram(hsv[:,:,2], bins=nbins, range=bins_range)
    
    # Concatenate the histograms into a single feature vector
    features = np.concatenate((h_hist[0], s_hist[0], v_hist[0])).astype(np.float64)
    
    # Normalize the result
    # Return the feature vector
    norm_features = features / np.sum(features)
    return norm_features


def extract_features(imgs, hist_bins=32, hist_range=(0, 256)):
    # Create a list to append feature vectors to
    features = []
    # Iterate through the list of images
    for file in imgs:
        # Read in each one by one
        image = mpimg.imread(file)
        # Apply color_hist() 
        hist_features = color_hist(image, nbins=hist_bins, bins_range=hist_range)
        # Append the new feature vector to the features list
        features.append(hist_features)
    # Return list of feature vectors
    return features


# Read in car and non-car images
cars = glob.glob('./vehicles_smallset/*1/*.jpeg')
notcars = glob.glob('./non-vehicles_smallset/*1/*.jpeg')

# Extract features
hist_bins = 2
car_features = extract_features(cars, hist_bins=hist_bins)
notcar_features = extract_features(notcars, hist_bins=hist_bins)

X = np.concatenate((car_features, notcar_features)).astype(np.float64)
X_scaler = StandardScaler().fit(X)
scaled_X = X_scaler.transform(X)

# Define the labels vector
y = np.concatenate((np.ones(len(car_features)), np.zeros(len(notcar_features))))

# Split up data into randomized training and test sets
rand_state = np.random.randint(0, 100)
X_train, X_test, y_train, y_test = train_test_split(
    scaled_X, y, test_size=0.2, random_state=rand_state)


print('Dataset includes', len(cars), 'cars and', len(notcars), 'not-cars')
print('Using', hist_bins,'histogram bins')
print('Feature vector length:', len(X_train[0]))
# Use a linear SVC 
svc = SVC(kernel='linear')
# Check the training time for the SVC
t=time.time()
svc.fit(X_train, y_train)
t2 = time.time()
print(round(t2-t, 2), 'Seconds to train SVC...')
# Check the score of the SVC
print('Test Accuracy of SVC = ', round(svc.score(X_test, y_test), 4))
# Check the prediction time for a single sample
t=time.time()
n_predict = 10
print('My SVC predicts: \t', svc.predict(X_test[0:n_predict]))
print('For these',n_predict, 'labels: \t', y_test[0:n_predict])
t2 = time.time()
print(round(t2-t, 5), 'Seconds to predict', n_predict,'labels with SVC')
