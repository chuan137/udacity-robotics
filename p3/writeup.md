## Project: Perception Pick & Place

### Writeup / README

[img1]: ./images/train_1.png
[img2]: ./images/train_2.png
[img3]: ./images/train_3.png
[step1]: ./images/step1_input.png
[step2]: ./images/step2_denoised.png
[step3]: ./images/step3_downsampled.png
[step4]: ./images/step4_segmented.png
[move1]: ./images/move_1.png
[move2]: ./images/move_2.png
[move3]: ./images/move_3.png

### Exercise 1, 2 and 3 pipeline implemented
**Exercise 1**: implement RANSAC.py (p3/exe1/RNASAC.py). 
A voxel grid filter is first used to downsample the input data, and a pass-through filter is used to cut the non-intrested volume. Finally, table is segmented with RANSA algorithm using a plane model (SACMODEL_PLANE). Parameter `max_distance` is the distance tolenrance that controls poits to be included in the model.

**Exercise 2**: implement function pcl_callback() in segmentation.py. (p3/exe2/segmentation.py)
The objects are segemented for further detection. Euclidean clustering (or density-based spatial cluster of applications with noise, DBSCAN) is a clustering algorithm  that creates clusters by grouping data points from their nearest neighbor. The algorithm is accelerated by using a k-d tree in search. Each detected object is assigned to a random color. And the segmented objects are publish to message '/pcl_cluster'.

**Exercise 3**: object detection with color and normal historgrams.
* Calculate histograms implemented in (p3/exe3/features.py). Add additional parameter `bins=32` with default value to methods `compute_color_histograms` and `compute_normal_histograms`.
*  Read the pick objects from files pick_list_*.yaml and set models in (p3/exe3/capture_features.py). A certain number of samples for each model is generated for training. Training data is taken with histograms with 128 bins.
* Train model with train_svm.py. A linear SVM model is used, with parameter C = 0.1.
* Object recognition pipeline in (p3/exe3/object_recognition.py). A loop over the segmented objects (starting from line 99). Feature is extracted both color and normals (line 109), and predction are made using the trained model (line 116). The trained model is loaded in the main method (line 151) before pipeline is started.


#### Capture features and train
After capture the features, train the model with p3/exe3/train_svm.py. I used a linear kernel, with C = 0.1 or 0.5. Below is the accuracies of the trained models.

* World 1, 50 samples, accuracy 98%
![train results for world 1][img1]
* World 2, 50 samples, accuracy 96%
![train results for world 2][img2]
* World 3, 200 samples, accuracy 86%
![train results for world 3][img3]

### Pick and Place Setup
Python code in p3/project.py. The requested parameters are in p3/output_*.yaml.

#### Object segmentation 
1. input point cloud from camera
![input][step1]

2. Denoise. I found that a mean k value of 20, and a standard deviation threshold of 0.1 provided the optimal outlier filtering.
![denoised][step2]

3. Downsample. I used an leaf size of 0.01 or 0.003. Using 0.01 is a good compromise of detail while reducing processing time, however, increasing the number would increase the model's accuracy. The image shows leaf size of 0.01.
![downsampled][step3]

4. Pass through filter. I used range -0.5 to 0.5 in Y dimension, and 0.6 to 1.0 in Z dimension. It crops the image for the intrested objects.

5. Segmentation. Use RANSAC as in exercise 3. I used max_distance 0.01.

6. Cluster. Use Euclidean clustering with tollerance 0.05, min cluster size 20 and max cluster size 50000. The clustering algorithm is accellerated with k-d tree search.
![segmented][step4]

#### 3. Object detection and move_it
Even though I use 200 samples in training, the object recognition in world 3 is very tricky, since the detection accuracy is relative low. I choosed to move the objects that are "correctly" detected, and continued to detection in the next iteration (line 179 to 205 in p3/project.py). I considered the object is correctly detected, when the label has not been detected and the label is unique in this iteration. So I used a global list `output_labels_list` to keep track of the detected labels.

My problem is I have to cope with a very slow laptop without graphic card. I generaly got 2~3 fps in moving simulation. And the robot arm cotroller fails to pick up the object even though it is requested. I took a few screenshots without recording a gif.
![move1][move1]
![move2]
![move3]

### Discussion
The objects are placed in fix position. The recognitions can be easier if the object can be viewed from different angles. E.g., move the objects by robot arm. Or rotate the robot body by small degree to get a different angle for recognitions.


