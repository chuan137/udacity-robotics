## Project: Perception Pick & Place

### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

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


### Pick and Place Setup

#### 1. For all three tabletop setups (`test*.world`), perform object recognition, then read in respective pick list (`pick_list_*.yaml`). Next construct the messages that would comprise a valid `PickPlace` request output them to `.yaml` format.

And here's another image! 
![demo-2](https://user-images.githubusercontent.com/20687560/28748286-9f65680e-7468-11e7-83dc-f1a32380b89c.png)

Spend some time at the end to discuss your code, what techniques you used, what worked and why, where the implementation might fail and how you might improve it if you were going to pursue this project further.  



