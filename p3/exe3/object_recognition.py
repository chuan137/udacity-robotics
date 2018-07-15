#!/usr/bin/env python

import numpy as np
import sklearn
from sklearn.preprocessing import LabelEncoder

import pickle

from sensor_stick.srv import GetNormals
from sensor_stick.features import compute_color_histograms
from sensor_stick.features import compute_normal_histograms
from visualization_msgs.msg import Marker

from sensor_stick.marker_tools import *
from sensor_stick.msg import DetectedObjectsArray
from sensor_stick.msg import DetectedObject
from sensor_stick.pcl_helper import *

def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster

LEAF_SIZE = 0.01

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):
    # TODO: Convert ROS msg to PCL data
    data = ros_to_pcl(pcl_msg)

    # TODO: Voxel Grid Downsampling
    vox = data.make_voxel_grid_filter()
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
    data_ds = vox.filter()

    # TODO: PassThrough Filter
    axis_min = 0.76
    axis_max = 1.1
    filter_axis = 'z'
    passthrough = data_ds.make_passthrough_filter()
    passthrough.set_filter_field_name(filter_axis)
    passthrough.set_filter_limits(axis_min, axis_max)
    data_pt = passthrough.filter()

    # TODO: RANSAC Plane Segmentation
    seg = data_pt.make_segmenter()
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)
    max_distance = 0.01
    seg.set_distance_threshold(max_distance)
    inliers, coefficients = seg.segment()

    # TODO: Extract inliers and outliers
    #cloud_table = data_pt.extract(inliers, negative=False)
    cloud_objects = data_pt.extract(inliers, negative=True)

    # TODO: Euclidean Clustering
    white_cloud = XYZRGB_to_XYZ(cloud_objects)
    tree = white_cloud.make_kdtree()

    ec = white_cloud.make_EuclideanClusterExtraction()
    ec.set_ClusterTolerance(0.05)
    ec.set_MinClusterSize(20)
    ec.set_MaxClusterSize(5000)
    ec.set_SearchMethod(tree)

    cluster_indicies = ec.Extract()
    print (len(cluster_indicies))

    # # TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately
    # cluster_color = get_color_list(len(cluster_indices))
    # color_cluster_point_list = []
    # for j, indices in enumerate(cluster_indices):
    #     for i, indice in enumerate(indices):
    #         color_cluster_point_list.append([white_cloud[indice][0],
    #                                         white_cloud[indice][1],
    #                                         white_cloud[indice][2],
    #                                         rgb_to_float(cluster_color[j])])
    # cluster_cloud = pcl.PointCloud_PointXYZRGB()
    # cluster_cloud.from_list(color_cluster_point_list)

    # # TODO: Convert PCL data to ROS messages
    # ros_cloud_table = pcl_to_ros(cloud_table)
    # ros_cloud_objects = pcl_to_ros(cloud_objects)
    # ros_cloud_cluster = pcl_to_ros(cluster_cloud)


    # # TODO: Publish ROS messages
    # pcl_objects_pub.publish(ros_cloud_objects)
    # pcl_table_pub.publish(ros_cloud_table)
    # pcl_cluster_pub.publish(ros_cloud_cluster)


# Exercise-3 TODOs: 

    # Classify the clusters! (loop through each detected cluster one at a time)
    detected_objects_labels = []
    detected_objects = []

    for index, pts_list in enumerate(cluster_indicies):
        # Grab the points for the cluster from the extracted outliers (cloud_objects)
        pcl_cluster = cloud_objects.extract(pts_list)

        # Compute the associated feature vector
        # TODO: convert the cluster from pcl to ROS using helper function
        ros_cluster = pcl_to_ros(pcl_cluster)

        # Extract histogram features
        # TODO: complete this step just as is covered in capture_features.py
        chists = compute_color_histograms(ros_cluster, using_hsv=True)
        normals = get_normals(ros_cluster)
        nhists = compute_normal_histograms(normals)
        feature = np.concatenate((chists, nhists))

        # Make the prediction, retrieve the label for the result
        # and add it to detected_objects_labels list
        prediction = clf.predict(scaler.transform(feature.reshape(1,-1)))
        label = encoder.inverse_transform(prediction)[0]
        detected_objects_labels.append(label)
        print (prediction, label)

        # Publish a label into RViz
        label_pos = list(white_cloud[pts_list[0]])
        label_pos[2] += .4
        object_markers_pub.publish(make_label(label,label_pos, index))

        # Add the detected object to the list of detected objects.
        do = DetectedObject()
        do.label = label
        do.cloud = ros_cluster
        detected_objects.append(do)
    
    rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects_labels), detected_objects_labels))

    # Publish the list of detected objects
    # This is the output you'll need to complete the upcoming project!
    detected_objects_pub.publish(detected_objects)

if __name__ == '__main__':

    # TODO: ROS node initialization
    rospy.init_node('object_recognition', anonymous=True)

    # TODO: Create Subscribers
    pcl_sub = rospy.Subscriber("/sensor_stick/point_cloud", pc2.PointCloud2, pcl_callback, queue_size=1)

    # TODO: Create Publishers
    object_markers_pub = rospy.Publisher("/object_markers", Marker, queue_size=1)
    detected_objects_pub = rospy.Publisher("/detected_objects", DetectedObjectsArray, queue_size=1)

    # TODO: Load Model From disk
    model = pickle.load(open('model.sav', 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']

    # Initialize color_list
    get_color_list.color_list = []

    # TODO: Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()
