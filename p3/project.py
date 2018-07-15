#!/usr/bin/env python

# Import modules
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

import rospy
import tf
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import String
from pr2_robot.srv import *
from rospy_message_converter import message_converter
import yaml


# Helper function to get surface normals
def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster

# Helper function to create a yaml friendly dictionary from ROS messages
def make_yaml_dict(test_scene_num, object_name, arm_name, pick_pose, place_pose):
    yaml_dict = {}
    yaml_dict["test_scene_num"] = test_scene_num.data
    yaml_dict["object_name"] = object_name.data
    yaml_dict["arm_name"]  = arm_name.data
    yaml_dict["pick_pose"] = message_converter.convert_ros_message_to_dictionary(pick_pose)
    yaml_dict["place_pose"] = message_converter.convert_ros_message_to_dictionary(place_pose)
    return yaml_dict

# Helper function to output to yaml file
def send_to_yaml(yaml_filename, dict_list):
    data_dict = {'object_list': dict_list}
    print (data_dict)
    with open(yaml_filename, 'w') as outfile:
        yaml.dump(data_dict, outfile, default_flow_style=False)

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

# Exercise-2 TODOs:

    # Convert ROS msg to PCL data
    cloud_objects = ros_to_pcl(pcl_msg)
    
    # Statistical Outlier Filtering
    outlier_filter = cloud_objects.make_statistical_outlier_filter()
    outlier_filter.set_mean_k(20)
    outlier_filter.set_std_dev_mul_thresh(0.1)
    cloud_objects = outlier_filter.filter()

    # Voxel Grid Downsampling
    LEAF_SIZE = 0.002
    vox = cloud_objects.make_voxel_grid_filter()
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
    cloud_objects = vox.filter()

    # PassThrough Filter
    axis_min = 0.6
    axis_max = 1.0
    filter_axis = 'z'
    passthrough = cloud_objects.make_passthrough_filter()
    passthrough.set_filter_field_name(filter_axis)
    passthrough.set_filter_limits(axis_min, axis_max)
    cloud_objects = passthrough.filter()

    axis_min = -0.4
    axis_max = 0.4
    filter_axis = 'y'
    passthrough2 = cloud_objects.make_passthrough_filter()
    passthrough2.set_filter_field_name(filter_axis)
    passthrough2.set_filter_limits(axis_min, axis_max)
    cloud_objects = passthrough2.filter()

    # RANSAC Plane Segmentation
    seg = cloud_objects.make_segmenter()
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)
    max_distance = 0.01
    seg.set_distance_threshold(max_distance)
    inliers, coefficients = seg.segment()

    # Extract inliers and outliers
    cloud_objects = cloud_objects.extract(inliers, negative=True)

    # Euclidean Clustering
    white_cloud = XYZRGB_to_XYZ(cloud_objects)
    tree = white_cloud.make_kdtree()

    ec = white_cloud.make_EuclideanClusterExtraction()
    ec.set_ClusterTolerance(0.05)
    ec.set_MinClusterSize(20)
    ec.set_MaxClusterSize(50000)
    ec.set_SearchMethod(tree)

    cluster_indicies = ec.Extract()

    # Create Cluster-Mask Point Cloud to visualize each cluster separately
    colors = get_color_list(len(cluster_indicies))
    cloud_objects_cluster_list = []
    for i, pts_list in enumerate(cluster_indicies):
        pcl_cluster = cloud_objects.extract(pts_list)
        color = colors[i]
        cluster = XYZ_to_XYZRGB(pcl_cluster, color)
        cloud_objects_cluster_list.extend(cluster)

    cloud_objects_cluster = pcl.PointCloud_PointXYZRGB()
    cloud_objects_cluster.from_list(cloud_objects_cluster_list)

    # Convert PCL data to ROS messages
    ros_cluster = pcl_to_ros(cloud_objects_cluster)

    # Publish ROS messages
    p_cluster_pub.publish(ros_cluster)

# Exercise-3 TODOs:
    detected_objects_labels = []
    detected_objects = []

    # Classify the clusters! (loop through each detected cluster one at a time)
    for index, pts_list in enumerate(cluster_indicies):

        # Grab the points for the cluster
        pcl_cluster = cloud_objects.extract(pts_list)
        ros_cluster = pcl_to_ros(pcl_cluster)

        # Compute the associated feature vector
        chists = compute_color_histograms(ros_cluster, bins=128, using_hsv=True)
        normals = get_normals(ros_cluster)
        nhists = compute_normal_histograms(normals, bins=128)
        feature = np.concatenate((chists, nhists))

        # Make the prediction
        prediction = clf.predict(scaler.transform(feature.reshape(1,-1)))
        label = encoder.inverse_transform(prediction)[0]
        detected_objects_labels.append(label)

        # Publish a label into RViz
        label_pos = list(white_cloud[pts_list[0]])
        label_pos[2] += .4
        object_markers_pub.publish(make_label(label,label_pos, index))

        # Add the detected object to the list of detected objects.
        do = DetectedObject()
        do.label = label
        do.cloud = ros_cluster
        detected_objects.append(do)

    # Publish the list of detected objects
    rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects_labels), detected_objects_labels))
    detected_objects_pub.publish(detected_objects)

    # Suggested location for where to invoke your pr2_mover() function within pcl_callback()
    # Could add some logic to determine whether or not your object detections are robust
    # before calling pr2_mover()
    ground_truth = []
    object_list_param = rospy.get_param('/object_list')
    for object_param in object_list_param:
        ground_truth.append(object_param['name'])

    ground_truth_set = set(ground_truth)
    detected_objects_set = set(detected_objects_labels)
    n1 = len(ground_truth_set)
    n2 = len(detected_objects_set)

    # run pr2_mover only when requirments are met:
    # 100% (3/3) objects in test1.world
    # 80% (4/5) objects in test2.world
    # 75% (6/8) objects in test3.world

    print (ground_truth)
    print (n1, n2)

    if ( n1 == 3 and n2 == 3) or ( n1 == 5 and n2 >= 4) or ( n1 == 8 and n2 >= 6):
        try:
            pr2_mover(detected_objects)
        except rospy.ROSInterruptException:
            pass

# function to load parameters and request PickPlace service
def pr2_mover(object_list):
    test_scene_num = Int32()
    object_name = String()
    which_arm = String()
    pick_pose = Pose()
    place_pose = Pose()

    scene_num = 3
    test_scene_num.data = scene_num

    object_list_param = rospy.get_param('/object_list')
    dropbox_list_param = rospy.get_param('/dropbox')

    object_group_dict = {}
    for object_param in object_list_param:
        object_group_dict[object_param['name']] = object_param['group']

    for box_param in dropbox_list_param:
        if box_param['group'] == 'red':
            pos_red_box = box_param['position']
        else:
            pos_green_box = box_param['position']

    output_list = []

    # Loop through the pick list
    for object_val in object_list:
        object_name.data = object_val.label
        object_group = object_group_dict[object_val.label]

        # Get the PointCloud for a given object and obtain it's centroid
        points_array = ros_to_pcl(object_val.cloud).to_array()
        temp = np.mean(points_array, axis=0)[:3]
        centroid = [np.asscalar(x) for x in temp]

        # Pick pose
        pick_pose.position.x = centroid[0]
        pick_pose.position.y = centroid[1]
        pick_pose.position.z = centroid[2]

        # place_pose
        if object_group == 'red':
            which_arm.data = 'left'
            place_pose.position.x = pos_red_box[0]
            place_pose.position.y = pos_red_box[1]
            place_pose.position.z = pos_red_box[2]
        else:
            which_arm.data = 'right'
            place_pose.position.x = pos_green_box[0]
            place_pose.position.y = pos_green_box[1]
            place_pose.position.z = pos_green_box[2]

        obj_yaml_dict = make_yaml_dict(test_scene_num, object_name, which_arm, pick_pose, place_pose)
        output_list.append(obj_yaml_dict)


        # Wait for 'pick_place_routine' service to come up
        # rospy.wait_for_service('pick_place_routine')

        # try:
        #     pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)

        #     # Insert your message variables to be sent as a service request
        #     resp = pick_place_routine(test_scene_num, object_name, which_arm, pick_pose, place_pose)

        #     print ("Response: ",resp.success)


        # except rospy.ServiceException, e:
        #     print "Service call failed: %s"%e

    # Output your request parameters into output yaml file
    #send_to_yaml('output_%i.yaml' % scene_num, output_list)
    print (output_list)


if __name__ == '__main__':
    # TODO: ROS node initialization
    rospy.init_node('perception', anonymous=True)
    
    # TODO: Create Subscribers
    pcl_sub = rospy.Subscriber("/pr2/world/points", pc2.PointCloud2, pcl_callback, queue_size=1)

    # TODO: Create Publishers
    p_cluster_pub = rospy.Publisher("/p/cluster", pc2.PointCloud2, queue_size=1)
    object_markers_pub = rospy.Publisher("/object_markers", Marker, queue_size=1)
    detected_objects_pub = rospy.Publisher("/detected_objects", DetectedObjectsArray, queue_size=1)
    pr2_command_pub = rospy.Publisher('/pr2/world_joint_controller/command', Float64, queue_size=10)

    # TODO: Load Model From disk
    model = pickle.load(open('model.sav', 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']

    # Initialize color_list
    get_color_list.color_list = []

    # Rotate PR2 in place to capture side tables for the collision map
    # pr2_command_pub.publish(2*np.pi)     # Rotate the PR2 counter-clockwise 90 deg
    # pr2_command_pub.publish(-np.pi)      # Rotate the PR2 clockwise 180 deg
    # pr2_command_pub.publish(np.pi/2)     # Rotate the PR2 back to neutral position

    # TODO: Spin while node is not shutdown
    print ("perception node started")

    while not rospy.is_shutdown():
        rospy.spin()
