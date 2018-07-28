# Robotics Nanodegree Project 4 - Deep Learning

## Contents

* [ Architecture ](#architecture)


[learn1]: ./images/learn1.png
[learn2]: ./images/learn2.png
[learn3]: ./images/learn3.png
[seg1]: ./images/seg1.png
[seg2]: ./images/seg2.png
[loss1]: ./images/loss1.png
[loss2]: ./images/loss2.png




## Architecture <a name="achitecture"></a>

In this project, the object is to track target in a video stream. For this purpose, semantic segmentation will be used on each frame, that is, each image pixels are labeled as one of the classes in question. As introduced in the lectures, the Fully Convolutional Network (FCN) will be used to solve the semantic segmentation problem.


### Fully Convolutional Netowrk
As an extension to the classical CNN network, FCN consists of both encoder and decoder blocks. In both achitecture, the input data are fed into *encoder block*, which is consisted of essentially convolution layers with a small kernel and stride 2 or larger. After each encoder block, the image size is smaller. But the depth (or number of filters) is increased. The geometric infomation are encoded in representations of the filters, with increasing understanding of the features in the origianl image. 

```python
def encoder_block(input_layer, filters, strides):
    return separable_conv2d_batchnorm(input_layer, filters, strides)
```
Separable convolution is a technique to reduce the weight parameters. Assuming the images in the filters are independant, convolutions are first applied on the image, then in the depth dimension.


Next to the encoder block, FCN uses a 1x1 convolution layer in contrast to the fully connected layer in CNN. This layer is just a regular convolution with kernel size 1x1 and stride size 1. 

```python
# 1x1 Convolution layer using conv2d_batchnorm().
conv_norm = conv2d_batchnorm(input_layer=conv2, filters=256, kernel_size=1, strides=1)
```

The 1x1 convolution layer is then followed by the *decoder blocks*. The decoder block upsamples the image, turing the input image to larger size. At the same time, the depth is reduced. At the end of decoder blocks, the number of filters equals to the number of classes. We can then apply the sofmax and use the class that has the highest probablity to label the pixel. Before the convolution is applied to the upsampled image, it is concatenated with a image from prior encoder layer. This technique is effectively same as skip connection. It helps to regain the lost spatial information.

```python
def decoder_block(small_ip_layer, large_ip_layer, filters):
    upsampled_layer = bilinear_upsample(small_ip_layer)
    large_op_layer = layers.concatenate([upsampled_layer, large_ip_layer])
    output_layer = separable_conv2d_batchnorm(large_op_layer, filters)
    output_layer = separable_conv2d_batchnorm(output_layer, filters)
    return output_layer
```
Combine the encoder blocks, 1x1 convolution layer and decoder blocks together, we have the FCN model. I choosed the filter size to be 64 and 128 for the first and second encoder block. The filter size is 256 for the 1x1 convolution layer. And the filter size is 128 and 64 for the first and last decoder block.

```python
def fcn_model(inputs, num_classes):
    # encoder blocks
    conv1 = encoder_block(input_layer=inputs, filters=64, strides=2)
    conv2 = encoder_block(input_layer=conv1, filters=128, strides=2)

    # 1x1 Convolution layer using conv2d_batchnorm().
    conv_norm = conv2d_batchnorm(input_layer=conv2, filters=256, kernel_size=1, strides=1)
    
    # decoder blocks
    deconv1 = decoder_block(small_ip_layer=conv_norm, large_ip_layer=conv1, filters=128)
    deconv2 = decoder_block(small_ip_layer=deconv1, large_ip_layer=inputs, filters=64)
    
    # output layer: label pixels with softmax
    return layers.Conv2D(num_classes, 1, activation='softmax', padding='same')(deconv2)
  ```


#### Traning model
I use the provided dataset to train the above model on the p2.xlarge ec2 instances. The machine provides a Tesla K80 GPU card with 12 GB memory. It cannot process all the data at once. The maximum batch size to train the above model on this card is 64. In each epoch, the data is feed into the network once. And learning process is an iteration over the epochs. The learning rate controls the step size of gradient descent while optimizing the network weights in ech epoch. It thus controls the speed of convergence of the weights. If the learning rate is small, the learning of the network will take more epochs to achieve the same effect. However, the learning can't be choosen to be too big, the algorithm may miss the true minimal. 


The losses after 10 epochs with different learning rate. After some test, I found that the optimal value for the train loss and validation loss to converge is 0.001. 

Learning rate 0.005
![learn2]

Learning rate 0.001, loss = 0.0296, val_loss = 0.0476
![learn1]

Learning rate 0.0005, loss = 0.0346, val_loss = 0.0518
![learn3]


#### Validation result
I used the provided data to train the network. I used learning rate 0.001, steps per epoch 65, and number of epochs 50. I get final IoU 0.561 and final score 0.418.

![loss1]

I then added more collected data into the provided data set. The size of the dataset is about 5800 images. I continued to train the network with same learning rate for 20 epochs. The result is slightly improved. Final IoU is 0.568 and final score is 0.434.

![loss2]


![seg1]
![seg2]