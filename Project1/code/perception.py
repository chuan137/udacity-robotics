import numpy as np
import cv2

# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def color_thresh(img, rgb_thresh=(160, 160, 160)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:,:,0] > rgb_thresh[0]) \
                & (img[:,:,1] > rgb_thresh[1]) \
                & (img[:,:,2] > rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    # Return the binary image
    return color_select

def rock_mask(img, lower=(80,100,100), upper=(100,255,255)):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower, upper)
    return np.where(mask, 1, 0)

# Define a function to convert from image coords to rover coords
def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the 
    # center bottom of the image.  
    x_pixel = -(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[1]/2 ).astype(np.float)
    return x_pixel, y_pixel


# Define a function to convert to radial coords in rover space
def to_polar_coords(x_pixel, y_pixel):
    # Convert (x_pixel, y_pixel) to (distance, angle) 
    # in polar coordinates in rover space
    # Calculate distance to each pixel
    dist = np.sqrt(x_pixel**2 + y_pixel**2)
    # Calculate angle away from vertical for each pixel
    angles = np.arctan2(y_pixel, x_pixel)
    return dist, angles

# Define a function to map rover space pixels to world space
def rotate_pix(xpix, ypix, yaw):
    # Convert yaw to radians
    yaw_rad = yaw * np.pi / 180
    xpix_rotated = (xpix * np.cos(yaw_rad)) - (ypix * np.sin(yaw_rad))
                            
    ypix_rotated = (xpix * np.sin(yaw_rad)) + (ypix * np.cos(yaw_rad))
    # Return the result  
    return xpix_rotated, ypix_rotated

def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale): 
    # Apply a scaling and a translation
    xpix_translated = (xpix_rot / scale) + xpos
    ypix_translated = (ypix_rot / scale) + ypos
    # Return the result  
    return xpix_translated, ypix_translated


# Define a function to apply rotation and translation (and clipping)
# Once you define the two functions above this function should work
def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale):
    # Apply rotation
    xpix_rot, ypix_rot = rotate_pix(xpix, ypix, yaw)
    # Apply translation
    xpix_tran, ypix_tran = translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale)
    # Perform rotation, translation and clipping all at once
    x_pix_world = np.clip(np.int_(xpix_tran), 0, world_size - 1)
    y_pix_world = np.clip(np.int_(ypix_tran), 0, world_size - 1)
    # Return the result
    return x_pix_world, y_pix_world

# Define a function to perform a perspective transform
def perspect_transform(img, src, dst):
           
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))# keep same size as input image
    mask = cv2.warpPerspective(np.ones_like(img[:,:,0]), M, (img.shape[1], img.shape[0]))

    return warped, mask

def trusted_pixels(xpix, ypix, maxdist=50):
    # angle less than +/- 30 degree, distance less than 5 meters
    #maxangle = maxangel * np.pi / 180.0
    #trust = np.logical_and(dist <= maxdist, np.abs(angle) < maxangle)
    dist, _ = to_polar_coords(xpix, ypix)
    trust = dist < 50
    return np.where(trust)

# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    # Perform perception steps to update Rover()
    # TODO: 
    # NOTE: camera image is coming to you in Rover.img
    image = Rover.img
    # 1) Define source and destination points for perspective transform
    dst_size = 5
    bottom_offset = 6
    source = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
    destination = np.float32([[image.shape[1]/2 - dst_size, image.shape[0] - bottom_offset],
                  [image.shape[1]/2 + dst_size, image.shape[0] - bottom_offset],
                  [image.shape[1]/2 + dst_size, image.shape[0] - 2*dst_size - bottom_offset], 
                  [image.shape[1]/2 - dst_size, image.shape[0] - 2*dst_size - bottom_offset],
                  ])
    # 2) Apply perspective transform
    warped, mask = perspect_transform(image, source, destination)
    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    nav = color_thresh(warped, rgb_thresh=(160,160,160))
    obst = (np.ones_like(nav) - nav) * mask
    rock = rock_mask(warped)
    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
        # Example: Rover.vision_image[:,:,0] = obstacle color-thresholded binary image
        #          Rover.vision_image[:,:,1] = rock_sample clor-thresholded binary image
        #          Rover.vision_image[:,:,2] = navigable terrain color-thresholded binary image
    Rover.vision_image[:,:,1] = rock*255
    Rover.vision_image[:,:,0] = obst*255
    Rover.vision_image[:,:,2] = nav*255
    # 5) Convert map image pixel values to rover-centric coords
    nav_x_rover, nav_y_rover = rover_coords(nav)
    obst_x_rover, obst_y_rover = rover_coords(obst)
    rock_x_rover, rock_y_rover = rover_coords(rock)
    # 6) Convert rover-centric pixel values to world coordinates
    params_p2w = (Rover.pos[0], Rover.pos[1], Rover.yaw, Rover.worldmap.shape[0], 2*dst_size)
    nav_x_world, nav_y_world = pix_to_world(nav_x_rover, nav_y_rover, *params_p2w)
    obst_x_world, obst_y_world = pix_to_world(obst_x_rover, obst_y_rover, *params_p2w)
    rock_x_world, rock_y_world = pix_to_world(rock_x_rover, rock_y_rover, *params_p2w)
    # 7) Update Rover worldmap (to be displayed on right side of screen)
        # Example: Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
        #          Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
        #          Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1
        # Update world map with trusted pixels only
    if Rover.pitch < 0.5 or Rover.pitch > 359.5:
        obst_trusted = trusted_pixels(obst_x_rover, obst_y_rover, maxdist=200)
        nav_trusted= trusted_pixels(nav_x_rover, nav_y_rover, maxdist=30)
        Rover.worldmap[obst_y_world[obst_trusted], obst_x_world[obst_trusted], 0] += 1
        Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
        Rover.worldmap[nav_y_world[nav_trusted], nav_x_world[nav_trusted], 2] += 10
        #Rover.worldmap[obst_y_world, obst_x_world, 0] += 1
        #Rover.worldmap[nav_y_world, nav_x_world, 2] += 5
# 8) Convert rover-centric pixel positions to polar coordinates
    nav_r_rover, nav_a_rover = to_polar_coords(nav_x_rover, nav_y_rover)
    # Update Rover pixel distances and angles
        # Rover.nav_dists = rover_centric_pixel_distances
        # Rover.nav_angles = rover_centric_angles
    Rover.nav_dists = nav_r_rover
    Rover.nav_angles = nav_a_rover
    
    
    return Rover
