from geometry_msgs.msg import Pose, Point, Quaternion

import numpy as np
import cv2

def ProcessMap(occupancy_grid, target_point):
    extended_grid = ExtendMap(occupancy_grid, target_point)
    
    inverted_grid = InvertMap(extended_grid)

    eroded_grid = ErodeMap(inverted_grid)
    
    # square_grid = SquareMap(eroded_grid)

    return eroded_grid

def InvertMap(occupancy_grid):
    '''
    Inverts map (i.e. all points above 30% probability of being occupied become 0, and all others become 1)
    This is done since the pathfinding library wants 0 as occupied, 1 as not
    '''
    to_invert = np.copy(occupancy_grid.data).astype('uint8')

    for i in range(to_invert.shape[0]):
        for l in range(to_invert.shape[1]):
            if to_invert[i, l] > 40 and to_invert[i, l] != 255:
                to_invert[i, l] = 0
            else:
                to_invert[i, l] = 255

    occupancy_grid.data = to_invert

    return occupancy_grid 

def ExtendMap(occupancy_grid, target_point):
    '''
    Extends the map such that the grid contains the target point
    i.e. adds rows/columns as needed such that the target point can be expressed as a location in the grid
    Done so pathfinding can be run on the resulting grid
    Marks all new points as 1 (unoccupied)
    '''
    origin_x = occupancy_grid.info.origin.position.x
    origin_y = occupancy_grid.info.origin.position.y    
    occupancy_grid.data = np.reshape(occupancy_grid.data, (occupancy_grid.info.height, occupancy_grid.info.width))

    if(target_point.x < occupancy_grid.info.origin.position.x):
        to_add = np.ones((int((occupancy_grid.info.origin.position.x - target_point.x)/occupancy_grid.info.resolution) + 5 , occupancy_grid.info.width))
        occupancy_grid.data = np.concatenate((to_add, occupancy_grid.data), axis = 0)
        origin_x -= occupancy_grid.info.resolution * np.shape(to_add)[0]
        occupancy_grid.info.height += np.shape(to_add)[0]
    elif(target_point.x > occupancy_grid.info.origin.position.x + occupancy_grid.info.resolution * occupancy_grid.info.height):
        to_add = np.ones((int((target_point.x - occupancy_grid.info.origin.position.x - occupancy_grid.info.resolution * occupancy_grid.info.height)/occupancy_grid.info.resolution) + 5 , occupancy_grid.info.width))
        occupancy_grid.data = np.concatenate((occupancy_grid.data, to_add), axis = 0)
        occupancy_grid.info.height += np.shape(to_add)[0]

    occupancy_grid.data = np.reshape(occupancy_grid.data, (occupancy_grid.info.height, occupancy_grid.info.width))

    if(target_point.y > occupancy_grid.info.origin.position.y):
        to_add = np.ones((occupancy_grid.info.height, int((target_point.y - occupancy_grid.info.origin.position.y)/occupancy_grid.info.resolution) + 5))
        occupancy_grid.data = np.concatenate((to_add, occupancy_grid.data), axis = 1)
        origin_y += occupancy_grid.info.resolution * np.shape(to_add)[1]
        occupancy_grid.info.width += np.shape(to_add)[1]
    elif(target_point.y < occupancy_grid.info.origin.position.y - occupancy_grid.info.resolution * occupancy_grid.info.width):
        to_add = np.ones((occupancy_grid.info.height, int((occupancy_grid.info.origin.position.y - occupancy_grid.info.resolution * occupancy_grid.info.width - target_point.y)/occupancy_grid.info.resolution) + 5))  
        occupancy_grid.data = np.concatenate((occupancy_grid.data, to_add), axis = 1) 
        print(np.shape(to_add))
        print(str(occupancy_grid.info.origin.position.y - occupancy_grid.info.resolution * occupancy_grid.info.width - target_point.y))
        occupancy_grid.info.width += np.shape(to_add)[1] 
    
    occupancy_grid.data = np.reshape(occupancy_grid.data, (occupancy_grid.info.height, occupancy_grid.info.width))
    
    origin_new = Point()
    origin_new.x = origin_x
    origin_new.y = origin_y
    origin_new.z = occupancy_grid.info.origin.position.z
    occupancy_grid.info.origin.position = origin_new
    print(np.shape(occupancy_grid.data))
    print(occupancy_grid.info.resolution)
    print(occupancy_grid.info.origin.position)
    print(target_point)
    return occupancy_grid

def ErodeMap(occupancy_grid):
    '''
    Runs OpenCV erode (makes obstacles appear larger)
    Done to account for robot size (have to give obstacles a certain berth)
    '''
    to_erode = np.copy(occupancy_grid.data)

    robot_width = 0.34 # 34 cm
    kernel_size = int(robot_width / (occupancy_grid.info.resolution))

    if(kernel_size % 2 == 0):
        kernel_size += 1
    
    kernel = np.ones((kernel_size,kernel_size), np.uint8)
    erodedArray = cv2.erode(to_erode, kernel)

    occupancy_grid.data = erodedArray

    return occupancy_grid

def SquareMap(occupancy_grid):
    if(occupancy_grid.info.width > occupancy_grid.info.height):
        diff = occupancy_grid.info.width - occupancy_grid.info.height
        top_add = np.ones(((diff-int(diff/2)), occupancy_grid.info.width))
        bottom_add = np.ones((int(diff/2), occupancy_grid.info.width))
        occupancy_grid.data = np.concatenate((top_add, occupancy_grid.data), axis = 0)
        occupancy_grid.data = np.concatenate((occupancy_grid.data, bottom_add), axis = 0)
        occupancy_grid.info.height = occupancy_grid.info.width
    elif(occupancy_grid.info.height > occupancy_grid.info.width):
        diff = occupancy_grid.info.height - occupancy_grid.info.width
        left_add = np.ones((occupancy_grid.info.height, (diff-int(diff/2))))
        right_add = np.ones((occupancy_grid.info.height, int(diff/2)))
        occupancy_grid.data = np.concatenate((left_add, occupancy_grid.data), axis = 1)
        occupancy_grid.data = np.concatenate((occupancy_grid.data, right_add), axis = 1)
        occupancy_grid.info.width = occupancy_grid.info.height
    return occupancy_grid