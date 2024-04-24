import numpy as np
import cv2

size = 800
qudrant_size= size//4
stepsize = 10

def MakeAMagicField():
    arr_to_fill = np.zeros((size,size))
    
    x = [np.random.randint(0,qudrant_size*2), np.random.randint(qudrant_size*0,qudrant_size*2), 
         np.random.randint(qudrant_size*2,qudrant_size*4), np.random.randint(qudrant_size*2,qudrant_size*4)]
    y = [np.random.randint(0,qudrant_size*2), np.random.randint(qudrant_size*2,qudrant_size*4), 
         np.random.randint(qudrant_size,qudrant_size*2), np.random.randint(qudrant_size*2,qudrant_size*4)]
    
    cv2.circle(arr_to_fill, (x[0],y[0]), 3, 255, 1), cv2.circle(arr_to_fill, (x[1],y[1]), 3, 255, 1),
    cv2.circle(arr_to_fill, (x[2],y[2]), 3, 255, 1), cv2.circle(arr_to_fill, (x[3],y[3]), 3, 255, 1)
    
    cv2.line(arr_to_fill, (x[0],y[0]), (x[1],y[1]), 255, 1), cv2.line(arr_to_fill, (x[1],y[1]), (x[3],y[3]), 255, 1),
    cv2.line(arr_to_fill, (x[3],y[3]), (x[2],y[2]), 255, 1), cv2.line(arr_to_fill, (x[2],y[2]), (x[0],y[0]), 255, 1)
    
    return arr_to_fill


def FromFieldToPointcloud(field):
    borderx = []
    bordery = []
    point_cloud = []
    
    # Split the field into chunks of size stepsize x stepsize.
    for i in range(int(size/stepsize)):
        for j in range(int(size/stepsize)):
            chunk = field[i*stepsize:(i+1)*stepsize, j*stepsize:(j+1)*stepsize]
            chunk = chunk.astype(np.uint8) # Convert the chunk to a numpy array of type uint8, to avoid truncation errors.

            # Check if the chunk contains a border, if it does, add the border to the list.
            if 255 in chunk:
                borderx.append(j*stepsize)
                borderx.append((j+1)*stepsize)
                
                bordery.append(i*stepsize)
                bordery.append((i+1)*stepsize)
    
    points = np.column_stack((borderx, bordery))
    hull = cv2.convexHull(points)
    hull_field = np.zeros_like(field)
    cv2.polylines(hull_field, [hull], True, 255, 1)
    
    # Check if a point is inside the hull, and if it is, add it to the list.
    for i in range(len(bordery)):
        for j in range(len(borderx)):
            if cv2.pointPolygonTest(hull, (borderx[i], bordery[j]), False) > 0:
                if [borderx[i], bordery[j]] not in point_cloud: # Check if (x,y) is already in the list.
                    hull_field[bordery[j], borderx[i]] = 255
                    point_cloud.append([borderx[i], bordery[j]]) # Add (x,y) to the list.
    
    # Draw the drone on the field.
    DroneLocation = [np.random.randint(0,qudrant_size*4), np.random.randint(0,qudrant_size*4)] # Random drone location - Change to GPS
    hull_field[DroneLocation[1], DroneLocation[0]] = 255
    cv2.circle(hull_field, (DroneLocation[0], DroneLocation[1]), 5, 255, 1)
    cv2.imshow("Field with Drone", hull_field), cv2.waitKey(0), cv2.destroyAllWindows()
    
    return point_cloud, hull_field, DroneLocation
        

def FromPointcloudToPath(point_cloud, hull_field, drone_location):
    #Solving the field with a sweep line algorithm
    path = []
    
    # Find the top and bottom of our field
    min_y, max_y = min(point_cloud, key=lambda x: x[1]), max(point_cloud, key=lambda x: x[1])

    # Calculate the distance from the drone to min_y and max_y
    distance_to_min_y = np.linalg.norm(np.array(drone_location) - np.array(min_y))
    distance_to_max_y = np.linalg.norm(np.array(drone_location) - np.array(max_y))
    
    if distance_to_min_y > distance_to_max_y:
        # Sort the point cloud by y-coordinate in descending order
        sorted_point_cloud = sorted(point_cloud, key=lambda point: point[1], reverse=True)
    else:
        # Sort the point cloud by y-coordinate in ascending order
        sorted_point_cloud = sorted(point_cloud, key=lambda point: point[1])

    # Find the unique rows in the sorted point cloud
    unique_rows = []
    current_row = [sorted_point_cloud[0]]
    for i in range(1, len(sorted_point_cloud)):
        if sorted_point_cloud[i][1] == current_row[0][1]:
            current_row.append(sorted_point_cloud[i])
        else:
            unique_rows.append(current_row)
            current_row = [sorted_point_cloud[i]]
    unique_rows.append(current_row)

    # Print the coordinates in each unique row
    for i, row in enumerate(unique_rows):
        sorted_ass = sorted(row, key=lambda point: point[0]) # Low --> High
        sorted_des = sorted(row, key=lambda point: point[0], reverse=True) # High --> Low
        
        # We need to move the oppisite direction every other itteration.
        if i%2:
            path.extend(sorted_ass)
        else:
            path.extend(sorted_des)
    
    # Visualize the path on the field
    for i in range(len(path) - 1):
        cv2.line(hull_field, tuple(path[i]), tuple(path[i + 1]), 255, 1)

    # Show the field with the path
    cv2.imshow("Field with Path", hull_field)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    
    return path
    

def main():
    field = MakeAMagicField() # Create a random field - This will be replaced by the actual field and gps coordinates.
    point_cloud, hull_field, drone_location = FromFieldToPointcloud(field) # Create a point cloud from the field, with a point for each place to photograph.
    path = FromPointcloudToPath(point_cloud, hull_field, drone_location) # Create a path from the point cloud. - Using the ... algorithm.
    return path
    
    
if __name__ == "__main__":
    main()                    