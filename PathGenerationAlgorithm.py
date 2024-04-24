import numpy as np
import cv2
import networkx as nx
from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2

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
    # Method 1 - Using the Traveling Salesman Problem Approximation algorithm from NetworkX
    """""
    # Create a graph
    G = nx.Graph()

    # Add nodes to the graph
    for point in point_cloud:
        G.add_node(tuple(point))

    # Add edges to the graph
    
    for i in range(len(point_cloud)):
        for j in range(i+1, len(point_cloud)):
            distance = np.linalg.norm(np.array(point_cloud[i]) - np.array(point_cloud[j]))    
            G.add_edge(tuple(point_cloud[i]), tuple(point_cloud[j]), weight=distance)
            
    # Find the shortest path using the Traveling Salesman Problem algorithm
    path = nx.approximation.traveling_salesman_problem(G)  # If we want to use a different algorithm, we can change this line.

    # Convert the path to a list of points
    path = [list(point) for point in path]

    # Add the drone location to the beginning of the path
    path.insert(0, drone_location)

    # Print the path in (x,y) format
    print("Path:", path)
    
    # Visualize the path on the field - Debugging
    for i in range(len(path)-1):
        cv2.line(hull_field, tuple(path[i]), tuple(path[i+1]), 255, 1)
    cv2.imshow("Field with Path", hull_field), cv2.waitKey(0), cv2.destroyAllWindows()
    return path
    """""
    
    # Method 2 - Using the OR-Tools library to solve the Traveling Salesman Problem
    """""
    point_cloud.append(drone_location)

    # Create a routing index manager
    manager = pywrapcp.RoutingIndexManager(len(point_cloud), 1, point_cloud.index(drone_location))

    # Create a routing model
    routing = pywrapcp.RoutingModel(manager)

    # Define distance function (callback) with penalty for long movements
    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        # Calculate Euclidean distance between nodes
        distance = np.linalg.norm(np.array(point_cloud[from_node]) - np.array(point_cloud[to_node]))
        return distance

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Define cost function
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Set first node (drone location) as the starting point
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)
    search_parameters.time_limit.seconds = 90
    search_parameters.log_search = True

    # Number of solutions to generate
    num_solutions = num_paths_to_generate

    # Collect all solutions
    solutions = []
    for _ in range(num_solutions):
        # Solve TSP for a new solution
        assignment = routing.SolveWithParameters(search_parameters)

        # Extract path
        path = []
        index = routing.Start(0)
        while not routing.IsEnd(index):
            node = manager.IndexToNode(index)
            path.append(point_cloud[node])
            index = assignment.Value(routing.NextVar(index))

        # Add the drone location to the beginning of the path
        path.insert(0, drone_location)
        solutions.append(path)

    # Calculate path lengths for each solution
    path_lengths = [sum(np.linalg.norm(np.array(path[i]) - np.array(path[i + 1])) for i in range(len(path) - 1)) for path in solutions]
    print(path_lengths)

    # Find the index of the shortest path
    shortest_path_index = np.argmin(path_lengths)
    shortest_path = solutions[shortest_path_index]
    print("Shortest Path:", shortest_path)

    # Visualize the shortest path on the field
    for i in range(len(shortest_path) - 1):
        cv2.line(hull_field, tuple(shortest_path[i]), tuple(shortest_path[i + 1]), 255, 1)

    # Show the field with the shortest path
    cv2.imshow("Field with Shortest Path", hull_field)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    return shortest_path
    """""
    
    # Method 3 - Solving the field with a sweep line algorithm
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