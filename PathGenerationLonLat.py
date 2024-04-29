import numpy as np
import xml.etree.ElementTree as ET

stepsize = 0.0003

def ReadCoordinatesFromXML(xml_file, target_block, target_field):
    tree = ET.parse(xml_file)
    root = tree.getroot()
    coordinates = []
    
    for trk in root.findall('.//trk'):
        block_nr = trk.find('BlockNr').text
        field_nr = trk.find('FieldNr').text
        if block_nr == target_block and field_nr == target_field:
            for trkpt in trk.findall('trkpt'):
                lat = float(trkpt.get('lat'))
                lon = float(trkpt.get('lon'))
                coordinates.append([lat, lon])
            return coordinates

def FromCoordinatesToField(coordinates):
    points = np.array(coordinates, dtype=np.float32)

    # Find the highest and lowest lon and lat values from points
    min_lon = np.min(points[:, 0]) # X-coordinate
    max_lon = np.max(points[:, 0])
    min_lat = np.min(points[:, 1]) # Y-coordinate
    max_lat = np.max(points[:, 1])

    inside_points = []
    for lat in np.arange(min_lat, max_lat, stepsize):
        for lon in np.arange(min_lon, max_lon, stepsize):
            point = (lon, lat)
            inside = False
            for i in range(len(coordinates) - 1):
                p1, p2 = coordinates[i], coordinates[i + 1]
                # Check if the point is on the line segment between p1 and p2
                if ((p1[0] <= lon <= p2[0]) or (p2[0] <= lon <= p1[0])) and ((p1[1] <= lat <= p2[1]) or (p2[1] <= lat <= p1[1])):
                    inside = True
                    break
            if inside:
                inside_points.append(point)

    return inside_points
    


def FromPointcloudToPath(point_cloud):
    #Solving the field with a sweep line algorithm
    path = []
    
    # Implement a quick read of the drones lon location in order to fly to the cloest side of the field.
    """""
    drone_location_lon = 12.3456789
    
    # Figure out if the drone is closer to the top or bottom of the field:
    distance_to_min_y = np.linalg.norm(drone_location_lon - min_y)  # drone_location only needs to be lon
    distance_to_max_y = np.linalg.norm(drone_location_lon - max_y)
    
    if distance_to_min_y > distance_to_max_y:
        # Sort the point cloud by y-coordinate in descending order
        sorted_point_cloud = sorted(point_cloud, key=lambda point: point[1], reverse=True)
    else:
        # Sort the point cloud by y-coordinate in ascending order
        sorted_point_cloud = sorted(point_cloud, key=lambda point: point[1])
    """""
    sorted_point_cloud = sorted(point_cloud, key=lambda point: point[1])
        
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
            
    return path

def FromPathtoXML(path):
    # Create the root element <gpx>
    gpx = ET.Element("gpx", version="1")
    trk = ET.SubElement(gpx, "trk")
    trk.text = "\n"
    
    # Loop through each point in the route
    for point in path:
        trkpt = ET.SubElement(trk, "trkpt")
        trkpt.tail = "\n"
        trkpt.set("lat", str(point[0]))  # Set the latitude attribute
        trkpt.set("lon", str(point[1]))  # Set the longitude attribute
    
    # Create an ElementTree object
    tree = ET.ElementTree(gpx)
    
    # Write the XML tree to a file
    tree.write("pathfordrone.xml")


def main():
    BlockNr = input('What is the block number, of the field?\n')
    FieldNr = input('What is the field number, of the field?\n')
    coordinates = ReadCoordinatesFromXML('DkGpx1.xml', BlockNr, FieldNr)
    points = FromCoordinatesToField(coordinates)
    path = FromPointcloudToPath(points)
    FromPathtoXML(path)
    
    
if __name__ == "__main__":
    main()