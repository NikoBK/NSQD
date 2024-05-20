import numpy as np
import xml.etree.ElementTree as ET
import cv2

stepsize = 0.00022711 # 13.746 meters in latitude and longitude


def read_coordinates_from_xml(xml_file, target_block, target_field):
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


def from_coordinates_to_field(coordinates):
    points = np.array(coordinates, dtype=np.float32)
    hull = cv2.convexHull(points, clockwise=False)  # Compute the convex hull

    inside_points = []

    min_lon = np.min(points[:, 0])
    max_lon = np.max(points[:, 0])
    min_lat = np.min(points[:, 1])
    max_lat = np.max(points[:, 1])

    # Check if each point is inside the convex hull
    for lat in np.arange(min_lat, max_lat, stepsize):
        for lon in np.arange(min_lon, max_lon, stepsize):
            if cv2.pointPolygonTest(hull, (lon, lat), measureDist=False) >= 0:
                inside_points.append((lon, lat))

    return inside_points


def from_pointcloud_to_path(point_cloud):
    # Solving the field with a sweep line algorithm
    path = []
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
        sorted_ass = sorted(row, key=lambda point: point[0])  # Low --> High
        sorted_des = sorted(row, key=lambda point: point[0], reverse=True)  # High --> Low

        # We need to move the oppisite direction every other itteration.
        if i % 2:
            path.extend(sorted_ass)
        else:
            path.extend(sorted_des)

    return path


def from_path_to_xml(path):
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
    blocknr = input('What is the block number, of the field?\n')
    fieldnr = input('What is the field number, of the field?\n')
    coordinates = read_coordinates_from_xml('DkGpx1.xml', blocknr, fieldnr)
    points = from_coordinates_to_field(coordinates)
    path = from_pointcloud_to_path(points)
    from_path_to_xml(path)


if __name__ == "__main__":
    main()
