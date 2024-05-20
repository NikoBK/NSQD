import os
import yaml

def convert_supervisely_to_yolo(supervisely_path, yolo_path, class_names, num_of_files = -1, remove_frame=True):
    if num_of_files == -1:
        num_of_files = len(os.listdir(supervisely_path))

    for filename in os.listdir(supervisely_path)[:num_of_files]:
        if filename.endswith(".json"):
            with open(os.path.join(supervisely_path, filename), 'r') as f:
                data = yaml.load(f, Loader=yaml.FullLoader)
                
                with open(os.path.join(yolo_path, os.path.splitext(os.path.splitext(filename)[0])[0] + ".txt"), 'w') as out_file:
                    img_height = data['size']['height']
                    img_width = data['size']['width']

                    if remove_frame:
                        img_height -= 280 #2*border thickness
                        img_width -= 280

                    for obj in data['objects']:
                        x1, y1, x2, y2 = obj['points']['exterior'][0] + obj['points']['exterior'][1]

                        if remove_frame:
                            x1 -= 140 #140 = border thickness 
                            x2 -= 140
                            y1 -= 140
                            y2 -= 140

                        width_norm = (x2 - x1) / img_width
                        height_norm = (y2 - y1) / img_height
                        x_center = x1 + ((x2 - x1) / 2)
                        y_center = y1 + ((y2 - y1) / 2)
                        x_center /= img_width
                        y_center /= img_height
                        classID = class_names.index(obj['classTitle'])
                        out_file.write(f"{classID} {x_center} {y_center} {width_norm} {height_norm}\n")

path = "dataset 2024-05-12 11_15_51\\ann"
output_path = "data\YOLOLabels"

class_names = ["Stone"]

# Example usage:
convert_supervisely_to_yolo(path, output_path, class_names, remove_frame=False)

