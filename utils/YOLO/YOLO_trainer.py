from ultralytics import YOLO
import yaml

#Default parameters from ultralytics
default_params = {"epochs": 100,
                  "batch": 16,
                  "optimizer": 'auto',
                  "freeze": None,
                  "lr0": 0.01,
                  "lrf": 0.01,
                  "momentum": 0.937,
                  "weight_decay": 0.0005,
                  "hsv_h": 0.015,
                  "hsv_s": 0.7,
                  "hsv_v": 0.4,
                  "degrees": 0.0,
                  "translate": 0.1,
                  "scale": 0.5,
                  "flipud": 0.0,
                  "fliplr": 0.5,
                  }

#Opens file with different hyperparameter iterations
with open("utils\YOLO\HyperparamSettings\optimizer.json", "r") as f:
    data = yaml.load(f, Loader=yaml.FullLoader)


#Very important that the code is inside __main__
#Causes errors otherwise (something to do with enabling gpu)
if __name__ == '__main__':
    for config in data["trainings"]:
        params = default_params.copy()

        for param in config:
            params[param] = config[param]


        model = YOLO("utils\YOLO\yolov8n.pt")

        results = model.train(data="utils\YOLO\config.yaml", 
                              epochs = params["epochs"],
                              batch = params["batch"],
                              optimizer = params["optimizer"],
                              freeze = params["freeze"],
                              lr0 = params["lr0"],
                              lrf = params["lrf"],
                              momentum = params["momentum"],
                              weight_decay = params["weight_decay"],
                              hsv_h = params["hsv_h"],
                              hsv_s = params["hsv_s"],
                              hsv_v = params["hsv_v"],
                              degrees = params["degrees"],
                              translate = params["translate"],
                              scale = params["scale"],
                              flipud = params["flipud"],
                              fliplr = params["fliplr"]
                              )