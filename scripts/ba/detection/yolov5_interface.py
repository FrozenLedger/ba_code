#!/usr/bin/env python3
import torch, yolov5

from pathlib import Path

if torch.cuda.is_available():
    print(f"GPU: {torch.cuda.get_device-name(0)} is available.")
else:
    print("No GPU available.")

class DetectionAdapter:
    """An adapter class for easier excess of different data structures from the yolo inference data."""
    def __init__(self,detection):
        self.__detection = detection

    @property
    def detection(self):
        return self.__detection

    @property
    def dataframe(self):
        try:
            return self.__dataframe
        except:
            self.__dataframe = self.__detection.pandas().xyxy[0]
            return self.__dataframe

    @property
    def nparray(self):
        try:
            return self.__nparray
        except:
            self.__nparray = self.__detection.pred[0].numpy()#dataframe.to_numpy(copy=True)
            return self.__nparray
        
    @property
    def bboxes(self):
        return self.nparray[:,:4]
    
    @property
    def confidences(self):
        return self.nparray[:,4]
    
    @property
    def classes(self):
        return self.nparray[:,5]
        
    #def __pred_write(self,imgID:int, data:pandas.DataFrame, clsID: int = None):
    #    predpath = pathlib.get_pred_path(imgID)

def Yolov5Model(weights="yolov5m"):
    """Reads the local model and weights of the yolov5 CNN by Ultralytics if possible. If there are no local weights, the the methode will try to download and save them locally."""
    fname = f"{weights}.pt"
    try:
        model = load(fname)
    except FileNotFoundError as e:
        print(e)
        print("Reload model from internet...")
        model = torch.hub.load("ultralytics/yolov5",weights)
        print("Done.")
        print("Save model...")
        save(model,fname)
        print("Done.")
    return ObjectDetectionModel(model)

def Trashnet():
    """Reads the local model and weights of the yolov5-detect-trash-classification by turnhancan' if possible. If there are no local weights, then the methode will try to download and save them locally."""
    # based on: https://stackoverflow.com/questions/67302634/how-do-i-load-a-local-model-with-torch-hub-load
    fname = "trashnet.pt"
    try:
        model = load(fname)
    except FileNotFoundError as e:
        print(e)
        print("Reload model from internet...")
        model = yolov5.load("turhancan97/yolov5-detect-trash-classification")
        print("Done.")
        print("Save model...")
        save(model,fname)
        print("Done.")
    return ObjectDetectionModel(model)

def __path():
    """Defines the path to the location where the CNN model and its weights will be stored and read from."""
    return f"{Path.home()}/CNN"

def save(model,fname):
    """Saves the model an its weights locally."""
    path = __path()
    Path(path).mkdir(parents=True,exist_ok=True)
    torch.save(model,f"{path}/{fname}")

def load(fname):
    """Reads the model given by name and returns it as pytorch-model."""
    path = __path()
    return torch.load(f"{path}/{fname}")    

class ObjectDetectionModel:
    """Base class for object detection."""
    def __init__(self,model):
        self.__model = model

    @property
    def model(self):
        return self.__model

    def __reset_clsIDs(self):
        try:
            del self.__model.classes
        except:
            pass

    def detect(self, impath, clsIDs=[]):
        """Returns the result of the inference by the model used. For the yolov5 model, the returned class will be Detection.
For more information about the Detection class, follow this: https://ml-research.github.io/alphailpdoc/src.yolov5.models.html"""
        if len(clsIDs) > 0:
            self.__model.classes = clsIDs
        else:
            self.__reset_clsIDs()
        
        return self.__model(impath) #DetectionAdapter(self.__model(impath))
 
if __name__ == "__main__":
    #model = Yolov5Model()
    obj_model = Yolov5Model() #yolov5.load("turhancan97/yolov5-detect-trash-classification")
    trash_model = Trashnet()

    imgID = 1707574283630295991
    impath = f"/tmp/rsd435_images/color_{imgID}.jpg"
    cls_id: int = 39 # 39 := bottle
    print(obj_model.detect(impath,clsIDs=[cls_id]))
    print(trash_model.detect(impath,clsIDs=[cls_id]))