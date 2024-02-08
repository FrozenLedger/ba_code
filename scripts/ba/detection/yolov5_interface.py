#!/usr/bin/env python3
import torch, yolov5

class DetectionAdapter:
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
    model = torch.hub.load("ultralytics/yolov5",weights)
    return ObjectDetectionModel(model)

def Trashnet():
    model = yolov5.load("turhancan97/yolov5-detect-trash-classification")
    return ObjectDetectionModel(model)

class ObjectDetectionModel:
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

    imgID = 1702375746
    impath = f"/home/workspace1/rsd435_images/Images/snapshot_{imgID}/rgb_img_{imgID}.jpg"
    cls_id: int = 39 # 39 := bottle
    print(obj_model.detect(impath,clsIDs=[cls_id]))
    print(trash_model.detect(impath,clsIDs=[cls_id]))