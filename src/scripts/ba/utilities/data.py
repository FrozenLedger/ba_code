class Data:
    def __init__(self,data):
        self.__data = data

    @property
    def data(self):
        return self.__data
    
    def setData(self,data):
        self.__data = data