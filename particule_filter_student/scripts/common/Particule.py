# from collections import namedtuple
# MyStruct = namedtuple("MyStruct", "field1 field2 field3")

class Particule:
    x = 0
    y = 0
    w = 0
    proba=0

    def __init__(self,x_0,y_0,w_0,proba_0):
      self.x=x_0
      self.y = y_0
      self.w = w_0
      self.proba = proba_0

    def id(self):
      return str(self.x)+'_'+str(self.y)