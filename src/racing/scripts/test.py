class inner:
    def __init__(self):
        print(  "inner init" )

    def __del__(self) :
        print(  "inner del" )
class outer:
    def __init__(self):
        print(  "outer init" )
        self.obj = inner()
    
    def deleteobj(self):
        del self.obj

obj1 = outer()
        