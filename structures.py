
class SatStruct(object):

    def __init__(self):
        self.max = None
        self.min = None

class ConsStruct(object):

    def __init__(self):
        self.sat = SatStruct()



#From https://stackoverflow.com/a/64562039
class structure(object):
    def __init__(self,**kwargs):
        self.Set(**kwargs)

    def Set(self,**kwargs):
        self.__dict__.update(kwargs)

    def __setattr__(self, name, value):
        pass
        object.__setattr__(self, name, value)

    def __getattr__(self, name):
        pass
        obj = structure()
        self.__setattr__(name=name, value=obj)
        return obj

    def __contains__(self, name):
       return name in self.__dict__

    def __getitem__(self, name):
        return self.__dict__[name]

if __name__ == '__main__':


    a = structure()
    a.b = 4
    a.c.d = 1
    pass
    pass
    print a.c.d