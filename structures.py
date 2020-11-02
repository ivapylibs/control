
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

    def __setitem__(self, key, value):
        self.__dict__[key]=value

# Basic demo of various ways to use the structure class
if __name__ == '__main__':
    a = structure()
    a.b = 1
    a.c.d = 2
    a['temp'] = 3
    a.c['e'] = 4
    a.k.l.m.x = 5

    print a.b
    print a.c.d
    print a.temp
    print a.c['e']
    print a.k.l.m.x
