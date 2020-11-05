import time


class Timer(object):
    def __init__(self, name=None):
        self.name = name

    def __enter__(self):
        self.tstart = time.time()

    def __exit__(self, type, value, traceback):
        if self.name:
            print('%s Elapsed Time: %s seconds' % (self.name, (time.time() - self.tstart)))
        else:
            print('Elapsed Time: %s seconds' % (time.time() - self.tstart))