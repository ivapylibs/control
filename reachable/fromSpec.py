from .empty import EmptySet
import structures

class FromSpec(EmptySet):
    def __init__(self, specs):
        super().__init__()
        self.specs = specs

    def setSpec(self, specs):
        self.specs = specs

