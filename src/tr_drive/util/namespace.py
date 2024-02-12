class DictRegulator:
    def __init__(self, d):
        for key, value in d.items():
            if isinstance(value, dict):
                setattr(self, key, DictRegulator(value))
            else:
                setattr(self, key, value)
    
    def add(self, key, value):
        setattr(self, key, value)
    
    def remove(self, key):
        delattr(self, key)
    
    def to_dict(self):
        res = {}
        for key, value in self.__dict__.items():
            if isinstance(value, DictRegulator):
                res[key] = value.to_dict()
            else:
                res[key] = value
        return res

