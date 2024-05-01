import time


"""
    Generate time string
"""
def stamp_str():
    return time.strftime('%Y-%m-%d_%H:%M:%S')


"""
    a.get('b', {}).get('c', {}) ... .get('d', default)
"""
def fetch(d: dict, keys: list, default = None):
    obj = d
    for key in keys:
        if key in obj:
            obj = obj[key]
        else:
            return default
    return obj

