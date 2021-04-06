def constrain(val, min_val, max_val):
    return min(max_val, max(min_val, val))



"""function for mapping value with limits, equivalent to Arduinos map function"""
def _map(x, in_min, in_max, out_min, out_max):
    x = min(in_max, max(in_min, x))
    d = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    return d

