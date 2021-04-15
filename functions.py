def constrain(val, min_val, max_val):
    return min(max_val, max(min_val, val))



"""function for mapping value with limits, equivalent to Arduinos map function"""
def _map(x, in_min, in_max, out_min, out_max):
    x = min(in_max, max(in_min, x))
    d = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    return d

import inspect


line_d = inspect.currentframe
def print_frame(f:line_d, *args):
    info = inspect.getframeinfo(f)
    print("\n------------------------------------------------------------")
    if len(args):
        for arg in args:
            print("  |==>  message: ", arg, "\n  |------------------------------------------------------------")
    print("  |printed at line: %s\n  |in fuction: %s \n  |in document:%s" % (info.lineno, info.function, info.filename))
    print("------------------------------------------------------------\n")
print_line = lambda *args: print_frame(line_d().f_back,*args)