import inspect
import math

line_d = inspect.currentframe
PI =  math.pi
def print_frame(f:line_d, *args):
    info = inspect.getframeinfo(f)
    print("\n------------------------------------------------------------")
    if len(args):
        for arg in args:
            print("  |==>  message: ", arg, "\n  |------------------------------------------------------------")
    print("  |printed at line: %s\n  |in fuction: %s \n  |in document:%s" % (info.lineno, info.function, info.filename))
    print("------------------------------------------------------------\n")

print_line = lambda *args: print_frame(line_d().f_back,*args)