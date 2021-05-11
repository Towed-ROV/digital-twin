from agx import Vec3
# ------------------------POSITION, SIZE AND WIRE------------------------#
WATER_DEPTH = 60
WATER_WIDTH = 30
WATER_LENGTH = 1000
WATER_DENSITY = 1027
WIRE_LENGTH = 200
WIRE_RESOLUTION = 3
WIRE_POS_ROV = (0, 0, 0.22)
# ------------------------ROV------------------------#
ROV_POS = Vec3(-WATER_LENGTH / 2 + 20, 0, 0)
ROV_K_P = 2.5
ROV_K_I = 100
ROV_K_D = 5
ROV_DEPTH_SETPOINT = -50
ROV_BODY_DENSITY = 1304.36
ROV_WING_DENSITY = 1027
ROV_TANK_DENSITY = 150
WING_SCALE = 1*1.25*2
ROV_SCALE = 0.001
MAX_WING_ANGLE = 35
MIN_WING_ANGLE = -35
K_P_TRIM = 0.02
K_I_TRIM = 0.02
K_D_TRIM = 1
TRIM_MAX_OUT = 8
TRIM_MIN_OUT = -8
CM_ROV = 20,0,0
# ------------------------SIMULATION------------------------#
SIM_TIME_STEP = 0.001
start = False
plot = False
adjust_rov = False
# ------------------------BOAT------------------------#
BOAT_K_P = 0.02
BOAT_K_I = 0.0001
BOAT_K_D = 0
BOAT_POS = Vec3(-WATER_LENGTH / 2 + 20 + WIRE_LENGTH, 0, 0)
BOAT_SPEED = 3.5
BOAT_MAX_OUT = 8
BOAT_MIN_OUT = -8
# ------------------------HYDORDYNAMICS------------------------#
STD_PDRAG = 0.6
STD_VDRAG = 0.1
STD_LIFT = 0.01
WING_PDRAG = STD_PDRAG*1
WING_LIFT = STD_LIFT*2
WING_VDRAG = STD_VDRAG*1
BODY_PDRAG = STD_PDRAG*1
BODY_LIFT = STD_LIFT*1
BODY_VDRAG = STD_VDRAG*1
# ------------------------MODELS------------------------#
WING_NAME = "wing_simp.obj"
MODEL_LOCATION = "models/"
ROV_MODEL_NAME = "ROV_SIMP_w_tail_nf.obj"