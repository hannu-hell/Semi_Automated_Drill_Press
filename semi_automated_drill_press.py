import time
from rotary_irq_rp2 import RotaryIRQ
import machine
from st7735 import TFT, TFTColor
from st7735 import sysfont
from machine import SPI,Pin
from stepper import Stepper
import time
import math
from stepper import Stepper


#------ Hardware setup ------#

spi = SPI(1, baudrate=20000000, polarity=0, phase=0,
              sck=Pin(10), mosi=Pin(11), miso=None)
tft=TFT(spi,16,17,13)
tft.initr()
tft.rgb(True)

s1 = Stepper(1, 2, 0, steps_per_rev=200, speed_sps=300, invert_dir=True)
s2 = Stepper(4, 5, 3, steps_per_rev=6400,speed_sps=100, invert_dir=True)

end_stop = machine.Pin(19, machine.Pin.IN, machine.Pin.PULL_UP)
rot_button = machine.Pin(8, machine.Pin.IN)
y_button = machine.Pin(7, machine.Pin.IN, machine.Pin.PULL_UP)
g_button = machine.Pin(27, machine.Pin.IN, machine.Pin.PULL_UP)
r_button = machine.Pin(28, machine.Pin.IN, machine.Pin.PULL_UP)
b_button = machine.Pin(6, machine.Pin.IN, machine.Pin.PULL_UP)
drill = machine.Pin(21, machine.Pin.OUT)
drill.value(1)

r = RotaryIRQ(pin_num_clk=12, 
              pin_num_dt=18, 
              min_val=-85, 
              max_val=85, 
              reverse=True, 
              range_mode=RotaryIRQ.RANGE_WRAP)

#------ Define variables ------#

y_but_active = False
b_but_active = False
g_but_active = False
r_but_active = False
rot_but_active = False
drill_active = False
drill_points = [(30.0,40.0),(-40.0,20.0),(-20.0,-20.0)]
drill_index = 0
drill_point = None
select_x_step = False
select_y_step = False
select_x_pos_int = False
select_y_pos_int = False
select_x_pos_dec = False
select_y_pos_dec = False
x_reverse = False
y_reverse = False

main_screen = True
step_screen = False
control_mode_screen = False
manual_mode_screen = False
move_x_screen = False
move_y_screen = False
auto_mode_screen = False
add_coordinates_screen = False
auto_process_screen = False

x_step = 1
y_step = 1
x_pos_int = 0
y_pos_int = 0
x_pos_dec = 0
y_pos_dec = 0
x_pos = 0.0
y_pos = 0.0


#------ Motion Calculation Functions ------#

# X-Axis
def x_axis_steps(displacement):
    if displacement < 0:
        displacement = displacement*-1
    steps = 200 * displacement
    return steps

def x_axis_steps_to_coordinates(steps):
    coords = steps / 200
    return round(coords,1)
    

# Y-Polar
def polar_steps(angle):
    if angle < 0:
        angle = angle*-1
    steps = (6400 * angle) / 360
    return round(steps)

def is_valid_coordinates(coordinates):
    polar_condition = math.sqrt(math.pow(85,2) - math.pow(coordinates[0],2))
    if coordinates[0] >= -85 and coordinates[0] <= 85:
        if coordinates[1] <= polar_condition and coordinates[1] >= -polar_condition:
            return True
    return False


def calculate_path_trajectory(P1, P2):
    # Calculate P1 handle from Origin
    P1_handle = math.sqrt(math.pow(P1[0],2) + math.pow(P1[1],2))

    # Calculate P2 handle from Origin
    P2_handle = math.sqrt(math.pow(P2[0],2) + math.pow(P2[1],2))

    # Calculate the cartesian displacement between P1 and P2
    x_displacement = P2[0] - P1[0]
    y_displacement = P2[1] - P1[1]

    # Calculate displacement vector between P1 and P2
    displacement_vector = math.sqrt(math.pow(x_displacement,2) + math.pow(y_displacement, 2))

    # Calculate the angle(beta) between P1 and P2 points from Origin
    beta = math.acos( ((math.pow(P1_handle,2)) + (math.pow(P2_handle,2)) - (math.pow(displacement_vector,2))) / (2*P1_handle*P2_handle) )

    # Adjust P1_handle by moving x-axis to match P2_handle (positive indicates move left)
    x_axis_handle_adjust = P2_handle - P1_handle

    # 1 indicates clockwise and -1 indicates anticlockwise
    rotation_direction = 1

    # Check rotation conditions

    # Calculate gradient 'm' for P1_handle
    m = P1[1] / P1[0]

    # Check P1 quadrant from coordinates and rotation connditions
    if P1[0] > 0 and P1[1] > 0:
        if P2[0] < 0 and P2[1] > 0:
            rotation_direction = 1
        elif P2[0] > 0  and P2[1] < 0:
            rotation_direction = -1
        elif P2[0] > 0 and P2[1] > 0:
            if P2[1] < (m*P2[0]):
                rotation_direction = -1
            elif P2[1] >= (m*P2[0]):
                rotation_direction = 1
        elif P2[0] < 0 and P2[1] < 0:
            if P2[1] < (m*P2[0]):
                rotation_direction = -1
            elif P2[1] >= (m*P2[0]):
                rotation_direction = 1

    if P1[0] < 0 and P1[1] > 0:
        if P2[0] > 0 and P2[1] > 0:
            rotation_direction = -1
        elif P2[0] < 0  and P2[1] < 0:
            rotation_direction = 1
        elif P2[0] < 0 and P2[1] > 0:
            if P2[1] < (m*P2[0]):
                rotation_direction = 1
            elif P2[1] >= (m*P2[0]):
                rotation_direction = -1
        elif P2[0] > 0 and P2[1] < 0:
            if P2[1] < (m*P2[0]):
                rotation_direction = 1
            elif P2[1] >= (m*P2[0]):
                rotation_direction = -1

    if P1[0] < 0 and P1[1] < 0:
        if P2[0] < 0 and P2[1] > 0:
            rotation_direction = -1
        elif P2[0] > 0  and P2[1] < 0:
            rotation_direction = 1
        elif P2[0] < 0 and P2[1] < 0:
            if P2[1] < (m*P2[0]):
                rotation_direction = 1
            elif P2[1] >= (m*P2[0]):
                rotation_direction = -1
        elif P2[0] > 0 and P2[1] > 0:
            if P2[1] < (m*P2[0]):
                rotation_direction = 1
            elif P2[1] >= (m*P2[0]):
                rotation_direction = -1
                
    if P1[0] > 0 and P1[1] < 0:
        if P2[0] > 0 and P2[1] > 0:
            rotation_direction = 1
        elif P2[0] < 0  and P2[1] < 0:
            rotation_direction = -1
        elif P2[0] > 0 and P2[1] < 0:
            if P2[1] < (m*P2[0]):
                rotation_direction = -1
            elif P2[1] >= (m*P2[0]):
                rotation_direction = 1
        elif P2[0] < 0 and P2[1] > 0:
            if P2[1] < (m*P2[0]):
                rotation_direction = -1
            elif P2[1] >= (m*P2[0]):
                rotation_direction = 1
    return (math.degrees(beta), x_axis_handle_adjust, rotation_direction)

#------ UI Functions ------#

def warning_sign(pos, msg):
    tft.fill(TFT.WHITE)
    tft.fillrect((0, pos[1]-10), (128, 40), TFT.BLACK)
    tft.line((pos[0], pos[1]), (pos[0]-10, pos[1]+20), TFT.YELLOW)
    tft.line((pos[0]-10, pos[1]+20), (pos[0]+10, pos[1]+20), TFT.YELLOW)
    tft.line((pos[0]+10, pos[1]+20), (pos[0], pos[1]), TFT.YELLOW)
    tft.line((pos[0], pos[1]+5),(pos[0], pos[1]+10), TFT.YELLOW)
    tft.fillcircle((pos[0],pos[1]+15), 3, TFT.YELLOW)
    tft.text((pos[0]+15, pos[1]+5), msg, TFT.YELLOW, sysfont.sysfont, 1, nowrap=True)
    
def get_final_pos(val_int, val_dec):
    final_x = str(val_int)+str('.')+str(val_dec)
    final_x = float(final_x)
    return final_x

def break_up_to_int_dec(x_coordinate, y_coordinate):
    x_coord = str(x_coordinate).split('.')
    x_int_val = x_coord[0]
    x_dec_val = x_coord[1]
    y_coord = str(y_coordinate).split('.')
    y_int_val = y_coord[0]
    y_dec_val = y_coord[1]
    return x_int_val, x_dec_val, y_int_val, y_dec_val
    
def draw_step_rects():
    tft.rect((58,62),(28,12),TFT.YELLOW)
    tft.rect((58,76),(28,12),TFT.YELLOW)

def update_x_step(val):
    tft.fillrect((59,63),(26,10),TFT.BLACK) 
    tft.text((60, 64), str(val), TFT.WHITE, sysfont.sysfont, 1, nowrap=True)

def update_y_step(val):
    tft.fillrect((59,77),(26,10),TFT.BLACK)
    tft.text((60, 78), str(val), TFT.WHITE, sysfont.sysfont, 1, nowrap=True)

def draw_figure_rects():
    tft.rect((23, 62),(28,12),TFT.YELLOW)
    tft.rect((63, 62),(28,12),TFT.YELLOW)
    tft.rect((23, 76),(28,12),TFT.YELLOW)
    tft.rect((63, 76),(28,12),TFT.YELLOW)
    
def update_x_int(val):
    tft.fillrect((24,63),(26,10),TFT.BLACK) 
    tft.text((25, 64), str(val), TFT.WHITE, sysfont.sysfont, 1, nowrap=True)

def update_y_int(val):
    tft.fillrect((24,77),(26,10),TFT.BLACK)
    tft.text((25, 78), str(val), TFT.WHITE, sysfont.sysfont, 1, nowrap=True)

def update_x_dec(val):
    tft.fillrect((64,63),(26,10),TFT.BLACK)
    tft.text((65, 64), str(val), TFT.WHITE, sysfont.sysfont, 1, nowrap=True)

def update_y_dec(val):
    tft.fillrect((64,77),(26,10),TFT.BLACK)
    tft.text((65, 78), str(val), TFT.WHITE, sysfont.sysfont, 1, nowrap=True)
    
def show_menu():
    tft.fillrect((2,2),(124,90), TFT.BLACK)
    tft.fillrect((6,6),(10,10),TFT.YELLOW)
    tft.fillrect((6,20),(10,10),TFT.GREEN)
    tft.fillrect((6,34),(10,10),TFT.BLUE)
    tft.fillrect((6,48),(10,10),TFT.RED)
    tft.rect((6,60),(118, 30), TFT.YELLOW)

    if main_screen:
        tft.text((20, 7), "CLOCKWISE", TFT.WHITE, sysfont.sysfont, 1, nowrap=True)
        tft.text((20, 21), "ANTI-CLOCKWISE", TFT.WHITE, sysfont.sysfont, 1, nowrap=True)
        tft.text((20, 35), "HOME X-AXIS", TFT.WHITE, sysfont.sysfont, 1, nowrap=True)
        tft.text((20, 49), "CONFIRM", TFT.WHITE, sysfont.sysfont, 1, nowrap=True)
        tft.text((10, 64), "-> ALIGN PLATFORM", TFT.WHITE, sysfont.sysfont, 1, nowrap=True)
        tft.text((10, 78), "-> CLAMP WORKPIECE", TFT.WHITE, sysfont.sysfont, 1, nowrap=True)
        
    elif step_screen:
        tft.text((20, 7), "X-AXIS STEP", TFT.WHITE, sysfont.sysfont, 1, nowrap=True)
        tft.text((20, 21), "POLAR/Y-STEP", TFT.WHITE, sysfont.sysfont, 1, nowrap=True)
        tft.text((20, 35), "MAIN MENU", TFT.WHITE, sysfont.sysfont, 1, nowrap=True)
        tft.text((20, 49), "CONFIRM", TFT.WHITE, sysfont.sysfont, 1, nowrap=True)
        tft.text((10, 64), "X-STEP: ", TFT.WHITE, sysfont.sysfont, 1, nowrap=True)
        update_x_step(x_step)
        tft.text((10, 78), "Y-STEP: ", TFT.WHITE, sysfont.sysfont, 1, nowrap=True)
        update_y_step(y_step)
        
    elif control_mode_screen:
        tft.text((20, 7), "MANUAL CONTROL", TFT.WHITE, sysfont.sysfont, 1, nowrap=True)
        tft.text((20, 21), "AUTO CONTROL", TFT.WHITE, sysfont.sysfont, 1, nowrap=True)
        tft.text((20, 35), "MAIN MENU", TFT.WHITE, sysfont.sysfont, 1, nowrap=True)
        tft.text((20, 49), "STEP SELECT", TFT.WHITE, sysfont.sysfont, 1, nowrap=True)
        draw_step_rects()
        tft.text((10, 64), "X-Step: ", TFT.WHITE, sysfont.sysfont, 1, nowrap=True)
        update_x_step(x_step)
        tft.text((10, 78), "Y-Step: ", TFT.WHITE, sysfont.sysfont, 1, nowrap=True)
        update_y_step(y_step)
        
    elif manual_mode_screen:
        tft.text((20, 7), "MOVE X", TFT.WHITE, sysfont.sysfont, 1, nowrap=True)
        tft.text((20, 21), "MOVE POLAR/Y", TFT.WHITE, sysfont.sysfont, 1, nowrap=True)
        tft.text((20, 35), "MAIN MENU", TFT.WHITE, sysfont.sysfont, 1, nowrap=True)
        tft.text((20, 49), "DRILL", TFT.WHITE, sysfont.sysfont, 1, nowrap=True)
        draw_step_rects()
        tft.text((10, 64), "X-Step: ", TFT.WHITE, sysfont.sysfont, 1, nowrap=True)
        update_x_step(x_step)
        tft.text((10, 78), "Y-Step: ", TFT.WHITE, sysfont.sysfont, 1, nowrap=True)
        update_y_step(y_step)
        
    elif move_x_screen:
        tft.text((20, 7), "X +", TFT.WHITE, sysfont.sysfont, 1, nowrap=True)
        tft.text((20, 21), "X -", TFT.WHITE, sysfont.sysfont, 1, nowrap=True)
        tft.text((20, 35), "BACK", TFT.WHITE, sysfont.sysfont, 1, nowrap=True)
        tft.text((20, 49), "STEP SELECT", TFT.WHITE, sysfont.sysfont, 1, nowrap=True)
        tft.rect((23, 62),(28,12),TFT.YELLOW)
        tft.rect((63, 62),(28,12),TFT.YELLOW)
        tft.rect((58,76),(28,12),TFT.YELLOW)
        tft.text((10, 64), "X:", TFT.WHITE, sysfont.sysfont, 1, nowrap=True)
        tft.text((25, 64), str(x_pos_int), TFT.WHITE, sysfont.sysfont, 1, nowrap=True)
        tft.text((45, 64), " . ", TFT.WHITE, sysfont.sysfont, 1, nowrap=True)
        tft.text((65, 64), str(x_pos_dec), TFT.WHITE, sysfont.sysfont, 1, nowrap=True)
        tft.text((10, 78), "X-Step: ", TFT.WHITE, sysfont.sysfont, 1, nowrap=True)
        tft.fillrect((59,77),(26,10),TFT.BLACK)
        tft.text((60, 78), str(x_step), TFT.WHITE, sysfont.sysfont, 1, nowrap=True)
    
    elif move_y_screen:
        tft.text((20, 7), "Y +", TFT.WHITE, sysfont.sysfont, 1, nowrap=True)
        tft.text((20, 21), "Y -", TFT.WHITE, sysfont.sysfont, 1, nowrap=True)
        tft.text((20, 35), "BACK", TFT.WHITE, sysfont.sysfont, 1, nowrap=True)
        tft.text((20, 49), "STEP SELECT", TFT.WHITE, sysfont.sysfont, 1, nowrap=True)
        tft.rect((23, 62),(28,12),TFT.YELLOW)
        tft.rect((63, 62),(28,12),TFT.YELLOW)
        tft.rect((58,76),(28,12),TFT.YELLOW)
        tft.text((10, 64), "Y:", TFT.WHITE, sysfont.sysfont, 1, nowrap=True)
        tft.text((25, 64), str(x_pos_int), TFT.WHITE, sysfont.sysfont, 1, nowrap=True)
        tft.text((45, 64), " . ", TFT.WHITE, sysfont.sysfont, 1, nowrap=True)
        tft.text((65, 64), str(x_pos_dec), TFT.WHITE, sysfont.sysfont, 1, nowrap=True)
        tft.text((10, 78), "Y-Step: ", TFT.WHITE, sysfont.sysfont, 1, nowrap=True)
        tft.fillrect((59,77),(26,10),TFT.BLACK)
        tft.text((60, 78), str(y_step), TFT.WHITE, sysfont.sysfont, 1, nowrap=True)
        
    elif auto_mode_screen:
        tft.text((20, 7), "CLEAR ALL", TFT.WHITE, sysfont.sysfont, 1, nowrap=True)
        tft.text((20, 21), "ADD COORDINATES", TFT.WHITE, sysfont.sysfont, 1, nowrap=True)
        tft.text((20, 35), "MAIN MENU", TFT.WHITE, sysfont.sysfont, 1, nowrap=True)
        tft.text((20, 49), "CONFIRM", TFT.WHITE, sysfont.sysfont, 1, nowrap=True)
        
        tft.text((10, 64), "Assigned", TFT.WHITE, sysfont.sysfont, 1, nowrap=True)
        tft.text((10, 78), "Coordinates", TFT.WHITE, sysfont.sysfont, 1, nowrap=True)
        tft.rect((85, 68),(28,12),TFT.YELLOW)
        tft.text((90, 70), str(len(drill_points)), TFT.WHITE, sysfont.sysfont, 1, nowrap=True)
        
    elif add_coordinates_screen:
        tft.text((20, 7), "X-POS", TFT.WHITE, sysfont.sysfont, 1, nowrap=True)
        tft.text((20, 21), "Y-POS", TFT.WHITE, sysfont.sysfont, 1, nowrap=True)
        tft.text((20, 35), "CLEAR COORDINATES", TFT.WHITE, sysfont.sysfont, 1, nowrap=True)
        tft.text((20, 49), "CONFIRM", TFT.WHITE, sysfont.sysfont, 1, nowrap=True)
        draw_figure_rects()
        tft.text((10, 64), "X:", TFT.WHITE, sysfont.sysfont, 1, nowrap=True)
        tft.text((25, 64), str(x_pos_int), TFT.WHITE, sysfont.sysfont, 1, nowrap=True)
        tft.text((45, 64), " . ", TFT.WHITE, sysfont.sysfont, 1, nowrap=True)
        tft.text((65, 64), str(x_pos_dec), TFT.WHITE, sysfont.sysfont, 1, nowrap=True)
        tft.text((10, 78), "Y:", TFT.WHITE, sysfont.sysfont, 1, nowrap=True)
        tft.text((25, 78), str(y_pos_int), TFT.WHITE, sysfont.sysfont, 1, nowrap=True)
        tft.text((45, 78), " . ", TFT.WHITE, sysfont.sysfont, 1, nowrap=True)
        tft.text((65, 78), str(y_pos_dec), TFT.WHITE, sysfont.sysfont, 1, nowrap=True)
        
    elif auto_process_screen:
        tft.text((20, 7), "ABORT", TFT.WHITE, sysfont.sysfont, 1, nowrap=True)
        tft.text((20, 21), "NEXT", TFT.WHITE, sysfont.sysfont, 1, nowrap=True)
        tft.text((20, 35), "PREVIOUS", TFT.WHITE, sysfont.sysfont, 1, nowrap=True)
        tft.text((20, 49), "DRILL", TFT.WHITE, sysfont.sysfont, 1, nowrap=True)
        draw_figure_rects()
        tft.text((10, 64), "X:", TFT.WHITE, sysfont.sysfont, 1, nowrap=True)
        tft.text((25, 64), str(x_pos_int), TFT.WHITE, sysfont.sysfont, 1, nowrap=True)
        tft.text((45, 64), " . ", TFT.WHITE, sysfont.sysfont, 1, nowrap=True)
        tft.text((65, 64), str(x_pos_dec), TFT.WHITE, sysfont.sysfont, 1, nowrap=True)
        tft.text((10, 78), "Y:", TFT.WHITE, sysfont.sysfont, 1, nowrap=True)
        tft.text((25, 78), str(y_pos_int), TFT.WHITE, sysfont.sysfont, 1, nowrap=True)
        tft.text((45, 78), " . ", TFT.WHITE, sysfont.sysfont, 1, nowrap=True)
        tft.text((65, 78), str(y_pos_dec), TFT.WHITE, sysfont.sysfont, 1, nowrap=True)

def show_yellow_button():
    if y_but_active == True:
        tft.fillrect((10, 96), (30, 30), TFT.WHITE)
        tft.fillrect((15, 101), (20, 20), TFT.YELLOW)
    else:
        tft.fillrect((10, 96), (30, 30), TFT.BLACK)
        tft.fillrect((15, 101), (20, 20), TFT.YELLOW)
        tft.line((10, 96),(15, 101), TFT.WHITE)
        tft.line((40, 96),(35, 101), TFT.WHITE)
        tft.line((10, 126),(15, 121), TFT.WHITE)
        tft.line((40, 126),(35, 121), TFT.WHITE)

def show_blue_button():
    if b_but_active == True:
        tft.fillrect((10, 128), (30, 30), TFT.WHITE)
        tft.fillrect((15, 133), (20, 20), TFT.BLUE)
    else:
        tft.fillrect((10, 128), (30, 30), TFT.BLACK)
        tft.fillrect((15, 133), (20, 20), TFT.BLUE)
        tft.line((10, 128),(15, 133), TFT.WHITE)
        tft.line((40, 128),(35, 133), TFT.WHITE)
        tft.line((10, 158),(15, 153), TFT.WHITE)
        tft.line((40, 158),(35, 153), TFT.WHITE)

def show_red_button():
    if r_but_active == True:    
        tft.fillrect((88, 128), (30, 30), TFT.WHITE)
        tft.fillrect((93, 133), (20, 20), TFT.RED)
    else:
        tft.fillrect((88, 128), (30, 30), TFT.BLACK)
        tft.fillrect((93, 133), (20, 20), TFT.RED)
        tft.line((88, 128),(93, 133), TFT.WHITE)
        tft.line((118, 128),(113, 133), TFT.WHITE)
        tft.line((88, 158),(93, 153), TFT.WHITE)
        tft.line((118, 158),(113, 153), TFT.WHITE)

def show_green_button():
    if g_but_active == True:    
        tft.fillrect((88, 96), (30, 30), TFT.WHITE)
        tft.fillrect((93, 101), (20, 20), TFT.GREEN)
    else:
        tft.fillrect((88, 96), (30, 30), TFT.BLACK)
        tft.fillrect((93, 101), (20, 20), TFT.GREEN)
        tft.line((88, 96),(93, 101), TFT.WHITE)
        tft.line((118, 96),(113, 101), TFT.WHITE)
        tft.line((88, 126),(93, 121), TFT.WHITE)
        tft.line((118, 126),(113, 121), TFT.WHITE)
        
def show_rot_button():
    if rot_but_active == True:
        tft.fillcircle((64, 127), 10, TFT.WHITE)
    else:
        tft.fillcircle((64, 127), 10, TFT.PURPLE)

def show_buttons():
    tft.fill(TFT.WHITE)

    tft.fillrect((10, 96), (30, 30), TFT.BLACK)
    tft.fillrect((15, 101), (20, 20), TFT.YELLOW)
    tft.line((10, 96),(15, 101), TFT.WHITE)
    tft.line((40, 96),(35, 101), TFT.WHITE)
    tft.line((10, 126),(15, 121), TFT.WHITE)
    tft.line((40, 126),(35, 121), TFT.WHITE)

    tft.fillrect((10, 128), (30, 30), TFT.BLACK)
    tft.fillrect((15, 133), (20, 20), TFT.BLUE)
    tft.line((10, 128),(15, 133), TFT.WHITE)
    tft.line((40, 128),(35, 133), TFT.WHITE)
    tft.line((10, 158),(15, 153), TFT.WHITE)
    tft.line((40, 158),(35, 153), TFT.WHITE)

    tft.fillrect((88, 96), (30, 30), TFT.BLACK)
    tft.fillrect((93, 101), (20, 20), TFT.GREEN)
    tft.line((88, 96),(93, 101), TFT.WHITE)
    tft.line((118, 96),(113, 101), TFT.WHITE)
    tft.line((88, 126),(93, 121), TFT.WHITE)
    tft.line((118, 126),(113, 121), TFT.WHITE)

    tft.fillrect((88, 128), (30, 30), TFT.BLACK)
    tft.fillrect((93, 133), (20, 20), TFT.RED)
    tft.line((88, 128),(93, 133), TFT.WHITE)
    tft.line((118, 128),(113, 133), TFT.WHITE)
    tft.line((88, 158),(93, 153), TFT.WHITE)
    tft.line((118, 158),(113, 153), TFT.WHITE)
    
    tft.fillcircle((64, 127), 19, TFT.BLACK)
    tft.fillcircle((64, 127), 10, TFT.PURPLE)

def show_home_screen():
    f = open("home.bmp", "rb")
    if f.read(2) == b"BM":  # header
        dummy = f.read(8)  # file size(4), creator bytes(4)
        offset = int.from_bytes(f.read(4), "little")
        hdrsize = int.from_bytes(f.read(4), "little")
        width = int.from_bytes(f.read(4), "little")
        height = int.from_bytes(f.read(4), "little")
        if int.from_bytes(f.read(2), "little") == 1:  # planes must be 1
            depth = int.from_bytes(f.read(2), "little")
            if (
                depth == 24 and int.from_bytes(f.read(4), "little") == 0
            ):  # compress method == uncompressed
                print("Image size:", width, "x", height)
                rowsize = (width * 3 + 3) & ~3
                if height < 0:
                    height = -height
                    flip = False
                else:
                    flip = True
                w, h = width, height
                if w > 128:
                    w = 128
                if h > 160:
                    h = 160
                tft._setwindowloc((0, 0), (w - 1, h - 1))
                for row in range(h):
                    if flip:
                        pos = offset + (height - 1 - row) * rowsize
                    else:
                        pos = offset + row * rowsize
                    if f.tell() != pos:
                        dummy = f.seek(pos)
                    for col in range(w):
                        bgr = f.read(3)
                        tft._pushcolor(TFTColor(bgr[2], bgr[1], bgr[0]))
                        
    tft.text((2, 5), "SEMI", TFT.RED, sysfont.sysfont, 2, nowrap=True)
    tft.text((2, 23), "AUTOMATED", TFT.GREEN, sysfont.sysfont, 1, nowrap=True)
    tft.text((2, 35), "DRILL", TFT.YELLOW, sysfont.sysfont, 2, nowrap=True)
    tft.text((2, 150), "modified_design:Hannu", TFT.YELLOW, sysfont.sysfont, 1, nowrap=True)
    
    
#------ Stepper Motor Functions ------#
    
def home_x_axis():
    s1.free_run(-1) #move backwards
    while end_stop.value(): #wait till the switch is triggered
        pass
    s1.stop()
    time.sleep(2)
    s1.speed(100)
    for i in range(23138):
        s1.step(1)
        time.sleep_ms(3) # 0.03 -0.05
    
def move_x_axis_stepper(direction, steps, disp):
    for i in range(steps):
        s1.step(direction)
        time.sleep_ms(3) # 0.03 -0.05
    if disp:
        pos = s1.get_pos()/200
        pos = round(pos,1)
        pos = str(pos)
        pos_int = pos.split('.')[0]
        pos_dec = pos.split('.')[1]
        tft.fillrect((24,63),(26,10),TFT.BLACK)
        tft.text((25, 64), pos_int, TFT.WHITE, sysfont.sysfont, 1, nowrap=True)
        tft.text((45, 64), " . ", TFT.WHITE, sysfont.sysfont, 1, nowrap=True)
        tft.fillrect((64,63),(26,10),TFT.BLACK)
        tft.text((65, 64), pos_dec, TFT.WHITE, sysfont.sysfont, 1, nowrap=True)
        
def move_polar_stepper(direction, steps, disp):
    for i in range(steps):
        s2.step(direction)
        time.sleep_ms(3) # 0.03 -0.05
    if disp:
        pos = s2.get_pos()*0.05
        pos = round(pos,1)
        pos = str(pos)
        pos_int = pos.split('.')[0]
        pos_dec = pos.split('.')[1]
        tft.fillrect((24,63),(26,10),TFT.BLACK)
        tft.text((25, 64), str(pos_int), TFT.WHITE, sysfont.sysfont, 1, nowrap=True)
        tft.text((45, 64), " . ", TFT.WHITE, sysfont.sysfont, 1, nowrap=True)
        tft.fillrect((64,63),(26,10),TFT.BLACK)
        tft.text((65, 64), str(pos_dec), TFT.WHITE, sysfont.sysfont, 1, nowrap=True)
        
def move_to_first_drill_point():
    theta = math.atan(drill_points[0][1]/drill_points[0][0])
    P1_handle = drill_points[0][1]/math.sin(theta)
    theta = math.degrees(theta)
    steps = x_axis_steps(P1_handle)
    if P1_handle > 0:
        move_x_axis_stepper(1, steps, 0)
    else:
        move_x_axis_stepper(-1, steps, 0)
    time.sleep(1)
    p_steps = polar_steps(theta)
    if theta > 0:
        print(theta)
        move_polar_stepper(1, p_steps, 0)
    else:
        print(theta)
        move_polar_stepper(-1, p_steps, 0)
      

# show_home_screen()
# time.sleep(2)
# warning_sign((15,70), "Homing X-Axis..")
# home_x_axis()
# warning_sign((15,70), "Complete")
# time.sleep(1)
s1.speed(100)
s1.overwrite_pos(0)
s2.overwrite_pos(0)
s1.stop()
s2.stop()
time.sleep(1)
show_buttons() 
show_menu()

val_old = r.value()

while True:
    if main_screen:
        if y_button.value() == 0:
            y_but_active = True
            show_yellow_button()
            time.sleep_ms(200)
            y_but_active = False
            show_yellow_button()
            print("Moving Platform Clockwise") # move clockwise platform
            steps = polar_steps(y_step)
            move_polar_stepper(1, steps, 0)
        if g_button.value() == 0:
            g_but_active = True
            show_green_button()
            time.sleep_ms(200)
            g_but_active = False
            show_green_button()
            # move anti-clockwise platform
            print("Moving Platform Anticlockwise")
            steps = polar_steps(y_step)
            move_polar_stepper(-1, steps, 0)
        if b_button.value() == 0:
            b_but_active = True
            show_blue_button()
            time.sleep_ms(200)
            b_but_active = False
            show_blue_button()
            # Home X-Axis
            print("Homing X-Axis")
        if r_button.value() == 0:
            r_but_active = True
            show_red_button()
            time.sleep_ms(200)
            r_but_active = False
            show_red_button()
            s1.overwrite_pos(0)
            s2.overwrite_pos(0)
            s1.stop()
            s2.stop()
            main_screen = False
            control_mode_screen = True
            show_menu()
    if step_screen:
        if select_x_step:
            val_new = r.value()
            if val_old != val_new:
                val_old = val_new
                if val_new == 0:
                    x_step = 0.1
                    update_x_step(x_step)
                elif val_new == 1:
                    x_step = 1
                    update_x_step(x_step)
                elif val_new == 2:
                    x_step = 5
                    update_x_step(x_step)
                elif val_new == 3:
                    x_step = 10
                    update_x_step(x_step)
                elif val_new == 4:
                    x_step = 50
                    update_x_step(x_step)
            if rot_button.value() == 0:
                rot_but_active = True
                show_rot_button()
                time.sleep_ms(200)
                rot_but_active = False
                show_rot_button()
                select_x_step = False
        if select_y_step:
            val_new = r.value()
            if val_old != val_new:
                val_old = val_new
                if val_new == 0:
                    y_step = 0.1
                elif val_new == 1:
                    y_step = 1
                elif val_new == 2:
                    y_step = 5
                elif val_new == 3:
                    y_step = 10
                elif val_new == 4:
                    y_step = 50
                update_y_step(y_step)
            if rot_button.value() == 0:
                rot_but_active = True
                show_rot_button()
                time.sleep_ms(200)
                rot_but_active = False
                show_rot_button()
                select_y_step = False
        if y_button.value() == 0:
            y_but_active = True
            show_yellow_button()
            time.sleep_ms(200)
            y_but_active = False
            show_yellow_button()
            # select x-step
            print("Selecting X-Step")
            r.set(value=0)
            r.set(min_val=0, max_val=4)
            select_x_step = True
            select_y_step = False
        if g_button.value() == 0:
            g_but_active = True
            show_green_button()
            time.sleep_ms(200)
            g_but_active = False
            show_green_button()
            # select y-step
            print("Selecting Y-Step")
            r.set(value=0)
            r.set(min_val=0, max_val=4)
            select_y_step = True
            select_x_step = False
        if b_button.value() == 0 and select_x_step == False and select_y_step == False:
            b_but_active = True
            show_blue_button()
            time.sleep_ms(200)
            b_but_active = False
            show_blue_button()
            step_screen = False
            main_screen = True
            show_menu()
        if r_button.value() == 0 and select_x_step == False and select_y_step == False:
            r_but_active = True
            show_red_button()
            time.sleep_ms(200)
            r_but_active = False
            show_red_button()
            step_screen = False
            control_mode_screen = True
            show_menu()
    if control_mode_screen:
        if y_button.value() == 0:
            y_but_active = True
            show_yellow_button()
            time.sleep_ms(200)
            y_but_active = False
            show_yellow_button()
            control_mode_screen = False
            manual_mode_screen = True
            show_menu()
        if g_button.value() == 0:
            g_but_active = True
            show_green_button()
            time.sleep_ms(200)
            g_but_active = False
            show_green_button()
            control_mode_screen = False
            auto_mode_screen = True
            show_menu() 
        if b_button.value() == 0:
            b_but_active = True
            show_blue_button()
            time.sleep_ms(200)
            b_but_active = False
            show_blue_button()
            control_mode_screen = False
            main_screen = True
            show_menu()
        if r_button.value() == 0:
            r_but_active = True
            show_red_button()
            time.sleep_ms(200)
            r_but_active = False
            show_red_button()
            control_mode_screen = False
            step_screen = True
            show_menu()
    if manual_mode_screen:
        if y_button.value() == 0:
            y_but_active = True
            show_yellow_button()
            time.sleep_ms(200)
            y_but_active = False
            show_yellow_button()
            manual_mode_screen = False
            move_x_screen = True
            show_menu()
        if g_button.value() == 0:
            g_but_active = True
            show_green_button()
            time.sleep_ms(200)
            g_but_active = False
            show_green_button()
            manual_mode_screen = False
            move_y_screen = True
            show_menu()
        if b_button.value() == 0:
            b_but_active = True
            show_blue_button()
            time.sleep_ms(200)
            b_but_active = False
            show_blue_button()
            manual_mode_screen = False
            main_screen = True
            show_menu()
        if r_button.value() == 0:
            r_but_active = True
            show_red_button()
            time.sleep_ms(200)
            r_but_active = False
            show_red_button()
            if drill_active:
                print("Drill Stopped")
                drill_active = False
                drill.value(1)
            else:
                drill_active = True
                print("Drill Started")
                drill.value(0)

    if move_x_screen:
        if y_button.value() == 0:
            y_but_active = True
            show_yellow_button()
            time.sleep_ms(200)
            y_but_active = False
            show_yellow_button()
            # move X positive direction
            steps = x_axis_steps(x_step)
            move_x_axis_stepper(1, steps, 1)

        if g_button.value() == 0:
            g_but_active = True
            show_green_button()
            time.sleep_ms(200)
            g_but_active = False
            show_green_button()
            # move X negetive direction
            steps = x_axis_steps(x_step)
            move_x_axis_stepper(-1, steps, 1)

        if b_button.value() == 0:
            b_but_active = True
            show_blue_button()
            time.sleep_ms(200)
            b_but_active = False
            show_blue_button()
            move_x_screen = False
            manual_mode_screen = True
            show_menu()
        if r_button.value() == 0:
            r_but_active = True
            show_red_button()
            time.sleep_ms(200)
            r_but_active = False
            show_red_button()
            move_x_screen = False
            step_screen = True
            show_menu()
    if move_y_screen:
        if y_button.value() == 0:
            y_but_active = True
            show_yellow_button()
            time.sleep_ms(200)
            y_but_active = False
            show_yellow_button()
            # move Y positive direction
            print("move Y Positive")
            steps = polar_steps(y_step)
            move_polar_stepper(1, steps, 1)
        if g_button.value() == 0:
            g_but_active = True
            show_green_button()
            time.sleep_ms(200)
            g_but_active = False
            show_green_button()
            # move Y negetive direction
            print("move Y Negetive")
            steps = polar_steps(y_step)
            move_polar_stepper(-1, steps, 1)
        if b_button.value() == 0:
            b_but_active = True
            show_blue_button()
            time.sleep_ms(200)
            b_but_active = False
            show_blue_button()
            move_y_screen = False
            manual_mode_screen = True
            show_menu()
        if r_button.value() == 0:
            r_but_active = True
            show_red_button()
            time.sleep_ms(200)
            r_but_active = False
            show_red_button()
            move_y_screen = False
            step_screen = True
            show_menu()
                
    if auto_mode_screen:
        if y_button.value() == 0:
            y_but_active = True
            show_yellow_button()
            time.sleep_ms(200)
            y_but_active = False
            show_yellow_button()
            drill_points.clear()
            tft.fillrect((7,61),(116, 28), TFT.BLACK)
            tft.text((40, 64), "Cleared", TFT.WHITE, sysfont.sysfont, 1, nowrap=True)
            tft.text((30, 78), "Coordinates", TFT.WHITE, sysfont.sysfont, 1, nowrap=True)
            time.sleep(0.5)
            tft.fillrect((7,61),(116, 28), TFT.BLACK)
            tft.text((10, 64), "Assigned", TFT.WHITE, sysfont.sysfont, 1, nowrap=True)
            tft.text((10, 78), "Coordinates", TFT.WHITE, sysfont.sysfont, 1, nowrap=True)
            tft.rect((85, 68),(28,12),TFT.YELLOW)
            tft.text((90, 70), str(len(drill_points)), TFT.WHITE, sysfont.sysfont, 1, nowrap=True)
            print("Clearing Drill Points List")
        if g_button.value() == 0:
            g_but_active = True
            show_green_button()
            time.sleep_ms(200)
            g_but_active = False
            show_green_button()
            auto_mode_screen = False
            add_coordinates_screen = True
            show_menu()
        if b_button.value() == 0:
            b_but_active = True
            show_blue_button()
            time.sleep_ms(200)
            b_but_active = False
            show_blue_button()
            auto_mode_screen = False
            main_screen = True
            show_menu()
        if r_button.value() == 0:
            r_but_active = True
            show_red_button()
            time.sleep_ms(200)
            r_but_active = False
            show_red_button()
            auto_mode_screen = False
            auto_process_screen = True
            if len(drill_points) != 0:
                drill_point = drill_points[drill_index]
                # Move to first drill point
                print("Moving to first drill point")
                warning_sign((15,70), "Moving to Point ")
                print(drill_points[drill_index])
                move_to_first_drill_point()
                time.sleep(1)
            else:
                print("Drill Point undefined")
                # show warning!
                warning_sign((15,70), "Point Undefined!")
                time.sleep(1)
                auto_mode_screen = True
                auto_process_screen = False
            show_buttons()
            show_menu()
            if auto_process_screen:
                x_int, x_dec, y_int, y_dec = break_up_to_int_dec(drill_points[0][0], drill_points[0][1])
                update_x_int(x_int)
                update_x_dec(x_dec)
                update_y_int(y_int)
                update_y_dec(y_dec)
  
    if add_coordinates_screen:
        if select_x_pos_dec:
            val_new = r.value()
            if val_old != val_new:
                val_old = val_new
                update_x_dec(val_new)
            if rot_button.value() == 0:
                rot_but_active = True
                show_rot_button()
                time.sleep_ms(200)
                rot_but_active = False
                show_rot_button()
                x_pos_dec = val_new
                select_x_pos_int = False
                select_x_pos_dec = False
                select_y_pos_int = False
                select_y_pos_dec = False
        if select_x_pos_int:
            val_new = r.value()
            if val_old != val_new:
                val_old = val_new
                update_x_int(val_new)
            if rot_button.value() == 0:
                rot_but_active = True
                show_rot_button()
                time.sleep_ms(200)
                rot_but_active = False
                show_rot_button()
                x_pos_int = val_new
                select_x_pos_int = False
                select_x_pos_dec = True
                select_y_pos_int = False
                select_y_pos_dec = False
                r.set(value=0)
                r.set(min_val=0, max_val=9)
        if select_y_pos_int:
            val_new = r.value()
            if val_old != val_new:
                val_old = val_new
                update_y_int(val_new)
            if rot_button.value() == 0:
                rot_but_active = True
                show_rot_button()
                time.sleep_ms(200)
                rot_but_active = False
                show_rot_button()
                y_pos_int = val_new
                select_x_pos_int = False
                select_x_pos_dec = False
                select_y_pos_int = False
                select_y_pos_dec = True
                r.set(value=0)
                r.set(min_val=0, max_val=9)
        if select_y_pos_dec:
            val_new = r.value()
            if val_old != val_new:
                val_old = val_new
                update_y_dec(val_new)
            if rot_button.value() == 0:
                rot_but_active = True
                show_rot_button()
                time.sleep_ms(200)
                rot_but_active = False
                show_rot_button()
                y_pos_dec = val_new
                select_x_pos_int = False
                select_x_pos_dec = False
                select_y_pos_int = False
                select_y_pos_dec = False
        if y_button.value() == 0:
            y_but_active = True
            show_yellow_button()
            time.sleep_ms(200)
            y_but_active = False
            show_yellow_button()
            r.set(value=0)
            r.set(min_val=-85, max_val=85)
            select_x_pos_int = True
            select_y_pos_int = False
            select_x_pos_dec = False
            select_y_pos_dec = False
            
        if g_button.value() == 0:
            g_but_active = True
            show_green_button()
            time.sleep_ms(200)
            g_but_active = False
            show_green_button()
            r.set(value=0)
            r.set(min_val=-85, max_val=85)
            select_x_pos_int = False
            select_y_pos_int = True
            select_x_pos_dec = False
            select_y_pos_dec = False

        if b_button.value() == 0:
            b_but_active = True
            show_blue_button()
            time.sleep_ms(200)
            b_but_active = False
            show_blue_button()
            x_pos_int = 0
            y_pos_int = 0
            x_pos_dec = 0
            y_pos_dec = 0
            update_x_int(x_pos_int)
            update_x_dec(x_pos_dec)
            update_y_int(x_pos_int)
            update_y_dec(y_pos_dec)  
        if r_button.value() == 0 and select_x_pos_int == False and select_y_pos_int == False and select_x_pos_dec == False and select_y_pos_dec == False:
            r_but_active = True
            show_red_button()
            time.sleep_ms(200)
            r_but_active = False
            show_red_button()
            x_pos = get_final_pos(x_pos_int, x_pos_dec)
            y_pos = get_final_pos(y_pos_int, y_pos_dec)
            if is_valid_coordinates((x_pos, y_pos)):
                drill_points.append((x_pos, y_pos))
            else:
                x_pos = 0
                y_pos = 0
                warning_sign((15,70), "Invalid Position!")
                time.sleep(1)
                show_buttons()
            add_coordinates_screen = False
            auto_mode_screen = True
            show_menu()
            
    if auto_process_screen:
        if y_button.value() == 0:
            y_but_active = True
            show_yellow_button()
            time.sleep_ms(200)
            y_but_active = False
            show_yellow_button()
            auto_process_screen = False
            main_screen = True
            show_menu()
        if g_button.value() == 0:
            g_but_active = True
            show_green_button()
            time.sleep_ms(200)
            g_but_active = False
            show_green_button()
            drill_index+=1
            if len(drill_points)>drill_index:
                drill_point = drill_points[drill_index]
                # move to drill point
                warning_sign((15,70), "Moving to Point")
                time.sleep(1)
                print("Moving to next drill point ")
                print(drill_points[drill_index])
                beta, x_axis_handle_adjust, rotation_direction = calculate_path_trajectory(drill_points[drill_index-1], drill_points[drill_index])
                steps = x_axis_steps(x_axis_handle_adjust)
                print(x_axis_handle_adjust)
                if x_axis_handle_adjust > 0:
                    move_x_axis_stepper(1, steps, 0)
                if x_axis_handle_adjust < 0:
                    move_x_axis_stepper(-1, steps, 0)
                p_steps = polar_steps(beta)
                move_polar_stepper(rotation_direction, p_steps, 0)
            else:
                # show warning!
                warning_sign((15,70), "Point Undefined!")
                time.sleep(1)
                print("drill point undefined")
                auto_mode_screen = True
                auto_process_screen = False
            show_buttons()
            show_menu()
            if auto_process_screen:
                x_int, x_dec, y_int, y_dec = break_up_to_int_dec(drill_points[drill_index][0], drill_points[drill_index][1])
                update_x_int(x_int)
                update_x_dec(x_dec)
                update_y_int(y_int)
                update_y_dec(y_dec)
        if b_button.value() == 0:
            b_but_active = True
            show_blue_button()
            time.sleep_ms(200)
            b_but_active = False
            show_blue_button()
            if drill_index != 0:
                if len(drill_points)>drill_index:
                    drill_point = drill_points[drill_index]
                    # move to drill point
                    warning_sign((15,70), "Moving to Point")
                    time.sleep(1)
                    print("Moving to previous drill point ")
                    print(drill_points[drill_index])
                    beta, x_axis_handle_adjust, rotation_direction = calculate_path_trajectory(drill_points[drill_index], drill_points[drill_index-1])
                    steps = x_axis_steps(x_axis_handle_adjust)
                    print(x_axis_handle_adjust)
                    if x_axis_handle_adjust > 0:
                        move_x_axis_stepper(1, steps, 0)
                    if x_axis_handle_adjust < 0:
                        move_x_axis_stepper(-1, steps, 0)
                    time.sleep(1)
                    p_steps = polar_steps(beta)
                    move_polar_stepper(rotation_direction, p_steps, 0)
                    drill_index-=1
                    x_int, x_dec, y_int, y_dec = break_up_to_int_dec(drill_points[drill_index][0], drill_points[drill_index][1])
                    update_x_int(x_int)
                    update_x_dec(x_dec)
                    update_y_int(y_int)
                    update_y_dec(y_dec)
                else:
                    print("drill point undefined")
                    # show warning
                    warning_sign((15,70), "Point Undefined!")
                    time.sleep(1)
                    auto_mode_screen = True
                    auto_process_screen = False
            else:
                print("drill point undefined")
                # show warning
                warning_sign((15,70), "Point Undefined!")
                time.sleep(1)
                auto_mode_screen = True
                auto_process_screen = False
            show_buttons()
            show_menu()
            if auto_process_screen:
                x_int, x_dec, y_int, y_dec = break_up_to_int_dec(drill_points[drill_index][0], drill_points[drill_index][1])
                update_x_int(x_int)
                update_x_dec(x_dec)
                update_y_int(y_int)
                update_y_dec(y_dec)
        if r_button.value() == 0:
            r_but_active = True
            show_red_button()
            time.sleep_ms(200)
            r_but_active = False
            show_red_button()
            if drill_active:
                print("Drill Stopped")
                drill_active = False
                drill.value(1)
            else:
                drill_active = True
                print("Drill Started")
                drill.value(0)
                
    
