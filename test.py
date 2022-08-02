import math as m

def dcos(deg):
    return m.degrees((m.cos(m.radians(32))))
    
MM_TO_NM = 10e6
MM_TO_IDX = 2184532 # Based on motor/stage...

DESIRED_POSITION_NM = 0

order = 1
zero_order_offset = 1
L = 550
grating_density = 0.0012
dX = DESIRED_POSITION_NM
a = ((2) * (1 / grating_density) * dcos(32) * ((dX + zero_order_offset)/(L)) * (MM_TO_NM)) / (order)
print("a: " + str(a))