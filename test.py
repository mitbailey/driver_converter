import math as m
import numpy as np

# Machine specific.
MM_TO_IDX = 2184560.64


# grating_density = 833.333 # grating density
# diff_order = 1
# zero_ofst = 0.34 # mm
# arm_length = 550.0 # mm
# incidence_ang = 32 # deg
# tangent_ang = 0 # deg
# conversion_slope = 2 / grating_density * 1e3 * np.cos(np.pi * incidence_ang / 180) * np.cos(np.pi * tangent_ang / 180) / arm_length * MM_TO_IDX / diff_order # verify eqn

# print("Conversion slope (Sunip): " + str(conversion_slope))
# print("...with offset: " + str(conversion_slope - zero_ofst))
# print("CONVERSION: " + str(INPUT_NM_POSITION) + " nm --> " + str(INPUT_NM_POSITION / conversion_slope) + " mm ")

# self.currpos_mm_disp.setText('Current: %.4f nm'%(self.current_position / self.conversion_slope - self.zero_ofst))

INPUT_POSITION_NM = 550
arm_length_mm = 56.53654
order = 1
grating_density_grv_mm = 1200
tan_ang_deg = 0
inc_ang_deg = 32
zero_offset_nm = 0

pos_mm = ((arm_length_mm * order * grating_density_grv_mm)/(2 * (m.cos(m.radians(tan_ang_deg))) * (m.cos(m.radians(inc_ang_deg))) * 1e6)) * (INPUT_POSITION_NM + zero_offset_nm)

print(str(INPUT_POSITION_NM) + " nm")
print(str(pos_mm) + " mm")
print("Conversion slope: " + str(((arm_length_mm * order * grating_density_grv_mm)/(2 * (m.cos(m.radians(tan_ang_deg))) * (m.cos(m.radians(inc_ang_deg))) * 1e6))))
