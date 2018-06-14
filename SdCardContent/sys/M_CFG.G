M92 X57.14 Y57.14 Z57.14 E55.00 ; Steps per unit
M203 X150.00 Y150.00 Z150.00 E50.00 ; Maximum feedrates (mm/s)
M201 X800 Y800 Z800 E10000 ; Maximum Acceleration (mm/s2)
M204 P3000.00 R3000.00 T3000.00 ; S=acceleration, T=retract acceleration
M205 S0.00 T0.00 B20000 X20.00 Z20.00 E5.00 ; S=Min feedrate (mm/s), T=Min travel feedrate (mm/s), B=minimum segment time (ms), X=maximum XY jerk (mm/s),  Z=maximum Z jerk (mm/s),  E=maximum E jerk (mm/s)
M206 X0.00 Y0.00 Z0.00 ; Home offset (mm)
M666 X0.00 Y0.00 Z0.00 ; Endstop adjustment (mm)
M665 L120.80 R63.50 S200.00 A0.00 B0.00 C0.00 ; Delta settings: L=diagonal_rod, R=radius, S=segments_per_second, ABC=diagonal_rod_trim_tower_[123]
M301 P20.00 I1.60 D56.77 ; PID settings
M304 P80.00 I12.71 D300.00 ; Bed PID settings
M851 Z0.00 ; Z offset
