( G91 incremental | G17 xy | G02 cw with XY end and R )
( Trafo ABC = yaw / pitch / roll. Look-at the apex at every corner. )
( Jog TCP onto the apex, tool Z along the cone axis, then run. )
( H = 40, R = 23.094, cone half-angle = 30 )
( )
( End of each quarter, ABC that point tool Z at the apex: )
(   start / N060: A0   B30  C0    lean in +X, pitch )
(   N070 +Y:      A0   B0   C30   lean in -Y, roll )
(   N080 +X:      A0   B-30 C0    lean in -X, pitch the other way )
(   N090 -Y:      A0   B0   C-30  lean in +Y, roll the other way )
(   N100 -X:      A0   B30  C0    back to start )
( G91 values below are the deltas that reach those end angles. )

N000 G91
N020 G17

N050 G01 X0 Y0 Z0 A0 B30 C0 F30
N060 G01 X-23.094 Y0 Z-40 A0 B0 C0 F30

N070 G02 X23.094 Y23.094 R23.094 A0 B-30 C30 F30
N080 G02 X23.094 Y-23.094 R23.094 A0 B-30 C-30 F30
N090 G02 X-23.094 Y-23.094 R23.094 A0 B30 C-30 F30
N100 G02 X-23.094 Y23.094 R23.094 A0 B30 C30 F30

N110 G01 X23.094 Y0 Z40 A0 B-30 C0 F30
N130 G53
