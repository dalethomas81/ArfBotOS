
( G17 xy plane | G18 zx plane | G16 ijk normal ) 
( G91 incremental | G99 IJK relative to current TCP ) 
( G55 rel DCS | G56 current pose = DCS origin | G53 reset DCS ) 
( G51 start angle smoothing | G50 end ) 
( A yaw, B pitch, C roll. Trafo is yaw/pitch/roll. ) 
( Jog TCP onto the apex, tool Z along the cone axis, then run. ) 

N000 G91 ( incremental, like CNC_FromFile ) 
N010 G17 ( xy plane - circle is perpendicular to the cone axis ) 

(N040 M510) ( wait for flag 10 )
N050 G01 X-60 Y0 Z-80 A0 B10 C0 E1000 F200

N070 G02 X60 Y60 R60 A0 B-10 C10
N070 G02 X60 Y-60 R60 A0 B-10 C-10
N070 G03 X-60 Y60 R60 A0 B10 C10
N070 G03 X-60 Y-60 R60 A0 B10 C-10
N070 G03 X60 Y-60 R60 A0 B-10 C-10
N070 G03 X60 Y60 R60 A0 B-10 C10

N140 G53 ( reset decoder coordinate system )