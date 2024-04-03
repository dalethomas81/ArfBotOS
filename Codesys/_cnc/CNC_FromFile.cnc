N000 G91 ( set relative )
N010 G01 X10 Y0 Z0 A0 B0 C0 F$R1$ 
N100 G18 ( set plane )
N120 G55 X0 Y0 Z0 A0 B0 C0 ( relative transformation )
N130 G03 I20 J40 K30 A0 B60 C0 T180 ( ccw circle/segment )
N131 G53 ( end transformation of coordinate system and resets to normal )