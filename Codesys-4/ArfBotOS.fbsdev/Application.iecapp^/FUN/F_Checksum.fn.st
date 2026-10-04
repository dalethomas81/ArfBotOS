FUNCTION F_Checksum : UINT
VAR_INPUT
	data: POINTER TO BYTE;
	length: UINT;
END_VAR
VAR
	curr_crc: UINT;
	sum1, sum2: USINT;
	_i, _j, _k: UINT;
END_VAR

// https://www.tutorialspoint.com/cyclic-redundancy-check-crc-in-arduino
curr_crc := 16#0000;
sum1 := UINT_TO_USINT(curr_crc);
sum2 := UINT_TO_USINT(SHR(curr_crc, 8));
FOR _i:=0 TO length - 1 BY 1 DO
	sum1 := (sum1 + data[_i]) MOD 255;
	sum2 := (sum2 + sum1) MOD 255;
END_FOR
{warning disable C0308}
F_Checksum := SHL(sum2, 8) OR sum1;
{warning restore C0308}
END_FUNCTION
