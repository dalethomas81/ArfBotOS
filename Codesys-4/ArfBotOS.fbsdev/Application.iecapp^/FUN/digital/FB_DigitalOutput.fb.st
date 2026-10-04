FUNCTION_BLOCK FB_DigitalOutput
VAR_INPUT
END_VAR
VAR_OUTPUT
END_VAR
VAR	
	// https://infosys.beckhoff.com/content/1033/tc3_plc_intro/11948825611.html?id=1756449079294237528
	Output AT %Q* : BOOL;
	_Output: BOOL;
	_FirstScan: BOOL;
END_VAR

IF _FirstScan THEN
	//
END_IF

// do some things?
Output :=_Output;

// do not put anything below this
_FirstScan := FALSE;
END_FUNCTION_BLOCK
