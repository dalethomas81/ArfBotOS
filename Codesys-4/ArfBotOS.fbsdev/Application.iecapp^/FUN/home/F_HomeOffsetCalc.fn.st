FUNCTION F_HomeOffsetCalc : LREAL
VAR_INPUT
	CurrentOffset: LREAL;
	CurrentPosition: LREAL;
	HomeSearchDirection: MC_Direction;
END_VAR
VAR
END_VAR

IF HomeSearchDirection = MC_Direction.positive THEN
	F_HomeOffsetCalc := -F_RoundLreal(ABS(CurrentPosition - CurrentOffset),2);
	
ELSIF HomeSearchDirection = MC_Direction.negative THEN
	F_HomeOffsetCalc := F_RoundLreal(ABS(CurrentPosition - CurrentOffset),2);
	
ELSE
	F_HomeOffsetCalc := CurrentOffset;
	
END_IF
END_FUNCTION
