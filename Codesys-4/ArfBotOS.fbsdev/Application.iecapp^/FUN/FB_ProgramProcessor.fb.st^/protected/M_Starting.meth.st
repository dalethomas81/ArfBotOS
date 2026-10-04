METHOD PROTECTED M_Starting : BOOL

// here we want to clear things and get ready for the execute state
// we can consider this the "INIT" state
(*FOR _i:=GVL_App.PROG_ARR_BEG TO GVL_App.PROG_ARR_END BY 1 DO
	_Returning[_i] := FALSE;
END_FOR
FOR _i:=GVL_App.PROG_ARR_BEG TO GVL_App.PROG_ARR_END BY 1 DO
	_ReturnPrograms[_i] := 0;
END_FOR
FOR _i:=GVL_App.PROG_CMDS_ARR_BEG TO GVL_App.PROG_CMDS_ARR_END BY 1 DO
	_ReturnCommands[_i] := 0;
END_FOR*)
//_CurrentPrgIdx := 0;
_SeqMain := _STEP_PRG;
SUPER^.M_StateComplete();
END_METHOD
