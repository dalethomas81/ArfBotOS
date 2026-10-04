METHOD PROTECTED M_Idle : BOOL

// in this state, we want to reset all the function blocks and hang out
FOR _i:=GVL_App.PROG_ARR_BEG TO GVL_App.PROG_ARR_END BY 1 DO
	_Returning[_i] := FALSE;
END_FOR
(*FOR _i:=GVL_App.PROG_ARR_BEG TO GVL_App.PROG_ARR_END BY 1 DO
	_ReturnPrograms[_i] := 0;
END_FOR
FOR _i:=GVL_App.PROG_CMDS_ARR_BEG TO GVL_App.PROG_CMDS_ARR_END BY 1 DO
	_ReturnCommands[_i] := 0;
END_FOR*)
_SeqMain := _STEP_IDLE;
_SeqAborting := 0;
_SeqClearing := 0;
_SeqResetting := 0;
END_METHOD
