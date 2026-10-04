METHOD PRIVATE _M_Processor : BOOL
VAR_INPUT
END_VAR

IF GVL_App.ResetRtrig.Q THEN
	ProcessPrograms.M_Home();
END_IF

ProcessPrograms(
	Programs:=PersistentVars.Programs,
	StoredPositions:=PersistentVars.StoredPositions,
	VisionRegisters:=PersistentVars.VisionRegisters,
	Flags:=PersistentVars.Flags,
	J1:=IoConfig_Globals.J1, J2:=IoConfig_Globals.J2, J3:=IoConfig_Globals.J3, J4:=IoConfig_Globals.J4, J5:=IoConfig_Globals.J5, J6:=IoConfig_Globals.J6,
	AxisGroup:= ArfBotAxisGroup,
	GroupHandler:= GroupHandler,
	CmdIO:=GVL_App.CmdIO,
	Enable:= GVL_App.Enable AND PersistentVars.Flags[GVL_App.HOME_COMPLETE].Value AND NOT GVL_App.xActiveAlarm,
	Error=> ,
	StatusText=>);

IF ProcessPrograms.PropBusy THEN
    // for displaying on the HMI
    // PGM: <number> CMD: <number> Type: <type> Command: <command> Comment: <comment>
    ActiveCommandDetails := CONCAT('PGM: ', UINT_TO_STRING(ProcessPrograms.P_CurrentProgram));
    ActiveCommandDetails := CONCAT(ActiveCommandDetails, ' | CMD: ');
    ActiveCommandDetails := CONCAT(ActiveCommandDetails,
                                    // TODO: the "current" index in this case is actually next
                                    // use a SEL here because I dont want to always do "index minus 1" because this would show a -1 on the HMI when Idle and thats gross
                                    UINT_TO_STRING(SEL(ProcessPrograms.P_CurrentCommandIndex <= 0,
                                                        ProcessPrograms.P_CurrentCommandIndex - 1, // FALSE
                                                        0))); // TRUE
    ActiveCommandDetails := CONCAT(ActiveCommandDetails, ' | ');
    ActiveCommandDetails := CONCAT(ActiveCommandDetails, TO_STRING(ProcessPrograms.P_CurrentCommand.CommandType));
    ActiveCommandDetails := CONCAT(ActiveCommandDetails, ': ');
    ActiveCommandDetails := CONCAT(ActiveCommandDetails, ProcessPrograms.P_CurrentCommand.CommandString);
    IF ProcessPrograms.P_CurrentCommand.CommandType = DUT_CommandType.CncCommand THEN
        ActiveCommandDetails := CONCAT(ActiveCommandDetails, ' | G-Code: ');
        ActiveCommandDetails := CONCAT(ActiveCommandDetails, ProcessPrograms.GCodeLines[0]);
    END_IF
    ActiveCommandDetails := CONCAT(ActiveCommandDetails, ' | Comment: ');
    ActiveCommandDetails := CONCAT(ActiveCommandDetails, ProcessPrograms.P_CurrentCommand.CommandComment);
END_IF

// disable all IO when enable drops
IF EnableFtrig.Q THEN
	FOR _i:=GVL_App.DIG_OUT_ARR_BEG TO GVL_App.DIG_OUT_ARR_END BY 1 DO
		GVL_App.CmdIO.Output[_i] := 0;
	END_FOR
	FOR _i:=GVL_App.PWM_OUT_ARR_BEG TO GVL_App.PWM_OUT_ARR_END BY 1 DO
		GVL_App.CmdIO.Pwm[_i] := 0;
	END_FOR
END_IF
END_METHOD
