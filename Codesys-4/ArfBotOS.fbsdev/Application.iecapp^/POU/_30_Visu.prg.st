PROGRAM _30_Visu
VAR	
    // alarms
	itfAlarmConfiguration : AlarmManager.IAlarmConfiguration;
    itfAlarmConfiguration7 : AlarmManager.IAlarmConfiguration7;
    iAlarmCfg : INT;
    
    //
	SelectedProgramNumber: INT;
	SelectedRecipeRowIndex: INT;
    SelectedRecipeRowIndexLast: INT;
	RecipeRowSelected: BOOL;
	InsertLineAbove: BOOL;
	InsertLineBelow: BOOL;
	DeleteLine: BOOL;
	CopySelectedCommand: BOOL;
	CopiedCommandType: DUT_CommandType;
	CopiedCommandString: STRING(GVL_App.COMMAND_LEN);
	CopiedCommandComment: STRING;
	PasteCommand: BOOL;
	JumpRowChanged: BOOL;
    ScrollingBegin: BOOL;
    ScrollingSingleUp: BOOL;
    ScrollingSingleDown: BOOL;
    ScrollingPageUp: BOOL;
    ScrollingPageDown: BOOL;
    ScrollingEnd: BOOL;
	
	_i,_j,_k: INT;
	
	SetStoredPositionAsCurrent: BOOL;
	SelectedStoredPosition: UINT;
	SetVisionRegisterAsCurrent: BOOL;
	SelectedVisionRegister: UINT;
	SelectedFlag: UINT;
	
	//
	TeachPosition: BOOL;
	stTeachPosition: ST_MoveCmd;
	
	//
	MoveCmdParser: FB_ParseMoveCmd;
	stMoveCmd: ST_MoveCmd;
	MoveCmdVisu: AddMoveCmd_VISU_STRUCT;
	
	//
	OutputCmdParser: FB_ParseOutputCmd;
	stOutputCmd: ST_OutputCmd;
	OutputCmdVisu: AddOutputCmd_VISU_STRUCT;
	
	//
	PauseCmdParser: FB_ParsePauseCmd;
	stPauseCmd: ST_PauseCmd;
	PauseCmdVisu: AddPauseCmd_VISU_STRUCT;
	
	//
	VisionCmdParser : FB_ParseVisionCmd;
	stVisionCmd : ST_VisionCmd;
	VisionCmdVisu: AddVisionCmd_VISU_STRUCT;
	
	//
	VarSetCmdParser : FB_ParseVarSetCmd;
	stVarSetCmd : ST_VarSetCmd;
	VarSetCmdVisu: AddVarSetCmd_VISU_STRUCT;
	
	//
	CncCmdParser : FB_ParseCncCmd;
	stCncCmd : ST_CncCmd;
	CncCmdVisu: AddCncCmd_VISU_STRUCT;
	
	//
	LogicCmdParser : FB_ParseLogicCmd;
	stLogicCmd : ST_LogicCmd;
	LogicCmdVisu: AddLogicCmd_VISU_STRUCT;
	
	//
	CurrentVisu: STRING;
	OpenDialog: VU.FbOpenDialogExtended;

END_VAR

//
GVL_App.lrGripperOpen := SEL(GVL_App.CmdIO.Output[0], 0.0, 100.0);

// scrolling
IF ScrollingBegin OR ScrollingSingleUp OR ScrollingSingleDown OR ScrollingPageUp OR ScrollingPageDown OR ScrollingEnd
    AND_THEN SelectedRecipeRowIndex <> SelectedRecipeRowIndexLast THEN
        SelectedRecipeRowIndexLast := SelectedRecipeRowIndex;
        ScrollingBegin := ScrollingSingleUp := ScrollingSingleDown := ScrollingPageUp := ScrollingPageDown := ScrollingEnd := FALSE;
END_IF

//
IF (InsertLineAbove OR InsertLineBelow) AND RecipeRowSelected THEN
	//
	IF InsertLineAbove THEN
		InsertLineAbove := FALSE;
		_j := SelectedRecipeRowIndex + 1;
	ELSIF InsertLineBelow THEN
		InsertLineBelow := FALSE;
		_j := SelectedRecipeRowIndex + 2;
	END_IF
	//
	FOR _i := GVL_App.PROG_CMDS_ARR_END TO _j BY -1 DO
		// shift all data
		IF _i >= GVL_App.PROG_CMDS_ARR_BEG AND _i <= GVL_App.PROG_CMDS_ARR_END THEN // can never really be too careful am I right?
			//
			MemUtils.MemCpy(pbyDest:=ADR(PersistentVars.Programs[SelectedProgramNumber].Commands[_i]),
							pbySrc:=ADR(PersistentVars.Programs[SelectedProgramNumber].Commands[_i - 1]),
							dwSize:=SIZEOF(PersistentVars.Programs[SelectedProgramNumber].Commands[_i]));
		END_IF
	END_FOR
	FOR _k := GVL_App.PROG_CMDS_ARR_END TO GVL_App.PROG_CMDS_ARR_BEG BY -1 DO
		// check if the command is a logic command with a jump function
		// if so, we need to move the jump row automatically so it doesnt point to an invalid command.
		// otherwise, the user would have to manually change this each time they move rows.
		JumpRowChanged := F_ChangeJumpRow(Command:=PersistentVars.Programs[SelectedProgramNumber].Commands[_k], BeginningRow:= _j-1, Change:= 1);
	END_FOR
	//
	PersistentVars.Programs[SelectedProgramNumber].Commands[_j - 1].CommandType := DUT_CommandType.Disabled;
	PersistentVars.Programs[SelectedProgramNumber].Commands[_j - 1].CommandString := '';
	PersistentVars.Programs[SelectedProgramNumber].Commands[_j - 1].CommandComment := '';
END_IF

//
IF DeleteLine AND RecipeRowSelected THEN
	DeleteLine := FALSE;
	FOR _i := SelectedRecipeRowIndex TO GVL_App.PROG_CMDS_ARR_END - 1 BY 1 DO
		IF _i >= GVL_App.PROG_CMDS_ARR_BEG AND _i <= GVL_App.PROG_CMDS_ARR_END THEN // can never really be too careful am I right?
			//
			MemUtils.MemCpy(pbyDest:=ADR(PersistentVars.Programs[SelectedProgramNumber].Commands[_i]),
							pbySrc:=ADR(PersistentVars.Programs[SelectedProgramNumber].Commands[_i + 1]),
							dwSize:=SIZEOF(PersistentVars.Programs[SelectedProgramNumber].Commands[_i]));
		END_IF
	END_FOR
	FOR _k := GVL_App.PROG_CMDS_ARR_END TO GVL_App.PROG_CMDS_ARR_BEG BY -1 DO
		// check if the command is a logic command with a jump function
		// if so, we need to move the jump row automatically so it doesnt point to an invalid command.
		// otherwise, the user would have to manually change this each time they move rows.
		JumpRowChanged := F_ChangeJumpRow(Command:=PersistentVars.Programs[SelectedProgramNumber].Commands[_k], BeginningRow:= SelectedRecipeRowIndex, Change:= -1);
	END_FOR
END_IF

//
IF SetStoredPositionAsCurrent THEN
	SetStoredPositionAsCurrent := FALSE;
	PersistentVars.StoredPositions[SelectedStoredPosition].Position.X := F_RoundLreal(_00_Main.ProcessJog.Position.v[0],2);
	PersistentVars.StoredPositions[SelectedStoredPosition].Position.Y := F_RoundLreal(_00_Main.ProcessJog.Position.v[1],2);
	PersistentVars.StoredPositions[SelectedStoredPosition].Position.Z := F_RoundLreal(_00_Main.ProcessJog.Position.v[2],2);
	PersistentVars.StoredPositions[SelectedStoredPosition].Position.A := F_RoundLreal(_00_Main.ProcessJog.Position.v[3],2);
	PersistentVars.StoredPositions[SelectedStoredPosition].Position.B := F_RoundLreal(_00_Main.ProcessJog.Position.v[4],2);
	PersistentVars.StoredPositions[SelectedStoredPosition].Position.C := F_RoundLreal(_00_Main.ProcessJog.Position.v[5],2);
END_IF

//
IF TeachPosition THEN
	TeachPosition := FALSE;
	//
	stTeachPosition.MoveType := DUT_Command_MoveType.DirectMove;
	stTeachPosition.Speed := 50;
	stTeachPosition.PositionType := DUT_Command_PositionType.AbsolutePosition;
	stTeachPosition.CoordSystem := _00_Main.ProcessJog.CoordSystem;
	stTeachPosition.CoordRef.X := F_RoundLreal(_00_Main.ProcessJog.Position.v[0],2);
	stTeachPosition.CoordRef.Y := F_RoundLreal(_00_Main.ProcessJog.Position.v[1],2);
	stTeachPosition.CoordRef.Z := F_RoundLreal(_00_Main.ProcessJog.Position.v[2],2);
	stTeachPosition.CoordRef.A := F_RoundLreal(_00_Main.ProcessJog.Position.v[3],2);
	stTeachPosition.CoordRef.B := F_RoundLreal(_00_Main.ProcessJog.Position.v[4],2);
	stTeachPosition.CoordRef.C := F_RoundLreal(_00_Main.ProcessJog.Position.v[5],2);
	PersistentVars.Programs[SelectedProgramNumber].Commands[SelectedRecipeRowIndex].CommandType := DUT_CommandType.MoveCommand;
	PersistentVars.Programs[SelectedProgramNumber].Commands[SelectedRecipeRowIndex].CommandString := F_BuildMoveCmd(stTeachPosition);
	PersistentVars.Programs[SelectedProgramNumber].Commands[SelectedRecipeRowIndex].CommandComment := 'Taught position';
END_IF

//
IF PasteCommand THEN
	PasteCommand := FALSE;
	PersistentVars.Programs[SelectedProgramNumber].Commands[SelectedRecipeRowIndex].CommandType := CopiedCommandType;
	PersistentVars.Programs[SelectedProgramNumber].Commands[SelectedRecipeRowIndex].CommandString := CopiedCommandString;
	PersistentVars.Programs[SelectedProgramNumber].Commands[SelectedRecipeRowIndex].CommandComment := CopiedCommandComment;
END_IF

//
IF CopySelectedCommand THEN
	CopySelectedCommand := FALSE;
	
	//
	CopiedCommandType := PersistentVars.Programs[SelectedProgramNumber].Commands[SelectedRecipeRowIndex].CommandType;
	CopiedCommandString := PersistentVars.Programs[SelectedProgramNumber].Commands[SelectedRecipeRowIndex].CommandString;
	CopiedCommandComment := PersistentVars.Programs[SelectedProgramNumber].Commands[SelectedRecipeRowIndex].CommandComment;
	
	CASE PersistentVars.Programs[SelectedProgramNumber].Commands[SelectedRecipeRowIndex].CommandType OF
		DUT_CommandType.Disabled:
			//
			
		DUT_CommandType.MoveCommand:
			MoveCmdParser(Command := PersistentVars.Programs[SelectedProgramNumber].Commands[SelectedRecipeRowIndex].CommandString);
			stMoveCmd := MoveCmdParser.stCommand;
			
		DUT_CommandType.PauseCommand:
			PauseCmdParser(Command:=PersistentVars.Programs[SelectedProgramNumber].Commands[SelectedRecipeRowIndex].CommandString);
			stPauseCmd := PauseCmdParser.stCommand;
			
		DUT_CommandType.InputCommand:
			//
			
		DUT_CommandType.OutputCommand:
			OutputCmdParser(Command:=PersistentVars.Programs[SelectedProgramNumber].Commands[SelectedRecipeRowIndex].CommandString);
			stOutputCmd := OutputCmdParser.stCommand;
			
		DUT_CommandType.VisionCommand:
			VisionCmdParser(Command:=PersistentVars.Programs[SelectedProgramNumber].Commands[SelectedRecipeRowIndex].CommandString);
			stVisionCmd := VisionCmdParser.stCommand;
			
		DUT_CommandType.VarSetCommand:
			VarSetCmdParser(Command:=PersistentVars.Programs[SelectedProgramNumber].Commands[SelectedRecipeRowIndex].CommandString);
			stVarSetCmd := VarSetCmdParser.stCommand;
			
		DUT_CommandType.CncCommand:
			CncCmdParser(Command:=PersistentVars.Programs[SelectedProgramNumber].Commands[SelectedRecipeRowIndex].CommandString);
			stCncCmd := CncCmdParser.stCommand;
			
		DUT_CommandType.LogicCommand:
			LogicCmdParser(Command:=PersistentVars.Programs[SelectedProgramNumber].Commands[SelectedRecipeRowIndex].CommandString);
			stLogicCmd := LogicCmdParser.stCommand;
			
	END_CASE
	
END_IF

// should be called after CopySelectedCommand case statement
CurrentVisu := VisuElems.CurrentVisu;
OpenDialog(
	xExecute:= , 
	xDone=> , 
	xBusy=> , 
	xError=> , 
	itfClientFilter:= , 
	eError=> , 
	sDialogName:= , 
	xModal:= TRUE, // TRUE - visu in background cannot be used
	pTopLeftPosition:= , 
	itfDialogOpenedListener:= , 
	pbyDialogInterfaceData:= , 
	udiDialogInterfaceDataSize:= , 
	itfDialogCloseListener:= , 
	pbyTagForDialogClosed:= , 
	dwDialogFlags:= );
IF (OpenDialog.xExecute AND OpenDialog.xDone) 
	OR OpenDialog.xError 
	THEN
		OpenDialog.xExecute := FALSE;
END_IF

// alarms
IF GVL_App.xClearHistory THEN
    FOR iAlarmCfg := 0 TO AlarmManager.AlarmGlobals.g_AlarmHandler.AlarmConfigurationCount - 1 DO
        itfAlarmConfiguration := AlarmManager.AlarmGlobals.g_AlarmHandler.GetAlarmConfiguration(iAlarmCfg);
        IF __QUERYINTERFACE(itfAlarmConfiguration, itfAlarmConfiguration7) THEN
            itfAlarmConfiguration7.ClearHistory(xKeepActiveAlarms := TRUE); // xKeepActiveAlarms := TRUE so a live fault isn’t ripped out of the recorder.
        END_IF
    END_FOR
    GVL_App.xClearHistory := FALSE;
END_IF
END_PROGRAM
