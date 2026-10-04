FUNCTION_BLOCK FB_VarSetCmdProcessor EXTENDS FB_Module
VAR_INPUT
	Command: STRING(GVL_App.COMMAND_LEN):='';
END_VAR
VAR_OUTPUT
END_VAR
VAR
END_VAR
VAR CONSTANT
	_STEP_DONE: 		INT := 1000;
	_STEP_IDLE: 		INT := 0000;
	_STEP_COMMAND: 		INT := 0010;
	_STEP_ERR: 			INT := 0020;
END_VAR
VAR
	_UnitModeCurrentState: PACK_ML.State;
	_VarSetCmdParser: FB_ParseVarSetCmd;
END_VAR

SUPER^();

// PackML state machine implementation
_UnitModeCurrentState := SUPER^._UnitMode.CurrentState;

//
CASE SUPER^._SeqMain OF
    _STEP_DONE:
		SUPER^._SeqMain := _STEP_IDLE;
        
    _STEP_IDLE:
		// hang out here until called to run
		
	_STEP_COMMAND:
        _VarSetCmdParser(Command:=Command);
		IF _VarSetCmdParser.Error THEN
			//
			SUPER^.M_Error(Description:=_VarSetCmdParser.ErrorDescription);
			// next, please
			SUPER^._SeqMain := _STEP_ERR;
			
		ELSE
			//
			IF _VarSetCmdParser.stCommand.ArrayType = DUT_Command_ArrayType.StoredPosition THEN
				SUPER^.StoredPositions[_VarSetCmdParser.stCommand.ArrayIndex].Position.X := 
												F_VarSetCmdOperate(SUPER^.StoredPositions[_VarSetCmdParser.stCommand.ArrayIndex].Position.X, 
																	_VarSetCmdParser.stCommand.XOperType, 
																	_VarSetCmdParser.stCommand.Coordinates.X);
																	
				SUPER^.StoredPositions[_VarSetCmdParser.stCommand.ArrayIndex].Position.Y := 
												F_VarSetCmdOperate(SUPER^.StoredPositions[_VarSetCmdParser.stCommand.ArrayIndex].Position.Y, 
																	_VarSetCmdParser.stCommand.YOperType, 
																	_VarSetCmdParser.stCommand.Coordinates.Y);
																	
				SUPER^.StoredPositions[_VarSetCmdParser.stCommand.ArrayIndex].Position.Z := 
												F_VarSetCmdOperate(SUPER^.StoredPositions[_VarSetCmdParser.stCommand.ArrayIndex].Position.Z, 
																	_VarSetCmdParser.stCommand.ZOperType, 
																	_VarSetCmdParser.stCommand.Coordinates.Z);
																	
				SUPER^.StoredPositions[_VarSetCmdParser.stCommand.ArrayIndex].Position.A := 
												F_VarSetCmdOperate(SUPER^.StoredPositions[_VarSetCmdParser.stCommand.ArrayIndex].Position.A, 
																	_VarSetCmdParser.stCommand.AOperType, 
																	_VarSetCmdParser.stCommand.Coordinates.A);
																	
				SUPER^.StoredPositions[_VarSetCmdParser.stCommand.ArrayIndex].Position.B := 
												F_VarSetCmdOperate(SUPER^.StoredPositions[_VarSetCmdParser.stCommand.ArrayIndex].Position.B, 
																	_VarSetCmdParser.stCommand.BOperType, 
																	_VarSetCmdParser.stCommand.Coordinates.B);
																	
				SUPER^.StoredPositions[_VarSetCmdParser.stCommand.ArrayIndex].Position.C := 
												F_VarSetCmdOperate(SUPER^.StoredPositions[_VarSetCmdParser.stCommand.ArrayIndex].Position.C, 
																	_VarSetCmdParser.stCommand.COperType, 
																	_VarSetCmdParser.stCommand.Coordinates.C);
				
				
			ELSIF _VarSetCmdParser.stCommand.ArrayType = DUT_Command_ArrayType.VisionRegister THEN
				SUPER^.VisionRegisters[_VarSetCmdParser.stCommand.ArrayIndex].Position.X := 
												F_VarSetCmdOperate(SUPER^.VisionRegisters[_VarSetCmdParser.stCommand.ArrayIndex].Position.X, 
																	_VarSetCmdParser.stCommand.XOperType, 
																	_VarSetCmdParser.stCommand.Coordinates.X);
																	
				SUPER^.VisionRegisters[_VarSetCmdParser.stCommand.ArrayIndex].Position.Y := 
												F_VarSetCmdOperate(SUPER^.VisionRegisters[_VarSetCmdParser.stCommand.ArrayIndex].Position.Y, 
																	_VarSetCmdParser.stCommand.YOperType, 
																	_VarSetCmdParser.stCommand.Coordinates.Y);
																	
				SUPER^.VisionRegisters[_VarSetCmdParser.stCommand.ArrayIndex].Position.Z := 
												F_VarSetCmdOperate(SUPER^.VisionRegisters[_VarSetCmdParser.stCommand.ArrayIndex].Position.Z, 
																	_VarSetCmdParser.stCommand.ZOperType, 
																	_VarSetCmdParser.stCommand.Coordinates.Z);
																	
				SUPER^.VisionRegisters[_VarSetCmdParser.stCommand.ArrayIndex].Position.A := 
												F_VarSetCmdOperate(SUPER^.VisionRegisters[_VarSetCmdParser.stCommand.ArrayIndex].Position.A, 
																	_VarSetCmdParser.stCommand.AOperType, 
																	_VarSetCmdParser.stCommand.Coordinates.A);
																	
				SUPER^.VisionRegisters[_VarSetCmdParser.stCommand.ArrayIndex].Position.B := 
												F_VarSetCmdOperate(SUPER^.VisionRegisters[_VarSetCmdParser.stCommand.ArrayIndex].Position.B, 
																	_VarSetCmdParser.stCommand.BOperType, 
																	_VarSetCmdParser.stCommand.Coordinates.B);
																	
				SUPER^.VisionRegisters[_VarSetCmdParser.stCommand.ArrayIndex].Position.C := 
												F_VarSetCmdOperate(SUPER^.VisionRegisters[_VarSetCmdParser.stCommand.ArrayIndex].Position.C, 
																	_VarSetCmdParser.stCommand.COperType, 
																	_VarSetCmdParser.stCommand.Coordinates.C);
				
			ELSIF _VarSetCmdParser.stCommand.ArrayType = DUT_Command_ArrayType.Flag THEN
				// will just let the flag take on the value of X
				SUPER^.Flags[_VarSetCmdParser.stCommand.ArrayIndex].Value := LREAL_TO_BOOL(_VarSetCmdParser.stCommand.Coordinates.X);
				
			END_IF
			// next, please
			SUPER^._SeqMain := _STEP_DONE;
			
		END_IF

END_CASE
END_FUNCTION_BLOCK
