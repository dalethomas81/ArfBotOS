METHOD PROTECTED M_Execute : BOOL
VAR
    _ErrorDescription: STRING;
END_VAR

// here we want to execute our commands/jobs/etc and set completed when done.
//
CASE _SeqMain OF
  	_STEP_DONE:
		_SeqMain := _STEP_IDLE;
		
	_STEP_IDLE:
		// hang out here until called to run
		_CurrentCmdIdx := 0;
		SUPER^.M_StateComplete();
		
	_STEP_CMD: //
		IF NOT Enable THEN
			_SeqMain := _STEP_DONE;
            _ErrorDescription := '';
            IF NOT GVL_App.Enable THEN
                _ErrorDescription := 'Robot not enabled';
            END_IF
            IF NOT PersistentVars.Flags[GVL_App.HOME_COMPLETE].Value THEN
                _ErrorDescription := CONCAT(_ErrorDescription, ' | Robot not homed');
            END_IF
            IF GVL_App.xActiveAlarm THEN
                _ErrorDescription := CONCAT(_ErrorDescription, ' | Active alarms exist');
            END_IF
            IF _ErrorDescription = '' THEN
                _ErrorDescription := 'Not enabled';
            END_IF
            _M_Error(Description := _ErrorDescription);
            
		ELSIF _CurrentCmdIdx <= GVL_App.PROG_CMDS_ARR_END AND _CurrentCmdIdx >= GVL_App.PROG_CMDS_ARR_BEG THEN
			//
			IF _Commands[_CurrentCmdIdx].CommandType = DUT_CommandType.Disabled THEN
				// want to increment here so that we can disable some commands 
				// in the program and it continue to execute
				_CurrentCmdIdx := _CurrentCmdIdx + 1;
				_SeqMain := _STEP_CMD;
				
			ELSE
				CASE _Commands[_CurrentCmdIdx].CommandType OF
					DUT_CommandType.MoveCommand: //
						StatusText := 'Move Command in Progress';
						ItfActiveProcessorUnitMode := _ProcessMoveCommand.ItfUnitMode;
						ItfActiveModule := _ProcessMoveCommand;
						_ProcessMoveCommand.Command := _Commands[_CurrentCmdIdx].CommandString;
						ItfActiveModule.M_Start();
					
					DUT_CommandType.PauseCommand: //
						StatusText := 'Pause Command in Progress';
						ItfActiveProcessorUnitMode := _ProcessPauseCommand.ItfUnitMode;
						ItfActiveModule := _ProcessPauseCommand;
						_ProcessPauseCommand.Command := _Commands[_CurrentCmdIdx].CommandString;
						ItfActiveModule.M_Start();
					
					DUT_CommandType.InputCommand: //
						StatusText := 'Input Command in Progress';
						//ActiveUnitMode := _ProcessInputCommand.ItfUnitMode;
						//ItfActiveModule := _ProcessInputCommand;
						//_ProcessInputCommand.Command := _Commands[_CurrentCmdIdx].CommandString;
						//ItfActiveModule.M_Start();
					
					DUT_CommandType.OutputCommand: //
						StatusText := 'Output Command in Progress';
						ItfActiveProcessorUnitMode := _ProcessOutputCommand.ItfUnitMode;
						ItfActiveModule := _ProcessOutputCommand;
						_ProcessOutputCommand.Command := _Commands[_CurrentCmdIdx].CommandString;
						ItfActiveModule.M_Start();
					
					DUT_CommandType.VisionCommand: //
						StatusText := 'Vision Command in Progress';
						ItfActiveProcessorUnitMode := _ProcessVisionCommand.ItfUnitMode;
						ItfActiveModule := _ProcessVisionCommand;
						_ProcessVisionCommand.Command := _Commands[_CurrentCmdIdx].CommandString;
						ItfActiveModule.M_Start();
					
					DUT_CommandType.VarSetCommand: //
						StatusText := 'VarSet Command in Progress';
						ItfActiveProcessorUnitMode := _ProcessVarsetCommand.ItfUnitMode;
						ItfActiveModule := _ProcessVarsetCommand;
						_ProcessVarsetCommand.Command := _Commands[_CurrentCmdIdx].CommandString;
						ItfActiveModule.M_Start();
					
					DUT_CommandType.CncCommand: //
						StatusText := 'CNC Command in Progress';
						ItfActiveProcessorUnitMode := _ProcessCncCommand.ItfUnitMode;
						ItfActiveModule := _ProcessCncCommand;
						_ProcessCncCommand.Command := _Commands[_CurrentCmdIdx].CommandString;
						ItfActiveModule.M_Start();
					
					DUT_CommandType.LogicCommand: //
						StatusText := 'Logic Command in Progress';
						ItfActiveProcessorUnitMode := _ProcessLogicCommand.ItfUnitMode;
						ItfActiveModule := _ProcessLogicCommand;
						_ProcessLogicCommand.Command := _Commands[_CurrentCmdIdx].CommandString;
						ItfActiveModule.M_Start();
						
				ELSE
					_SeqMain := _STEP_CMD;
					
				END_CASE
				
				// wait for executing
				IF ItfActiveModule.PropCurrentState = PACK_ML.State.Execute THEN
					_ActiveCommand := _Commands[_CurrentCmdIdx];
					_CurrentCmdIdx := _CurrentCmdIdx + 1;
					_SeqMain := _STEP_WAIT;
				END_IF
			END_IF
				
		ELSE
			_SeqMain := _STEP_DONE;
		END_IF
		
	_STEP_WAIT: // wait for command to finish
		//
		IF _ProcessorDoneRtrig.Q THEN
			_SeqMain := _STEP_CMD;
		END_IF
		//
		IF ItfActiveModule.PropError THEN
			SUPER^._UnitMode.Suspend();
			_SeqMain := _STEP_ERR;
		END_IF
		
	_STEP_ERR: //
	
END_CASE

// watch for rising edge of idle state. this was done for all processors since 
// the vision processor runs in a separate task and isnt guaranteed to start in the same 
// PLC scan.
_ProcessorDoneRtrig(clk:=ItfActiveProcessorUnitMode.CurrentState = PACK_ML.State.Idle);
END_METHOD
