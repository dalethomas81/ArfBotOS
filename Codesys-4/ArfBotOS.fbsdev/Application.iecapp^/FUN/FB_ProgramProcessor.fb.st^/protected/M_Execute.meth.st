METHOD PROTECTED M_Execute : BOOL
VAR
    _ErrorDescription: STRING;
END_VAR

{warning disable C0371}
// here we want to execute our commands/jobs/etc and set completed when done.
//
CASE _SeqMain OF
  	_STEP_DONE:
		_SeqMain := _STEP_IDLE;
		
	_STEP_IDLE:
		// hang out here until called to run
		_CurrentPrgIdx := 0;
		SUPER^.M_StateComplete();
		
	_STEP_PRG: //
		IF NOT Enable THEN
			_SeqMain := _STEP_DONE;
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
            
		ELSIF _CurrentPrgIdx <= GVL_App.PROG_ARR_END AND _CurrentPrgIdx >= GVL_App.PROG_ARR_BEG THEN
			//
			StatusText := '';
			ItfActiveModule := _ProcessCommands;
			ItfActiveProcessorUnitMode := _ProcessCommands.ItfUnitMode;
			
			IF _ProcessCommands.M_SetCommands(StartingCommand:=0, Commands:=Programs[_CurrentPrgIdx].Commands) THEN
				_ProcessCommands.M_Start();
				_SeqMain := _STEP_WAIT_EXECUTE;
			ELSE
				SUPER^.M_Error('Failed to set Command Processor. Check commands.');
				_SeqMain := _STEP_ERR;
			END_IF
			
				
		ELSE
			_SeqMain := _STEP_DONE;
		END_IF
		
	_STEP_WAIT_EXECUTE:
		IF _ProcessCommands.PropCurrentState = PACK_ML.State.Execute THEN
			_SeqMain := _STEP_WAIT_IDLE;
		END_IF
		
	_STEP_WAIT_IDLE: // wait for program to finish
		//
		IF _ProcessCommands.PropCurrentState = PACK_ML.State.Idle THEN
			IF _Returning[_CurrentPrgIdx] THEN
				_Returning[_CurrentPrgIdx] := FALSE;
				IF _ReturnPrograms[_CurrentPrgIdx] <= GVL_App.PROG_ARR_END AND_THEN _ReturnPrograms[_CurrentPrgIdx] >= GVL_App.PROG_ARR_BEG
					AND_THEN _ReturnCommands[_CurrentPrgIdx] <= GVL_App.PROG_CMDS_ARR_END + 1 AND_THEN _ReturnCommands[_CurrentPrgIdx] >= GVL_App.PROG_CMDS_ARR_BEG // +1 because we want to be able to tell if calling program was on last command
					THEN
						//
						IF _ProcessCommands.M_SetCommands(StartingCommand:=_ReturnCommands[_CurrentPrgIdx], Commands:=Programs[_ReturnPrograms[_CurrentPrgIdx]].Commands) THEN
							_CurrentPrgIdx := _ReturnPrograms[_CurrentPrgIdx];
							_ProcessCommands.M_Start();
							_SeqMain := _STEP_WAIT_EXECUTE;
						ELSE
							SUPER^.M_Error('Failed to set Command Processor. Check commands.');
							_SeqMain := _STEP_ERR;
						END_IF
				ELSE
					_SeqMain := _STEP_DONE;
				END_IF
			ELSE
				_SeqMain := _STEP_DONE;
			END_IF
		END_IF
		//
		IF _ProcessCommands.PropError THEN
			SUPER^._UnitMode.Suspend();
			_SeqMain := _STEP_ERR;
		END_IF
		
	_STEP_ERR: //
	
END_CASE

// watch for rising edge of idle state. this was done for all processors since 
// the vision processor runs in a separate task and isnt guaranteed to start in the same 
// PLC scan.
//_ProcessorDoneRtrig(clk:=ItfActiveProcessorUnitMode.CurrentState = PACK_ML.State.Idle);
{warning restore C0371}
END_METHOD
