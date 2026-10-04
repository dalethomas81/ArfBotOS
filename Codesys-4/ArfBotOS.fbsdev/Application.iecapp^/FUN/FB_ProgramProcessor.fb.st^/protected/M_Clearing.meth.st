METHOD PROTECTED M_Clearing : BOOL

SUPER^.M_StateComplete();

(*IF SUPER^.Flags[GVL_App.HOME_COMPLETE].Value THEN
	CASE _SeqClearing OF
		0:
			_ProcessMoveCommand.M_Home();
			_SeqClearing := _SeqClearing + 1;
		1:
			IF _ProcessMoveCommand.PropCurrentState = PACK_ML.State.Idle THEN
				_SeqClearing := _SeqClearing + 1;
			END_IF
		2:
			IF NOT _ProcessMoveCommand.Error THEN
				_ProcessMoveCommand.Command := GVL_App.Pose2;
				_ProcessMoveCommand.M_Start();
				_SeqClearing := _SeqClearing + 1;
			ELSE
				SUPER^._M_Error(Description:='Move processor errored. Please reset and try again');
				_SeqClearing := -1;
			END_IF
		3:
			IF _ProcessMoveCommand.PropCurrentState = PACK_ML.State.Execute THEN
				_SeqClearing := _SeqClearing + 1;
			END_IF
		4:
			IF _ProcessMoveCommand.PropCurrentState = PACK_ML.State.Complete 
				OR _ProcessMoveCommand.PropCurrentState = PACK_ML.State.Stopped 
				OR _ProcessMoveCommand.PropCurrentState = PACK_ML.State.Aborted
				OR _ProcessMoveCommand.PropCurrentState = PACK_ML.State.Idle  THEN
					SUPER^.M_StateComplete();
					_SeqClearing := 0;
			END_IF
	END_CASE
ELSE
	SUPER^._M_Error(Description:='Drives are not homed. Please home and try again');
	_SeqClearing := -1;
END_IF

IF _ProcessMoveCommand.Error THEN
	SUPER^._M_Error(Description:='Move processor in error state. Please reset and try again');
	_SeqClearing := -1;
END_IF
*)

(*
IF ItfActiveModule <> 0 THEN
	//
	CASE _Seq OF
		0:
			IF NOT ItfActiveModule.PropCurrentState = PACK_ML.State.Stopped THEN
				ItfActiveModule.M_Abort();
				_Seq := 1;
			ELSE
				SUPER^.M_StateComplete();
			END_IF
		1:
			CASE ItfActiveModule.PropCurrentState OF
				PACK_ML.State.Aborted:
					ItfActiveModule.Clear();
				PACK_ML.State.Stopped:
					_Seq := 0;
			END_CASE
			
	END_CASE
ELSE
	_Seq := 0;
	SUPER^.M_StateComplete();
END_IF
*)
END_METHOD
