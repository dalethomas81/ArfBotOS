METHOD PUBLIC M_SetHome : BOOL
VAR_INPUT
END_VAR

{warning disable C0371}
// adjust all the persistent variables to take on the calibrated values
StoredPositions[GVL_App.HOM_POS_IDX].Position.X := F_HomeOffsetCalc(	CurrentOffset:= PersistentVars.StoredPositions[GVL_App.HOM_POS_IDX].Position.X, 
																				CurrentPosition:= J1.fActPosition, 
																				HomeSearchDirection:= _SMC_Homing_J1.nDirection);

StoredPositions[GVL_App.HOM_POS_IDX].Position.Y := F_HomeOffsetCalc(	CurrentOffset:= PersistentVars.StoredPositions[GVL_App.HOM_POS_IDX].Position.Y, 
																				CurrentPosition:= J2.fActPosition, 
																				HomeSearchDirection:= _SMC_Homing_J2.nDirection);

StoredPositions[GVL_App.HOM_POS_IDX].Position.Z := F_HomeOffsetCalc(	CurrentOffset:= PersistentVars.StoredPositions[GVL_App.HOM_POS_IDX].Position.Z, 
																				CurrentPosition:= J3.fActPosition, 
																				HomeSearchDirection:= _SMC_Homing_J3.nDirection);

StoredPositions[GVL_App.HOM_POS_IDX].Position.A := F_HomeOffsetCalc(	CurrentOffset:= PersistentVars.StoredPositions[GVL_App.HOM_POS_IDX].Position.A, 
																				CurrentPosition:= J4.fActPosition, 
																				HomeSearchDirection:= _SMC_Homing_J4.nDirection);

StoredPositions[GVL_App.HOM_POS_IDX].Position.B := F_HomeOffsetCalc(	CurrentOffset:= PersistentVars.StoredPositions[GVL_App.HOM_POS_IDX].Position.B, 
																				CurrentPosition:= J5.fActPosition, 
																				HomeSearchDirection:= _SMC_Homing_J5.nDirection);

StoredPositions[GVL_App.HOM_POS_IDX].Position.C := F_HomeOffsetCalc(	CurrentOffset:= PersistentVars.StoredPositions[GVL_App.HOM_POS_IDX].Position.C, 
																				CurrentPosition:= J6.fActPosition, 
																				HomeSearchDirection:= _SMC_Homing_J6.nDirection);

// this isnt totally needed but will calibrate the current session without the need to rehome
_MC_SetPosition_J1.Execute:=TRUE;
_MC_SetPosition_J2.Execute:=TRUE;
_MC_SetPosition_J3.Execute:=TRUE;
_MC_SetPosition_J4.Execute:=TRUE;
_MC_SetPosition_J5.Execute:=TRUE;
_MC_SetPosition_J6.Execute:=TRUE;
{warning restore C0371}
END_METHOD
