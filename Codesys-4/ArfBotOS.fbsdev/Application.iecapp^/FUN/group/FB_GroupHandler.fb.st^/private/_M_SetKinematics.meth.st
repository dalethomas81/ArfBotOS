METHOD PRIVATE _M_SetKinematics : BOOL
VAR_INPUT
END_VAR

CASE _SequenceSetKinematics OF
	0: // IDLE
		IF _SetKinematics THEN
			_SetKinematics := FALSE;
			_KinematicsSet := FALSE;
			_MC_GroupReadActualPosition.Enable := FALSE;
			_SMC_SetKinConfiguration.Execute := FALSE;
			_SequenceSetKinematics := _SequenceSetKinematics + 1;
		END_IF
		
	1: // read group position
		_MC_GroupReadActualPosition.Enable := TRUE;
		_SequenceSetKinematics := _SequenceSetKinematics + 1;
		
	2: // wait for group read position
		// TODO add timeout timer here
		IF _MC_GroupReadActualPosition.Error THEN
			Error := TRUE;
			ErrorID := _MC_GroupReadActualPosition.ErrorID;
			_SequenceSetKinematics := 0;
		ELSIF _MC_GroupReadActualPosition.Valid THEN
			_SequenceSetKinematics := _SequenceSetKinematics + 1;
		END_IF
		
	3: // set kinematics
        // orientation convention is always ZYZ for softmotion
        GVL_App.OriConvention := SMC_ORI_CONVENTION.ZYZ;
        //
		_SMC_SetKinConfiguration.Execute := TRUE;
		_SequenceSetKinematics := _SequenceSetKinematics + 1;
		
	4: // wait for set kinematics
		// TODO add timeout timer here
		IF _SMC_SetKinConfiguration.Error THEN
			Error := TRUE;
			ErrorID := _SMC_SetKinConfiguration.ErrorID;
			_SequenceSetKinematics := 0;
		ELSIF _SMC_SetKinConfiguration.Done THEN
			_SMC_SetKinConfiguration.Execute := FALSE;
			_SequenceSetKinematics := _SequenceSetKinematics + 1;
		END_IF
		
	5:
		_SetCoordinates := TRUE;
		_SequenceSetKinematics := _SequenceSetKinematics + 1;
	
	6:
		// TODO add timeout timer here
		IF _MC_SetCoordinateTransform.Error THEN
			Error := TRUE;
			ErrorID := _MC_SetCoordinateTransform.ErrorID;
			_SequenceSetCoordTrans := 0;
			_SequenceSetKinematics := 0;
		ELSIF _SMC_GroupSetTool.Error THEN
			Error := TRUE;
			ErrorID := _SMC_GroupSetTool.ErrorID;
			_SequenceSetCoordTrans := 0;
			_SequenceSetKinematics := 0;
		ELSIF _SequenceSetCoordTrans <= 0 THEN
			_KinematicsSet := TRUE;
			_SequenceSetKinematics := 0;
		END_IF
		
END_CASE

CASE _SequenceSetCoordTrans OF
	0: //
		IF _SetCoordinates THEN
			_SetCoordinates := FALSE;
			_SMC_GroupSetTool.Execute := FALSE;
			_MC_SetCoordinateTransform.Execute := FALSE;
			_SequenceSetCoordTrans := _SequenceSetCoordTrans + 1;
		END_IF
	
	1: //
		_MC_COORD_REF := StoredPositions[GVL_App.MCS_POS_IDX].Position; // MCS
		_MC_SetCoordinateTransform.CoordSystem := SMC_COORD_SYSTEM.MCS;
		_MC_SetCoordinateTransform.Execute := TRUE;
		_SequenceSetCoordTrans := _SequenceSetCoordTrans + 1;
		
	2: //
		// TODO add timeout timer here
		IF _MC_SetCoordinateTransform.Error THEN
			(*SUPER^.M_Error(Description:=CONCAT('Move Command Error: ',
										CONCAT(TO_STRING(_MC_SetCoordinateTransform.ErrorID),
										CONCAT(' ', SMC_ErrorString(_MC_SetCoordinateTransform.ErrorID,
													SMC_Language_Type.english)))));*)
			_SequenceSetCoordTrans := 10; // error step so we dont spam errors
			
		ELSIF _MC_SetCoordinateTransform.Done THEN
			_MC_SetCoordinateTransform.Execute := FALSE;
			_SequenceSetCoordTrans := _SequenceSetCoordTrans + 1;
		END_IF
		
	3: //
		_MC_COORD_REF := StoredPositions[GVL_App.PC1_POS_IDX].Position; // PCS 1
		_MC_SetCoordinateTransform.CoordSystem := SMC_COORD_SYSTEM.PCS_1;
		_MC_SetCoordinateTransform.Execute := TRUE;
		_SequenceSetCoordTrans := _SequenceSetCoordTrans + 1;
		
	4: //
		// TODO add timeout timer here
		IF _MC_SetCoordinateTransform.Error THEN
			(*SUPER^.M_Error(Description:=CONCAT('Move Command Error: ',
										CONCAT(TO_STRING(_MC_SetCoordinateTransform.ErrorID),
										CONCAT(' ', SMC_ErrorString(_MC_SetCoordinateTransform.ErrorID,
													SMC_Language_Type.english)))));*)
			_SequenceSetCoordTrans := 10; // error step so we dont spam errors
			
		ELSIF _MC_SetCoordinateTransform.Done THEN
			_MC_SetCoordinateTransform.Execute := FALSE;
			_SequenceSetCoordTrans := _SequenceSetCoordTrans + 1;
		END_IF
		
	5: //
		_MC_COORD_REF := StoredPositions[GVL_App.PC2_POS_IDX].Position; // PCS 2
		_MC_SetCoordinateTransform.CoordSystem := SMC_COORD_SYSTEM.PCS_2;
		_MC_SetCoordinateTransform.Execute := TRUE;
		_SequenceSetCoordTrans := _SequenceSetCoordTrans + 1;
		
	6: //
		// TODO add timeout timer here
		IF _MC_SetCoordinateTransform.Error THEN
			(*SUPER^.M_Error(Description:=CONCAT('Move Command Error: ',
										CONCAT(TO_STRING(_MC_SetCoordinateTransform.ErrorID),
										CONCAT(' ', SMC_ErrorString(_MC_SetCoordinateTransform.ErrorID,
													SMC_Language_Type.english)))));*)
			_SequenceSetCoordTrans := 10; // error step so we dont spam errors
			
		ELSIF _MC_SetCoordinateTransform.Done THEN
			_MC_SetCoordinateTransform.Execute := FALSE;
			_SequenceSetCoordTrans := _SequenceSetCoordTrans + 1;
		END_IF
		
	7:  // set tool offset
		_MC_COORD_REF := StoredPositions[GVL_App.TCP_POS_IDX].Position; // TCP
		_SMC_GroupSetTool.Execute := TRUE;
		_SequenceSetCoordTrans := _SequenceSetCoordTrans + 1;
		
	8: // monitor tool offset
		// TODO add timeout timer here
		IF _SMC_GroupSetTool.Error THEN
			(*SUPER^.M_Error(Description:=CONCAT('Move Command Error: ',
										CONCAT(TO_STRING(_SMC_GroupSetTool.ErrorID),
										CONCAT(' ', SMC_ErrorString(_SMC_GroupSetTool.ErrorID,
													SMC_Language_Type.english)))));*)
			_SequenceSetCoordTrans := 999; // error step so we dont spam errors
			
		ELSIF _SMC_GroupSetTool.Done THEN
			_SMC_GroupSetTool.Execute := FALSE;
			_SequenceSetCoordTrans := 0;
		END_IF

END_CASE
END_METHOD
