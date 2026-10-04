FUNCTION_BLOCK FB_PositionTracker
VAR_IN_OUT
	AxisGroup: AXIS_GROUP_REF_SM3;
	J1: AXIS_REF_SM3;
	J2: AXIS_REF_SM3;
	J3: AXIS_REF_SM3;
	J4: AXIS_REF_SM3;
	J5: AXIS_REF_SM3;
	J6: AXIS_REF_SM3;
END_VAR
VAR_INPUT
    OriConvention: SMC_ORI_CONVENTION := SMC_ORI_CONVENTION.ZYZ;
END_VAR
VAR_OUTPUT
	VisuStruct3DTrack: VisuStruct3DTrack;
    Position: SMC_PosInfo;
	Arm: SMC_ArticulatedRobot_6DOF_J0_State;
	Elbow: SMC_ArticulatedRobot_6DOF_J1_State;
	Hand: SMC_ArticulatedRobot_6DOF_J4_State;
END_VAR
VAR
	_SMC_TrafoConfig_ArticulatedRobot_6DOF: SMC_TrafoConfig_ArticulatedRobot_6DOF;
	_SMC_Trafo_ArticulatedRobot_6DOF: SMC_Trafo_ArticulatedRobot_6DOF;
	_SMC_Trafof_ArticulatedRobot_6DOF: SMC_Trafof_ArticulatedRobot_6DOF;
	_VisuStruct3DPathPoint: ARRAY [0..5000] OF VisuStruct3DPathPoint;
	_SMC_PositionTracker: SMC_PositionTracker;
	_PositionTrackerClearR_Trig: R_Trig;
	_Clear: BOOL:=FALSE;
END_VAR

// NOTE currently the settings for J6 invert direction under scaling/mapping contradict using motion control 
// libraries and CNC. For CNC to work properly, the setting is required to be enabled (inverted). Additionally
// the homing direction and position will need to be negated. This all requires a full download. Will be leaving 
// setting disabled for now so that we can continue to use motion control. 
//
// to recap, take these steps if you would like to try out CNC:
// 1. open the axis pool settings and under the scaling/mapping section enable 'invert direction'
// 2. in the A_Homing POU, change the homing direction to positive.
// 3. in _00_MAIN POU, under FirstScan set the initial home offset to a positive value (keep the value and just change the sign).
// 4. do a full download.
// 5. go online and set the persistent value (if previously initialized in FirstScan) to a positive value.
//
// UPDATE: Figured out a work around for this issue. In the A_Interpolater POU, I negated the set position in the ControlAxisByPos
// function block for J6 such that it drives the opposite way. This appears to have resolved the issue. Why this is needed, I do not
// know. I am sure it is some minor overlooked detail somewhere in the code...


_SMC_TrafoConfig_ArticulatedRobot_6DOF.a1:=ArfBotAxisGroup.Trafo.a1;//AxisGroup.Trafo.a1;
_SMC_TrafoConfig_ArticulatedRobot_6DOF.a2:=ArfBotAxisGroup.Trafo.a2;//AxisGroup.Trafo.a2;
_SMC_TrafoConfig_ArticulatedRobot_6DOF.a3:=ArfBotAxisGroup.Trafo.a3;//AxisGroup.Trafo.a3;
// conflicting documentation on if d1 needs to be pos or neg. only works as negative otherwise trafo reports invalid configuration
// https://product-help.schneider-electric.com/Machine%20Expert/V2.0/zh/codesys_softmotion/codesys_softmotion/modules/_sm_trafo_pou_six_dof_robot.html
// https://forge.codesys.com/forge/talk/Motion/thread/16e1f0327f/
_SMC_TrafoConfig_ArticulatedRobot_6DOF.d1:=-ArfBotAxisGroup.Trafo.d1;//-AxisGroup.Trafo.d1;
_SMC_TrafoConfig_ArticulatedRobot_6DOF.d3:=ArfBotAxisGroup.Trafo.d3;//AxisGroup.Trafo.d3;
_SMC_TrafoConfig_ArticulatedRobot_6DOF.d4:=ArfBotAxisGroup.Trafo.d4;//AxisGroup.Trafo.d4;
_SMC_TrafoConfig_ArticulatedRobot_6DOF.d6:=ArfBotAxisGroup.Trafo.d6;//AxisGroup.Trafo.d6;
_SMC_TrafoConfig_ArticulatedRobot_6DOF.q3_max_deg:=160; // TODO put magic numbers somewhere appropriate
_SMC_TrafoConfig_ArticulatedRobot_6DOF.q3_min_deg:=-160; // TODO put magic numbers somewhere appropriate
_SMC_TrafoConfig_ArticulatedRobot_6DOF.q5_max_deg:=160; // TODO put magic numbers somewhere appropriate
_SMC_TrafoConfig_ArticulatedRobot_6DOF.q5_min_deg:=-160; // TODO put magic numbers somewhere appropriate

// track position
_SMC_Trafof_ArticulatedRobot_6DOF(config:=_SMC_TrafoConfig_ArticulatedRobot_6DOF,
							Drive0:=J1,
							Drive1:=J2,
							Drive2:=J3,
							Drive3:=J4,
							Drive4:=J5,
							Drive5:=J6,
							vToolPos=>,
							mToolOrientation=>,
							armState=>Arm,
							elbowState=>Elbow,
							handState=>Hand);
Position.dX := _SMC_Trafof_ArticulatedRobot_6DOF.vToolPos.dX;
Position.dY := _SMC_Trafof_ArticulatedRobot_6DOF.vToolPos.dY;
Position.dZ := _SMC_Trafof_ArticulatedRobot_6DOF.vToolPos.dZ;

// https://content.helpme-codesys.com/en/libs/SM3_Math/4.12.0.0/Matrix3/fld-Matrix3.html
// ATTENTION: ZYZ is the only orientation convention that was reliable during testing.
CASE OriConvention OF
    SMC_ORI_CONVENTION.ADDAXES:
        Position.dA := J4.fActPosition;
        Position.dB := J5.fActPosition;
        Position.dC := J6.fActPosition;
    SMC_ORI_CONVENTION.XYZ:
        SMC_Matrix3_ToRollPitchYaw_deg( mR:=_SMC_Trafof_ArticulatedRobot_6DOF.mToolOrientation, 
                                        dRoll=> Position.dA, dPitch=> Position.dB, dYaw=> Position.dC);
    SMC_ORI_CONVENTION.ZYX:
        // yaw(Z), pitch(Y) and roll(X)
        SMC_Matrix3_ToYawPitchRoll( mR:=_SMC_Trafof_ArticulatedRobot_6DOF.mToolOrientation, 
                                    dYaw_rad=> Position.dA, dPitch_rad=> Position.dB, dRoll_rad=> Position.dC);
                                    Position.dA := F_RadToDeg(Position.dA);
                                    Position.dB := F_RadToDeg(Position.dB);
                                    Position.dC := F_RadToDeg(Position.dC);
    SMC_ORI_CONVENTION.ZYZ:
        SMC_Matrix3_ToZYZ_deg(  mR:=_SMC_Trafof_ArticulatedRobot_6DOF.mToolOrientation, 
                                dA_deg=> Position.dA, dB_deg=> Position.dB, dC_deg=> Position.dC);
END_CASE

_PositionTrackerClearR_Trig(Clk:=_Clear);
_SMC_PositionTracker(
	bEnable:= TRUE, 
	bClear:= _PositionTrackerClearR_Trig.Q, 
	dX:= _SMC_Trafof_ArticulatedRobot_6DOF.vToolPos.dX, 
	dY:= _SMC_Trafof_ArticulatedRobot_6DOF.vToolPos.dY, 
	dZ:= _SMC_Trafof_ArticulatedRobot_6DOF.vToolPos.dZ, 
	udiNumberOfPointsInArray:= SIZEOF(_VisuStruct3DPathPoint) / SIZEOF(VisuStruct3DPathPoint), 
	pBuffer:= ADR(_VisuStruct3DPathPoint), 
	vs3dt=> VisuStruct3DTrack);
_Clear := FALSE;
END_FUNCTION_BLOCK
