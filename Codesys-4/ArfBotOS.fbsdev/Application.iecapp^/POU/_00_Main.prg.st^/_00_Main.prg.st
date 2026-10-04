PROGRAM _00_Main
VAR
	//
    TaskInfo : POINTER TO CmpIecTask.Task_Info2;
    TaskHandle: CmpIecTask.RTS_IEC_HANDLE;
    Result: CmpIecTask.RTS_IEC_RESULT;
    
	// group handler for all motion fbs
	GroupHandler: FB_GroupHandler;
	
	// tuning
	MoveDistance: LREAL := 5.0;
	MoveRelative: ARRAY[0..5] OF MC_MoveRelative;
	EstimateDeadTime: ARRAY[0..5] OF SMC_EstimateDeadTime;
	TuneStart: ARRAY[0..5] OF BOOL;
	TraceStart: ARRAY[0..5] OF BOOL;
	TraceStop: ARRAY[0..5] OF BOOL;
	SetTuningMoveValues: BOOL:=TRUE;
	fCycleTimeMs: LREAL; (* MainTask / SoftMotion bus cycle, ms *)
	fMeasuredLagMs: ARRAY[0..5] OF LREAL;
	fEstimatedDeadTimeRaw: ARRAY[0..5] OF LREAL; (* unclamped SMC_EstimateDeadTime median *)
	fSuggestedDeadTime: ARRAY[0..5] OF LREAL; (* D in MainTask cycles, clamped to (0, 20] *)
	nDeadTimeSamples: ARRAY[0..5] OF INT;
	bDeadTimeSuggested: ARRAY[0..5] OF BOOL;
	UseSuggestedD: ARRAY[0..5] OF BOOL;
	sDeadTimeHelp: ARRAY[0..5] OF STRING(255);
	fKpPeakLag: ARRAY[0..5] OF LREAL;
	fKpCruiseLag: ARRAY[0..5] OF LREAL;
	fKpFinalLag: ARRAY[0..5] OF LREAL;
	fKpOvershoot: ARRAY[0..5] OF LREAL;
	fKpUndershoot: ARRAY[0..5] OF LREAL;
	nKpReversals: ARRAY[0..5] OF INT;
	nKpVelReversals: ARRAY[0..5] OF INT;
	fSuggestedKp: ARRAY[0..5] OF LREAL;
	bKpSuggested: ARRAY[0..5] OF BOOL;
	UseSuggestedKp: ARRAY[0..5] OF BOOL;
	RaiseKp: ARRAY[0..5] OF BOOL;
	LowerKp: ARRAY[0..5] OF BOOL;
	sKpVerdict: ARRAY[0..5] OF STRING(16);
	sKpHelp: ARRAY[0..5] OF STRING(255);
	
	// homing
	ProcessHome: FB_HomingProcessor;
	HomingStatusText: STRING(255);
	
	// jogging
	ProcessJog: FB_JogProcessor;
	JogStatusText: STRING(255);
	ButtonTextIndex: INT:=0;
	
	// cnc
	VisuStruct3DControl_J: VisuStruct3DControl;
	TrackPosition: FB_PositionTracker;
	
	// program processor
	ProcessPrograms: FB_ProgramProcessor;
	ProcessCommand_ActiveProgram: UINT := 0;
	ActiveCommandDetails: STRING(255):='';
	
	// controller
	Controller: FB_Controller;
	
	//
	EnableRtrig: R_TRIG;
	EnableFtrig: F_TRIG;
	_i, _j, _k: INT;
	BuildTests: BOOL:=FALSE;
	FirstScan: BOOL:=TRUE;

END_VAR

{region "utility"}
    (* Get the current task info *)
    {IF defined (IsSimulationMode)}
        GVL_App.dwMainInterval := 1000;
    {ELSE}
        TaskHandle := CmpIecTask.IecTaskGetCurrent(ADR(Result));
        TaskInfo := CmpIecTask.IecTaskGetInfo3(TaskHandle, ADR(Result));
        GVL_App.dwMainInterval := TaskInfo^.dwInterval;
    {END_IF}
    
	GVL_App.ResetRtrig(CLK:=GVL_App.Reset);
	IF GVL_App.ResetRtrig.Q THEN
		GVL_App.Reset := FALSE;
        AlarmManager.AlarmGlobals.g_AlarmHandler.AcknowledgeAll();
        //AlarmManager.AlarmGlobals.g_AlarmHandler.AcknowledgeAllOfGroup(Alm_AlarmConfiguration_Alarmgroup_IDs.ID_Processors); // only processors
	END_IF
	
	GVL_App.LicenseStatus();

	IF FirstScan THEN
		M_Init();	
	END_IF
	
	IF BuildTests THEN
		BuildTests := FALSE;
		M_BuildTests();
	END_IF
	
	EnableRtrig(CLK:=GVL_App.PowerOn);
	EnableFtrig(CLK:=GVL_App.PowerOn);
	IF EnableRTrig.Q THEN
		GVL_App.Enable := TRUE;
	END_IF
	IF EnableFTrig.Q THEN
		GVL_App.Enable := FALSE;
	END_IF
{endregion}
     
{region "methods"}
	_M_GroupHandler();
    _M_Jogging();
    _M_Homing();
    _M_Cnc();
    _M_Processor();
	_M_Controller();
	_M_Tuning();
{endregion}

// do not put anything below this
FirstScan:=FALSE;
END_PROGRAM
