PROGRAM _60_PathPlanner
VAR
	PlanPath: FB_CncPathPlanner;
END_VAR
VAR CONSTANT
	wVarCount : WORD := 1;
END_VAR
VAR
	lrR1 : LREAL:=30;
	
	astVar : ARRAY [0..wVarCount] OF SMC_SingleVar :=
			[(strVarName:='R1', pAdr := ADR(lrR1), eVarType := SMC_VarType.SMC_TYPE_LREAL)];
			
	stVarList : SMC_VarList := (wNumberVars := wVarCount, psvVarlist := ADR(astVar));
END_VAR

// this has to be called in a separate task with lower priority
// https://content.helpme-codesys.com/en/libs/SM3_CNC/Current/SM_CNC_POUs/NC2/Interpreter/SMC_NCInterpreter.html
PlanPath(VarList:= stVarList, DataOut=> );
END_PROGRAM
