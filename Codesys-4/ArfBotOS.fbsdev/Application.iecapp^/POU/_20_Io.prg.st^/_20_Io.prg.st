PROGRAM _20_Io
VAR
	//
	IoInputs: ARRAY [0..31] OF BYTE;
	IoOutputs: ARRAY [0..31] OF BYTE;
	
	//
	rx_heartbeat: BOOL;
	rx_en: BOOL;
    
    //
    IoOption: INT:=0;
	
	//
	FirstScan: BOOL:=TRUE;
END_VAR

//
IF FirstScan THEN
    //
END_IF

// leaving this pragma here for reference and for learning.
// it is not needed and was an attempt to parameterize configuring the IO
// the project define evaluates and could be a super handy technique in the future so I'll leave it here for reference.
// the defines for the project can be found in the Toolbar under Project -> Project Settings... -> Compile options -> Edit project defines...
{IF project_defined(IoOption1)}
    //IoOption := 1;
    //_M_MapIoOption1();
{ELSIF project_defined(IoOption2)}
    //IoOption := 2;
    //_M_MapIoOption2();
{ELSE}
    //IoOption := 0;
{END_IF}

// detect if the slave is connected and map its IO
IF IoOption1.wState = ETC_SLAVE_STATE.ETC_SLAVE_OPERATIONAL THEN
    _M_MapIoOption1();
END_IF
IF IoOption2.wState = ETC_SLAVE_STATE.ETC_SLAVE_OPERATIONAL THEN
    _M_MapIoOption2();
END_IF

// do not put anything below this
FirstScan := FALSE;
END_PROGRAM
