PROGRAM _50_Retains
VAR
	//
	SaveRetain, RestoreRetain: BOOL;
	pApplication : POINTER TO CmpApp.APPLICATION;
	ResultFind, ResultStore, ResultRestore: UDINT;
	
	//
	FirstScan: BOOL:=TRUE;
END_VAR

IF RestoreRetain THEN
	pApplication := CmpApp.AppFindApplicationByName('Application', ADR(ResultFind));
	ResultRestore := CmpApp.AppRestoreRetainsFromFile(pApplication, './Application/BackupRetain.ret');
	RestoreRetain := FALSE;
END_IF
   
IF SaveRetain THEN
	pApplication := CmpApp.AppFindApplicationByName('Application', ADR(ResultFind));
    ResultStore := AppStoreRetainsInFile(pApplication, './Application/BackupRetain.ret');
   	ResultStore := AppStoreRetainsInFile(pApplication, './Application/Application.ret');
   	SaveRetain := FALSE;
END_IF

// do not put anything below this
FirstScan := FALSE;
END_PROGRAM
