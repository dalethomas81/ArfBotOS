PROGRAM _40_Vision
// vision is separated into its own program so that we can
// assign it to its own core! how awesome is that?!
VAR
	ProcessVisionCommand: FB_VisionCmdProcessor;
   	HostName   : STRING;
	VisionSystemUrl: STRING;
	VisionOutputImageUrl: STRING;
	VisionSystemIpAddress_Last: STRING(19);
    VisionOutputImageScaling: REAL := 1000.0;
    VisionImageWidth: INT:=365; // TODO figure out a way to dynamically set this based on the values set in the visu element properties
    VisionImageHeight: INT:=255;
END_VAR
VAR CONSTANT
	LocalIpAddress: STRING(19):='127.0.0.1';
END_VAR

// used to show the output image on the HMI
// local vision:  http://<plc-hostname>.local:5000/output_sized?width=365&height=255
// remote vision: http://<vision-ip>:5000/output_sized?width=365&height=255
IF PersistentVars.VisionSystemIpAddress <> VisionSystemIpAddress_Last THEN
    VisionSystemIpAddress_Last := PersistentVars.VisionSystemIpAddress;
    //
    IF PersistentVars.VisionSystemIpAddress = LocalIpAddress THEN
        // WebVisu runs in the PC browser, so 127.0.0.1 would hit the PC.
        // Use this PLC's mDNS name instead.
        // https://forge.codesys.com/forge/talk/Runtime/thread/61d60a9fb7/
        SysSockGetHostName(szHostName:=HostName, diNameLen:=SIZEOF(HostName));
        VisionSystemUrl := CONCAT('http://', HostName);
        VisionSystemUrl := CONCAT(VisionSystemUrl, '.local');
    ELSE
        // Dedicated vision Pi: the Programming HMI IP is already that box.
        VisionSystemUrl := CONCAT('http://', PersistentVars.VisionSystemIpAddress);
    END_IF
    //
    VisionSystemUrl := CONCAT(VisionSystemUrl, ':5000/vision');
    VisionOutputImageUrl := CONCAT(VisionSystemUrl,'/output_sized?width=');
    VisionOutputImageUrl := CONCAT(VisionOutputImageUrl,INT_TO_STRING(VisionImageWidth));
    VisionOutputImageUrl := CONCAT(VisionOutputImageUrl,'&height=');
    VisionOutputImageUrl := CONCAT(VisionOutputImageUrl,INT_TO_STRING(VisionImageHeight));
END_IF

//
ProcessVisionCommand(
    VisionSystemIpAddress:= PersistentVars.VisionSystemIpAddress,
    VisionSystemPort:= PersistentVars.VisionSystemPort,
    StoredPositions:= PersistentVars.StoredPositions,
    VisionRegisters:= PersistentVars.VisionRegisters,
    Flags:= PersistentVars.Flags,
    LogicCmdIO:= GVL_App.CmdIO,
    ItfUnitMode=> ,
    Error=> ,
    ErrorDescription=> ,
    VisionStateText=> );
END_PROGRAM
