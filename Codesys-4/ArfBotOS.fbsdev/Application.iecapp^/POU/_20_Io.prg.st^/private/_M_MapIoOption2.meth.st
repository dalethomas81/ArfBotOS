METHOD PRIVATE _M_MapIoOption2
VAR_INST
	//
	TxCrc, RxCrc: UINT;
	ChecksumMisCount: INT;
	
	//
	HeartbeatOn: TON;
	HeartbeatOff: TON;
	HeartbeatPt: TIME:=T#1000MS;
    
    //
	bufferIdx: INT;
    
    //
    DeviceID: BYTE;
    IODeviceID: BYTE:=2;
	
END_VAR

//
HeartbeatOn(in:=NOT HeartbeatOff.Q,PT:=HeartbeatPt);
HeartbeatOff(in:=HeartbeatOn.Q,PT:=HeartbeatPt);


{region 'Output'}
// check that we are talking to the right device type
IF DeviceID = IODeviceID THEN
    // index 0 is for system outputs
    IoOutputs[0].0:=HeartbeatOn.Q; // send a heartbeat to the drive
    IoOutputs[0].1:=GVL_App.Enable; //
    // IoOutputs[0].2:=; //
    // IoOutputs[0].3:=; //
    // IoOutputs[0].4:=; //
    // IoOutputs[0].5:=; //
    // IoOutputs[0].6:=; //
    // IoOutputs[0].7:=; //
    
    // map outputs
    IoOutputs[1].0 := GVL_App.CmdIO.Output[0];
    IoOutputs[1].1 := GVL_App.CmdIO.Output[1];
    IoOutputs[1].2 := GVL_App.CmdIO.Output[2];
    IoOutputs[1].3 := GVL_App.CmdIO.Output[3];
    IoOutputs[1].4 := GVL_App.CmdIO.Output[4];
    IoOutputs[1].5 := GVL_App.CmdIO.Output[5];
    IoOutputs[1].6 := GVL_App.CmdIO.Output[6];
    IoOutputs[1].7 := GVL_App.CmdIO.Output[7];
    IoOutputs[2].0 := GVL_App.CmdIO.Output[8];
    IoOutputs[2].1 := GVL_App.CmdIO.Output[9];
    IoOutputs[2].2 := GVL_App.CmdIO.Output[10];
    IoOutputs[2].3 := GVL_App.CmdIO.Output[11];
    IoOutputs[2].4 := GVL_App.CmdIO.Output[12];
    IoOutputs[2].5 := GVL_App.CmdIO.Output[13];
    IoOutputs[2].6 := GVL_App.CmdIO.Output[14];
    IoOutputs[2].7 := GVL_App.CmdIO.Output[15];
    
    // bytes 3 through 28 are unused
    //
    
    // byte 29 will be used to send the expected device id
    IoOutputs[29] := IODeviceID;
    
END_IF

// bytes 30 and 31 will be used for crc
// calc crc here
TxCrc := F_Checksum(ADR(IoOutputs), 30);
// append calc crc to tx buffer
MEMUtils.MemCpy(pbyDest:=ADR(IoOutputs[30]), 
				pbySrc:=ADR(TxCrc), 
				dwSize:=2);
{endregion}
	
{region 'Input'}
// copy crc from rx buffer
MEMUtils.MemCpy(pbyDest:=ADR(RxCrc), 
				pbySrc:=ADR(IoInputs[30]), 
				dwSize:=2);

// compare calc crc with rx crc
IF RxCrc = F_Checksum(ADR(IoInputs), 30) THEN
	// byte 0 is for system inputs
	rx_heartbeat	:= IoInputs[0].0; // receive the heartbeat back
	rx_en 			:= IoInputs[0].1; // receive the enable back
	//	 			:= IoInputs[0].2;
	// 				:= IoInputs[0].3;
	// 				:= IoInputs[0].4;
	// 				:= IoInputs[0].5;
	// 				:= IoInputs[0].6;
	// 				:= IoInputs[0].7;
    
    // map inputs
    GVL_App.CmdIO.Input[0] := IoInputs[1].0;
    GVL_App.CmdIO.Input[1] := IoInputs[1].1;
    GVL_App.CmdIO.Input[2] := IoInputs[1].2;
    GVL_App.CmdIO.Input[3] := IoInputs[1].3;
    GVL_App.CmdIO.Input[4] := IoInputs[1].4;
    GVL_App.CmdIO.Input[5] := IoInputs[1].5;
    GVL_App.CmdIO.Input[6] := IoInputs[1].6;
    GVL_App.CmdIO.Input[7] := IoInputs[1].7;
    GVL_App.CmdIO.Input[8] := IoInputs[2].0;
    GVL_App.CmdIO.Input[9] := IoInputs[2].1;
    GVL_App.CmdIO.Input[10] := IoInputs[2].2;
    GVL_App.CmdIO.Input[11] := IoInputs[2].3;
    GVL_App.CmdIO.Input[12] := IoInputs[2].4;
    GVL_App.CmdIO.Input[13] := IoInputs[2].5;
    GVL_App.CmdIO.Input[14] := IoInputs[2].6;
    GVL_App.CmdIO.Input[15] := IoInputs[2].7;
	
	// bytes 3 through 28 are unused
	//
    
    // byte 29 will be used to identify the device
	DeviceID := IoInputs[29];
	
	// Rx buffer index 30 and 31 will be for crc
ELSE
	// increment crc mismatch counter
	ChecksumMisCount := ChecksumMisCount + 1;
END_IF
{endregion}
END_METHOD
