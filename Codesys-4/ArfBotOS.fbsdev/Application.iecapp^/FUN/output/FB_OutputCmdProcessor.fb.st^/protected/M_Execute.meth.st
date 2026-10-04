METHOD PROTECTED M_Execute : BOOL

// here we want to execute our commands/jobs/etc and set completed when done.
IF SUPER^._SeqMain =_STEP_IDLE THEN
	SUPER^.M_StateComplete();
END_IF
END_METHOD
