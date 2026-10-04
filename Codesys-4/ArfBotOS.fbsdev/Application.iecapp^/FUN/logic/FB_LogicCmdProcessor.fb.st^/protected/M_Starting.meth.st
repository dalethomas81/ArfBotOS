METHOD PROTECTED M_Starting : BOOL

// here we want to clear things and get ready for the execute state
// I can consider this the "INIT" state
SUPER^._SeqMain := _STEP_COMMAND;
SUPER^.M_StateComplete();
END_METHOD
