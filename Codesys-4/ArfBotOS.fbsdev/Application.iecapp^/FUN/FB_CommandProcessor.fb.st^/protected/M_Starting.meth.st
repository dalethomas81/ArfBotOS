METHOD PROTECTED M_Starting : BOOL

// here we want to clear things and get ready for the execute state
// we can consider this the "INIT" state
_SeqMain := _STEP_CMD;
SUPER^._TimeoutTimer.PT := T#0MS; // timeout is ignored if set to 0
SUPER^.M_StateComplete();
END_METHOD
