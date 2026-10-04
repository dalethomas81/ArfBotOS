METHOD PROTECTED M_Unholding : BOOL

//
SUPER^._SeqMain := _STEP_COMMAND;

//
SUPER^.M_StateComplete();
END_METHOD
