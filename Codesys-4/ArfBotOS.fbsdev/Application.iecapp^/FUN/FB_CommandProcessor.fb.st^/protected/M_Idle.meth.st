METHOD PROTECTED M_Idle : BOOL

// in this state, we want to reset all the function blocks and hang out
_SeqMain := _STEP_IDLE;
_SeqAborting := 0;
_SeqClearing := 0;
_SeqResetting := 0;
END_METHOD
