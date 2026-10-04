METHOD PROTECTED M_Idle : BOOL

// in this state, we want to reset all the function blocks and hang out

_MC_MoveCircularAbsolute.Execute := FALSE;
_MC_MoveCircularRelative.Execute := FALSE;
_MC_MoveDirectAbsolute.Execute := FALSE;
_MC_MoveDirectRelative.Execute := FALSE;
_MC_MoveLinearAbsolute.Execute := FALSE;
_MC_MoveLinearRelative.Execute := FALSE;
_SequenceMoveCircularAbsolute := 0;
_SequenceMoveCircularRelative := 0;
_SequenceMoveDirectAbsolute := 0;
_SequenceMoveDirectRelative := 0;
_SequenceMoveLinearAbsolute := 0;
_SequenceMoveLinearRelative := 0;
_SequenceSetCoordTrans := 0;
SUPER^._SeqMain := 0;
END_METHOD
