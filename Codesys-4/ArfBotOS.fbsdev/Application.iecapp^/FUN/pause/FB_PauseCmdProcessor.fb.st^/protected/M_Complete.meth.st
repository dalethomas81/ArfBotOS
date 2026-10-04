METHOD PROTECTED M_Complete : BOOL

// once complete, reset to get back to idle
SUPER^._UnitMode.Reset();
END_METHOD
