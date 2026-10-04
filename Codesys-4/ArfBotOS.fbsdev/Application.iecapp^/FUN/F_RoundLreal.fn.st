FUNCTION F_RoundLreal : LREAL
VAR_INPUT
   rIn : LREAL;      
   Digit: INT;         
END_VAR

VAR
   d1 : DINT;
   r0,r1,r2,Rdigit: LREAL;               
END_VAR

// https://forge.codesys.com/forge/talk/Engineering/thread/8d1ee86294/
rdigit   :=INT_TO_LREAL(Digit);
r0 := EXPT(10,rdigit); 
r1 := rIn * r0;
d1 := TRUNC(r1);
r2 := DINT_TO_LREAL(d1);
F_RoundLreal := r2 / r0;
END_FUNCTION
