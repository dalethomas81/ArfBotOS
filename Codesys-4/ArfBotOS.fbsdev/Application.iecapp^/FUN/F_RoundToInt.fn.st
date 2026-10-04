(* Round LREAL to nearest INT (half away from zero via ±0.5 then truncate).
   LREAL_TO_INT alone truncates toward zero (27.8->27, -27.8->-27), which
   systematically under-drives |Hz| and biases crawl toward stop.
   Positive: add 0.5 then LREAL_TO_INT. Negative: subtract 0.5 then LREAL_TO_INT
   so both sides round to nearest (e.g. 27.8->28, -27.8->-28).
   Used when converting LREAL step rate (Hz) to INT16 for the Teensy PTO. *)

FUNCTION F_RoundToInt : INT
VAR_INPUT
	x : LREAL;
END_VAR
VAR
END_VAR

IF x >= 0.0 THEN
    F_RoundToInt := LREAL_TO_INT(x + 0.5);
ELSE
    F_RoundToInt := LREAL_TO_INT(x - 0.5);
END_IF
END_FUNCTION
