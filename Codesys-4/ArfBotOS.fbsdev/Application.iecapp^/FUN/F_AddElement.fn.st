FUNCTION F_AddElement
VAR_IN_OUT
	Command: STRING(GVL_App.COMMAND_LEN);
END_VAR
VAR_INPUT
	Delimiter: STRING(1):=' '; // default to space
	Element: STRING(GVL_App.COMMAND_LEN);
END_VAR
VAR
END_VAR

//
// input
// 	Element: 'bar'
// 	Delimiter: ','
// 	Command: 'foo'
//
// output
// 	Command: 'foo,bar'
//

Command:=CONCAT(Command, Element);
Command:=CONCAT(Command, Delimiter);
END_FUNCTION
