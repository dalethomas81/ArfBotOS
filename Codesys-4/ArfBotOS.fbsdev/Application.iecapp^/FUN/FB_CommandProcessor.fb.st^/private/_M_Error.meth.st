METHOD PRIVATE _M_Error
VAR_INPUT
    Description: STRING;
END_VAR

SUPER^.M_Error(Description := CONCAT('Command Processor: ', Description));
END_METHOD
