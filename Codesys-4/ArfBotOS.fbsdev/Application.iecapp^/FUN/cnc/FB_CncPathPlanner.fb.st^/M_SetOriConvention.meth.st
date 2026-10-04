METHOD PUBLIC M_SetOriConvention : BOOL
VAR_INPUT
	OriConvention: SMC_ORI_CONVENTION := SMC_ORI_CONVENTION.ZYZ;
END_VAR

_OriConvention := OriConvention;
GVL_App.OriConvention := OriConvention;
END_METHOD
