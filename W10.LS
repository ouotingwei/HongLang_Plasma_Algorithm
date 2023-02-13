/PROG  W10.LS
/ATTR
OWNER       = MNEDITOR;
COMMENT     = "";
PROG_SIZE   = 636;
CREATE      = DATE 23-01-07  TIME 11:59:14;
MODIFIED    = DATE 23-01-07  TIME 12:02:18;
FILE_NAME   = ;
VERSION     = 0;
LINE_COUNT  = 4;
MEMORY_SIZE = 992;
PROTECT     = READ_WRITE;
TCD:  STACK_SIZE	= 0,
      TASK_PRIORITY	= 50,
      TIME_SLICE	= 0,
      BUSY_LAMP_OFF	= 0,
      ABORT_REQUEST	= 0,
      PAUSE_REQUEST	= 0;
DEFAULT_GROUP	= 1,*,*,*,*;
CONTROL_CODE	= 00000000 00000000;
/MN
   1:J P[1] 100% FINE    ;
   2:L P[2] 100mm/sec CNT100    ;
/POS
P[1]{
   GP1:
	UF : 0, UT : 6,		CONFIG : 'N U T, 0, 0, 0',
	X =  210.000  mm,	Y =   -375.000  mm,	Z =   -250.000  mm,
	W =  90.000 deg,	P =   0.000 deg,	R =   90.000 deg
};
P[2]{
   GP1:
	UF : 0, UT : 6,		CONFIG : 'N U T, 0, 0, 0',
	X =  210.000  mm,	Y =   -375.000  mm,	Z =   -250.000  mm,
	W =  90.000 deg,	P =   0.000 deg,	R =   90.000 deg
};
/END
