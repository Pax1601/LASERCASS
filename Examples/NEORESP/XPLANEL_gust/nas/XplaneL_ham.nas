$ EXPORT AERO MATRIX in F06 for derivatives comparison wrt NEOCASS
$
$ Warning: ALTER may depend on the NASTRAN release
$          The number of line (81 here) can be changed
ID XLANEL, HAM
TIME 10000
SOL 145
COMPILE FLUTTER LIST
ALTER 81
DIAGON(30)
MATPRN KHH,MHH,BHH// $
TABPT MKLIST// $
TABPT QHHL// $
DIAGOFF(30)
EXIT $
ENDALTER $
CEND
$
METHOD = 1
FMETHOD = 3
DISP(PRINT) = ALL
ECHO = NONE
SUBCASE 1
$
BEGIN BULK
$-------2-------3-------4-------5-------6-------7-------8-------9-------10
FLUTTER 3       PK      1       2       4       L       5
FLFACT,1,1.
FLFACT,2,0.0
FLFACT,4,4800.,6000.
PARAM,POST,-1
PARAM,OGEOM,NO
PARAM,AUTOSPC,YES
PARAM,GRDPNT,0
PARAM,OPPHIPA,-1
$
$-------2-------3-------4-------5-------6-------7-------8-------9-------10
PARAM   LMODES  6 
$
$-------2-------3-------4-------5-------6-------7-------8-------9-------10
$ Modal analysis
$-------2-------3-------4-------5-------6-------7-------8-------9-------10
EIGR           1    MGIV                              20                        
           MAX        
$
$-------2-------3-------4-------5-------6-------7-------8-------9-------10
$ Symmetry constraint
$-------2-------3-------4-------5-------6-------7-------8-------9-------10
SUPORT  2000    123456 
$
$-------2-------3-------4-------5-------6-------7-------8-------9-------10
$ Aerodynamic data
$-------2-------3-------4-------5-------6-------7-------8-------9-------10
AERO    0       238.    3.83    1.225
$
MKAERO1 0.70   
        0.001   0.01    0.05    0.1     0.2     0.5     1.0      2.0   
INCLUDE 'XplaneL_nas.inc'
$
ENDDATA