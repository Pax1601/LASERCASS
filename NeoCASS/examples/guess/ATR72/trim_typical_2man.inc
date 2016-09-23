
$
$ Flight condition parameters
$
TRIM= 1, Cruise/Climb
$-------2-------3-------4-------5-------6-------7-------8-------9-------10
$
$  ID: 1
$
$ CRUISE/CLIMB determine:
$  alpha   
$  acc_x   
$  Elevator
$ Input: acc_z,.
$
TRIM    1       1       0.44    4444    SIDES   0       ROLL    0       
        PITCH   0       YAW     0       URDD2   0       URDD3   25      
        URDD4   0       URDD5   0       URDD6   0       flap1r  0       
        flap2r  0       aileronr0       rudder1 0       CLIMB   0       
        BANK    0       HEAD    0       THRUST  0       
$
TRIM= 2, Snap roll
$-------2-------3-------4-------5-------6-------7-------8-------9-------10
$
$  ID: 2
$
$ SNAP ROLL determine:
$  acc_x 
$  acc_y 
$  acc_z 
$  p_rate
$  q_rate
$  r_rate
$ Input: alpha,Elevator,Rudder,.
$
TRIM    2       0       .55     5555    ANGLEA  5       SIDES   0       
        ROLL    0       PITCH   0       YAW     0       flap1r  0       
        flap2r  0       aileronr0       elev1r  15      rudder1 15      
        CLIMB   0       BANK    0       HEAD    0       THRUST  0       
$
$-------2-------3-------4-------5-------6-------7-------8-------9-------10

