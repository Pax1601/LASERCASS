
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
TRIM    1       1       0.4     0       SIDES   0       ROLL    0       
        PITCH   0       YAW     0       URDD2   0       URDD3   27      
        URDD4   0       URDD5   0       URDD6   0       flap1r  0       
        flap2r  0       aileronr0       rudder1 0       CLIMB   0       
        BANK    0       HEAD    0       THRUST  0       
$
TRIM= 2, Sideslip levelled flight
$-------2-------3-------4-------5-------6-------7-------8-------9-------10
$
$  ID: 2
$
$ SIDESLIP LEVELLED FLIGHT determine:
$  alpha   
$  acc_x   
$  acc_y   
$  Aileron 
$  Elevator
$  Rudder  
$ Input: beta,acc_z,.
$
TRIM    2       0       0.3     0       SIDES   20      ROLL    0       
        PITCH   0       YAW     0       URDD3   9.81    URDD4   0       
        URDD5   0       URDD6   0       flap1r  0       flap2r  0       
        CLIMB   0       BANK    0       HEAD    0       THRUST  0       
$
$-------2-------3-------4-------5-------6-------7-------8-------9-------10

