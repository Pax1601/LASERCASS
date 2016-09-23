Files included:

wing_trim_aa.inc: main file for SMARTCAD

This example shows how to use SMARTCAD trim solver in attached axis, solve_free_lin_trim_aa.

It is useful for the analysis of an isolated wing.


SUPORT and SPC co-operates to define which rigid body DOF can be left as free or not
be considered.
SUPORT card will imply equation of motion to be solved for the DOF required.
SPC card, on the other hand, will set as constaint all the parameters related to the required DOF.
For example:
SPC1    1       5       2000   
will imply URDD5 and PITCH will be treated as constrained (which means they are
not unknowns of the trim problem). 
Their value must be given in a TRIM card, even if they are null.

In this file, different heading cards are provided, depending on the type of
analysis to run.
The user can simply overwrite them with those available here, load the new file
and run the solver:

global beam_model;
beam_model=load_nastran_model('wing_trim_aa.inc');
solve_free_lin_trim_aa(1);

%
%------------------------------------------------------------------------
%
EXAMPLE 1: isolated wing at pull up

SOL 144
AEROS           0       3.57    35      125     0       0       0       
SUPORT  1       2000    12346  
TRIM= 1
$-------2-------3-------4-------5-------6-------7-------8-------9-------10
TRIM    1       1       0.7     11000   URDD3   25      SIDES   0     
        ROLL    0       PITCH   0       aileronr0       BANK    0
        THRUST  0       HEAD    0       CLIMB   0       YAW     0
        flap1r  0       flap2r  0       URDD5   0
SPC= 1
SPC1    1       5       2000   

In this case node 2000 is SUPORTed at DOF 1 2 3 4 6, which means five equations
for rigid body equilibrium will be solved.
The equation ruling pitch dynamic will not be solved.
Indeed, node 2000 is constrained at DOF 5 through an SPC card.

TRIM card will define the maneuver:
- no pitch dynamic is involved, thus URDD5=0 and PITCH=0;
- the pull up has a vertical acceleration equal to 25 m/s^2;
- please note the cards BANK THRUST HEAD and CLIMB must appear but are not used;
- symmetric maneuver, thus SIDES = 0;
- angular velocities null, ROLL, PITCH, YAW = 0;
- control surface aileronr not deflected.

Thus, the problem has 5 equations with 5 unknowns:
1) ANGLEA
2) URDD1 (x acc).
3) URDD2 (y acc).
4) URDD4 (roll acc).
5) URDD6 (yaw acc).

Of coursem being the maneuver symmetric, we expect URDD2, URDD4 and URDD6 to be null.

Running solve_free_lin_trim_aa(1), we have:

Solving rigid aircraft trim condition...
 - X acc:      -9.04986e-14 [m/s^2].
 - Y acc:      -1.68839e-07 [m/s^2].
 - Z acc:      25 [m/s^2].
 - P-DOT:      -6.62595e-07 [rad/s^2].
 - Q-DOT:      0 [rad/s^2].
 - R-DOT:      -2.1004e-09 [rad/s^2].

 - Alpha:      1.53565 [deg].
 - Sideslip:   0 [deg].
 - Roll rate:  0 [-] (p*BREF/(2VREF)).
 - Pitch rate: 0 [-] (q*CREF/(2VREF)).
 - Yaw rate:   0 [-] (r*BREF/(2VREF)).
 - Control flap1r:   0 [deg].
 - Control flap2r:   0 [deg].
 - Control aileronr:   0 [deg].
done.
Solving deformable aircraft trim condition...
 - X acc:      -1.00552e-13 [m/s^2].
 - Y acc:      -1.68311e-07 [m/s^2].
 - Z acc:      25 [m/s^2].
 - P-DOT:      -7.31159e-07 [rad/s^2].
 - Q-DOT:      0 [rad/s^2].
 - R-DOT:      -2.27723e-09 [rad/s^2].

 - Alpha:      1.47544 [deg].
 - Sideslip:   0 [deg].
 - Roll rate:  0 [-] (p*BREF/(2VREF)).
 - Pitch rate: 0 [-] (q*CREF/(2VREF)).
 - Yaw rate:   0 [-] (r*BREF/(2VREF)).
 - Control flap1r:   0 [deg].
 - Control flap2r:   0 [deg].
 - Control aileronr:   0 [deg].
done.

completed.

In beam_model.Res.Aero.RStab_Der and beam_model.Res.Aero.DStab_Der,
you can find aeroelasticc derivatives.

WARNING: since we constrained DOF 5 there will be no aero derivative associated to this DOF.

If you want these derivatives simply modify the file into:

SOL 144
AEROS           0       3.57    35      125     0       0       0       
SUPORT  1       2000    123456  
TRIM= 1
$-------2-------3-------4-------5-------6-------7-------8-------9-------10
TRIM    1       1       0.7     11000   URDD3   25      SIDES   0     
        ROLL    0       PITCH   0       aileronr0       BANK    0
        THRUST  0       HEAD    0       CLIMB   0       YAW     0
        flap1r  0       flap2r  0       
$SPC= 1
$SPC1    1       135     2002    2020   
$SPC1    1       5       2000   
%------------------------------------------------------------------------

In this case, we have 6 DOS with 6 equation and all the derivatives will
be available.
%
%------------------------------------------------------------------------
%
EXAMPLE 2: wing in roll, given an aileron input command, determine the 
           istantaneous acceleration

SOL 144
AEROS           0       3.57    35      125     0       0       0       
SUPORT  1       2000    4  
TRIM= 1
$-------2-------3-------4-------5-------6-------7-------8-------9-------10
TRIM    1       1       0.7     11000   URDD3   0       SIDES   0     
        ROLL    0       PITCH   0       aileronr5       BANK    0
        THRUST  0       HEAD    0       CLIMB   0       YAW     0
        flap1r  0       flap2r  0       URDD5   0       URDD6   0
        URDD1   0       URDD2   0       ANGLEA  0
SPC= 1
SPC1    1       12356   2000 

Running solve_free_lin_trim_aa(1), we have:

Solving rigid aircraft trim condition...
 - X acc:      0 [m/s^2].
 - Y acc:      0 [m/s^2].
 - Z acc:      0 [m/s^2].
 - P-DOT:      1.82396 [rad/s^2].
 - Q-DOT:      0 [rad/s^2].
 - R-DOT:      0 [rad/s^2].

 - Alpha:      0 [deg].
 - Sideslip:   0 [deg].
 - Roll rate:  0 [-] (p*BREF/(2VREF)).
 - Pitch rate: 0 [-] (q*CREF/(2VREF)).
 - Yaw rate:   0 [-] (r*BREF/(2VREF)).
 - Control flap1r:   0 [deg].
 - Control flap2r:   0 [deg].
 - Control aileronr:   5 [deg].
done.
Solving deformable aircraft trim condition...
 - X acc:      0 [m/s^2].
 - Y acc:      0 [m/s^2].
 - Z acc:      0 [m/s^2].
 - P-DOT:      1.73715 [rad/s^2].
 - Q-DOT:      0 [rad/s^2].
 - R-DOT:      0 [rad/s^2].

 - Alpha:      0 [deg].
 - Sideslip:   0 [deg].
 - Roll rate:  0 [-] (p*BREF/(2VREF)).
 - Pitch rate: 0 [-] (q*CREF/(2VREF)).
 - Yaw rate:   0 [-] (r*BREF/(2VREF)).
 - Control flap1r:   0 [deg].
 - Control flap2r:   0 [deg].
 - Control aileronr:   5 [deg].
done.

completed.

You can plot the defomed shape:

plot_beam_defo(1,100) where 100 is a scale magnitude factor.
%
%------------------------------------------------------------------------
%
EXAMPLE: wing in roll, given an aileron input command, determine the 
           steady roll rate

SOL 144
AEROS           0       3.57    35      125     0       0       0       
SUPORT  1       2000    4  
TRIM= 1
$-------2-------3-------4-------5-------6-------7-------8-------9-------10
TRIM    1       1       0.7     11000   URDD3   0       SIDES   0     
        URDD4   0       PITCH   0       aileronr5       BANK    0
        THRUST  0       HEAD    0       CLIMB   0       YAW     0
        flap1r  0       flap2r  0       URDD5   0       URDD6   0
        URDD1   0       URDD2   0       ANGLEA  0
SPC= 1
SPC1    1       12356   2000 

In this case we set URDD4 = 0 and leave ROLL as unknown.

Running solve_free_lin_trim_aa(1), we have:

Solving rigid aircraft trim condition...
 - X acc:      0 [m/s^2].
 - Y acc:      0 [m/s^2].
 - Z acc:      0 [m/s^2].
 - P-DOT:      0 [rad/s^2].
 - Q-DOT:      0 [rad/s^2].
 - R-DOT:      0 [rad/s^2].

 - Alpha:      0 [deg].
 - Sideslip:   0 [deg].
 - Roll rate:  0.0279866 [-] (p*BREF/(2VREF)).
 - Pitch rate: 0 [-] (q*CREF/(2VREF)).
 - Yaw rate:   0 [-] (r*BREF/(2VREF)).
 - Control flap1r:   0 [deg].
 - Control flap2r:   0 [deg].
 - Control aileronr:   5 [deg].
done.
Solving deformable aircraft trim condition...
 - X acc:      0 [m/s^2].
 - Y acc:      0 [m/s^2].
 - Z acc:      0 [m/s^2].
 - P-DOT:      0 [rad/s^2].
 - Q-DOT:      0 [rad/s^2].
 - R-DOT:      0 [rad/s^2].

 - Alpha:      0 [deg].
 - Sideslip:   0 [deg].
 - Roll rate:  0.0252112 [-] (p*BREF/(2VREF)).
 - Pitch rate: 0 [-] (q*CREF/(2VREF)).
 - Yaw rate:   0 [-] (r*BREF/(2VREF)).
 - Control flap1r:   0 [deg].
 - Control flap2r:   0 [deg].
 - Control aileronr:   5 [deg].
done.

completed.
