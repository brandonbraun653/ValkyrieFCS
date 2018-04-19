#pragma once
#ifndef FCS_LQR_CONTROLLER 
#define FCS_LQR_CONTROLLER

/* Some notes:

1. One of the problems with solving the M^-1 matrix for rotor speeds is
that some of the values can be negative. 

M^-1 =  [ 1/(4*b), -1/(2*b*l),          0, -1/(4*d)]
        [ 1/(4*b),          0, -1/(2*b*l),  1/(4*d)]
        [ 1/(4*b),  1/(2*b*l),          0, -1/(4*d)]
        [ 1/(4*b),          0,  1/(2*b*l),  1/(4*d)]

According to "Attitude Stabilization of a VTOL Quadrotor Aircraft" pg.4, 
if the desired total thrust 'Ft' isn't high enough, some of the Omega^2
terms in the output become negative, indicating motor rotation reversal.

2. Results of (1) aren't practical (except for a 3D quadrotor), so the lqr
implementation is going to need to cap these extremeties before taking the
square root to solve for the actual rotor speed 'Omega'. I believe simply
setting these values to 0 should work ok. This would map to the ESC's
minimum possible command for rotation without stalling.

3. It looks like the motor mixing is already done with M^-1. The real
question is if this works with the current setup...I think the equations
were designed with a '+' configuration instead of 'x'. Should be fairly
simple to convert between the two by redefining tx and ty in (2.16). 

4. Referring to (3), the torque equations are going to need to be recalculated
and the matrix inverted again...

*/

#endif 