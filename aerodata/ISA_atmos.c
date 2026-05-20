/*---------------------------------------------------------------------- */
/*                                                                       */
/* ISA atmosphere model taken from "Elements of airplane performance"    */
/* by G .J. J. Ruijgrok, Delft university press/ 1996.                   */
/*                                                                       */
/* File "ISA_atmos.c"                                                    */
/* by L. Sonneveldt                                                      */
/* May, 2006                                                             */
/*                                                                       */
/*---------------------------------------------------------------------- */

void atmos(double alt, double vt, double *coeff ){

    double rho0 = 1.225;
    double Re = 6371000;
    double R = 287.05;
    double T0 = 288.15;
    double g0 = 9.80665;
    double gamma = 1.4;
    double temp, rho, mach, qbar, grav;

    temp = T0 - 0.0065 * alt;
    if (alt >= 11000.0) {
       temp = 216.65;
    }

    rho = rho0 * exp((-g0 / (R * temp)) * alt);
    mach = vt / sqrt(gamma * R * temp);
    qbar = .5 * rho * vt * vt;
    grav = g0*(Re*Re/((Re+alt)*(Re+alt)));
 
    coeff[0] = mach;
    coeff[1] = qbar;
    coeff[2] = grav;
}
