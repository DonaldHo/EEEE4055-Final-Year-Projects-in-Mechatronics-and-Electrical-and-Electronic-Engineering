/**
  ******************************************************************************
  * @file     llc_converter.c
  ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "llc_converter.h"
/** Functions ----------------------------------------------------------------*/
///////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * Function to solve the FHA equation for an LLC resonant converter.
 *
 * This function:
 * 1. Computes the cubic equation coefficients using input voltage and power.
 * 2. Solves the cubic equation for three complex roots.
 * 3. Finds the maximum real square root among the solutions.
 * 4. Multiplies the result with the resonance frequency.
 *
 * @param1 Vin: Input voltage of PV panel (VGMPPT).
 * @param2 Pin: Input power of PV panel (PGMPPT).
 * @param3 fsw: Pointer to store the switching frequency result.
 *
 * @return Returns 1 if a valid frequency is found, otherwise returns 0.
*/
int solve_FHA_equation(double Vin, double Pin, double *fsw) {
	double complex roots[3];  // Array to store roots of the cubic equation.
	double a, b, c, d;        // Coefficients for the cubic equation.

	// Compute cubic equation coefficients based on input voltage and power.
	compute_FHA_cubic_coefficients(Vin, Pin, &a, &b, &c, &d);

	// Solve the cubic equation for three complex roots.
	solve_cubic(a, b, c, d, roots);

	// Find the maximum real square root among the computed roots.
	double max_root = find_max_real_sqrt(roots);

	double fr = 1/(2*3.1416*sqrt(Lr*Cr));

	// Check if a valid frequency is found.
	if (max_root > 0) {
	  *fsw = max_root * fr; // Multiply by resonance frequency.
	  return 1;  // Success.
	} else {
	  return 0;  // Failure.
	}
}

/**
 * Function to compute the coefficients of a cubic equation in the form:
 *
 *     a*x^3 + b*x^2 + c*x + d = 0
 *
 * based on the given input voltage (Vin) and power (Pin).
 *
 * @param1 Vin: Input voltage of PV panel.
 * @param2 Pin: Input power of PV panel.
 * @param3 a: Pointer to store the coefficient of x^3.
 * @param4 b: Pointer to store the coefficient of x^2.
 * @param5 c: Pointer to store the coefficient of x.
 * @param6 d: Pointer to store the constant term.
 */
void compute_FHA_cubic_coefficients(double Vin, double Pin, double *a, double *b, double *c, double *d) {
    // Assume 100% efficiency
//    double Rload = (Vout * Vout) / Pin;
    double Rload = 325;

    // Equivalent AC resistance for full-bridge LLC with full-bridge rectification
    double Rac = (8.0 / (M_PI * M_PI)) * (N * N) * (0.5 * 0.5) * Rload;

    // Parameters for the cubic equation
    double K = Lm / Lr;
    double Q = sqrt(Lr / Cr) / Rac;

    double Gain = 0;
    if (Vin>TH1) {
    	Gain = (Vout/Vin) * N /2*2;
    }
    else if (Vin>TH2) {
		Gain = (Vout/Vin) * N /2;
	}
    else{
    	Gain = (Vout / Vin) * N /2/2;
    }
    // Compute cubic equation coefficients
    *a = 1.0;
    *b = ((pow(K, -2) * pow(K + 1, 2)) - (2.0 * Q * Q) - (pow(Gain, -2))) / (Q * Q);
    *c = (Q * Q - (2.0 * pow(K, -2) * (K + 1))) / (Q * Q);
    *d = pow(1.0 / (K * Q), 2);
}

/**
 * Function to solve a cubic equation of the form:
 *
 *     a*x³ + b*x² + c*x + d = 0
 *
 * This function uses Cardano’s method to find the real and complex roots.
 *
 * @param1 a: Coefficient of x³.
 * @param2 b: Coefficient of x².
 * @param3 c: Coefficient of x.
 * @param4 d: Constant term.
 * @param5 roots: Array to store the three roots of the equation (real or complex).
 */
void solve_cubic(double a, double b, double c, double d, double complex roots[3]) {
    // Convert to a depressed cubic equation by normalizing coefficients.
    double A = b / a;  // Normalize b.
    double B = c / a;  // Normalize c.
    double C = d / a;  // Normalize d.

    // Compute intermediate values used in the cubic formula.
    double Q = (3 * B - pow(A, 2)) / 9;  // Q helps determine the nature of the roots.
    double R = (9 * A * B - 27 * C - 2 * pow(A, 3)) / 54; // R is a key term in root calculation.
    double D = pow(Q, 3) + pow(R, 2); // Discriminant: determines the nature of the roots.

    if (D >= 0) {
        // Case 1: One real root, two complex conjugate roots.
        double S = cbrt(R + sqrt(D)); // Cube root of (R + sqrt(D)).
        double T = cbrt(R - sqrt(D)); // Cube root of (R - sqrt(D)).

        // The first root is real.
        roots[0] = S + T - A / 3;

        // Compute real and imaginary parts of the two complex conjugate roots.
        double real_part = -0.5 * (S + T) - A / 3; // Real part of complex roots.
        double imag_part = sqrt(3) / 2 * (S - T);  // Imaginary part of complex roots.

        // Store the complex conjugate roots.
        roots[1] = real_part + imag_part * I;
        roots[2] = real_part - imag_part * I;
    } else {
        // Case 2: Three real roots (D < 0).
        double theta = acos(R / sqrt(-pow(Q, 3))); // Angle for trigonometric solution.

        // Compute the three real roots using trigonometric identities.
        roots[0] = 2 * sqrt(-Q) * cos(theta / 3) - A / 3;
        roots[1] = 2 * sqrt(-Q) * cos((theta + 2 * M_PI) / 3) - A / 3;
        roots[2] = 2 * sqrt(-Q) * cos((theta - 2 * M_PI) / 3) - A / 3;
    }
}

/**
 * Function to compute the square roots of the given complex numbers
 * and return the maximum real value among them.
 *
 * @param1 roots: Array containing three complex numbers.
 * @return Maximum real value among the square roots of the given complex numbers.
 */
double find_max_real_sqrt(double complex roots[3]) {
    double max_value = 0.00;

    for (int i = 0; i < 3; i++) {
        double complex sqrt_root = csqrt(roots[i]);

        // If the result is real, compare with max_value.
        if (cimag(sqrt_root) == 0) {
            double real_part = creal(sqrt_root);
            if (real_part > max_value) {
                max_value = real_part;
            }
        }
    }
    return max_value;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * Function to initialize the Perturb and Observe (P&O) parameters.
 *
 * This function sets the initial values for the P&O algorithm used in Maximum Power Point Tracking (MPPT).
 * It ensures valid inputs and initializes the given structure with the specified parameters.
 *
 * @param PandO_var: Pointer to PandO_param structure, which stores past values and algorithm parameters.
 * @param Vpv: Photovoltaic (PV) voltage measurement.
 * @param Ipv: Photovoltaic (PV) current measurement.
 * @param fsw: Switching frequency for LLC converter.
 * @param deltafreq: Step size for frequency perturbation.
 * @param threshold: Threshold value for power change detection.
 * @return 1 if initialization is successful, otherwise 0 (invalid pointer or input values).
 */
int initPandO(struct LLC_param *LLC_var, struct PandO_param *PandO_var, double Vpv, double Ipv, double fsw, double deltafreq, double threshold) {
    // Check for NULL pointer.
    if (PandO_var == NULL) {
        return 0;
    }

    // Check for invalid input values (NaN, infinity, or negative values if not allowed).
    if (isnan(Vpv) || isnan(Ipv) || isnan(deltafreq) || isnan(threshold) || deltafreq <= 0 || threshold < 0 || fsw < 0) {
        return 0;
    }

    // Initialize structure members
    PandO_var->Pold = Ipv * Vpv;
    PandO_var->Vold = Vpv;
    PandO_var->Iold = Ipv;
    PandO_var->deltafreq = deltafreq;
    PandO_var->threshold = threshold;
    LLC_var->fold = fsw;

    return 1;
}

/**
 * Function for Perturb and Observe (P&O) algorithm to adjust frequency.
 *
 * @param LLC_var: Pointer to LLC_param structure, which holds past values and parameters.
 * @param Vpv: PV voltage (current voltage measurement).
 * @param Ipv: PV current (current current measurement).
 * @return 1 if power change is below threshold (no update needed), otherwise 0.
 */
int PandO(struct LLC_param *LLC_var, struct PandO_param *PandO_var, double Vpv, double Ipv) {
    // Compute current power based on PV voltage and current.
    double Ppv = Vpv * Ipv;

    double factor = 1.0 / (1.0 + exp(-fabs(Ppv - PandO_var->Pold)));

    // Check if the power change is below the threshold.
    if (fabs(Ppv - PandO_var->Pold) < PandO_var->threshold) {
        return 1;
    }

    // Power decreased.
    if ((Ppv - PandO_var->Pold) < 0) {
    	// Voltage decreased.
        if ((Vpv - PandO_var->Vold) < 0) {
        	LLC_var->fold = LLC_var->fold + PandO_var->deltafreq * factor;
		// Voltage increased.
        } else {
        	LLC_var->fold = LLC_var->fold - PandO_var->deltafreq * factor;
        }
    // Power increased.
    } else {
    	// Voltage decreased.
        if ((Vpv - PandO_var->Vold) < 0) {
        	LLC_var->fold = LLC_var->fold - PandO_var->deltafreq * factor;
        // Voltage increased.
        } else {
        	LLC_var->fold = LLC_var->fold + PandO_var->deltafreq * factor;
        }
    }

    // Update stored values for next iteration.
    PandO_var->Vold = Vpv;  // Store current voltage as previous voltage.
    PandO_var->Pold = Ppv;  // Store current power as previous power.

    return 0;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * Initializes the CPI (Center Point Iteration) parameters for MPPT algorithm.
 *
 * This function sets up the initial frequency range and iteration parameters for the CPI algorithm.
 *
 * @param LLC_var: Pointer to LLC_param structure, used to store the selected frequency.
 * @param CPI_var: Pointer to CPI_param structure, which stores past power values and frequency settings.
 * @param lowerBoundary: The lower boundary of the frequency range.
 * @param upperBoundary: The upper boundary of the frequency range.
 * @param threshold: The power convergence threshold for stopping conditions.
 * @return 1 if initialization is successful, otherwise 0 (invalid pointer or input values).
 */

int initCPI(struct LLC_param *LLC_var, struct CPI_param *CPI_var, double lowerBoundary, double upperBoundary, double threshold) {
    // Check for NULL pointer.
    if (LLC_var == NULL || CPI_var == NULL) {
        return 0;
    }

    // Check for invalid input values (NaN, infinity, or negative values if not allowed).
    if (isnan(lowerBoundary) || isnan(upperBoundary) || threshold < 0) {
        return 0;
    }

    // Initialize structure members
    CPI_var->f1 = lowerBoundary;
    CPI_var->f2 = upperBoundary;
    CPI_var->f3 = (lowerBoundary+upperBoundary)/2;
    CPI_var->f4 = (CPI_var->f1 + CPI_var->f3) / 2;
    CPI_var->f5 = (CPI_var->f3 + CPI_var->f2) / 2;
    CPI_var->P3 = 0.0;
    CPI_var->P4 = 0.0;
    CPI_var->P5 = 0.0;
    CPI_var->threshold = threshold;
    CPI_var->index = 3; // Start iteration from index 2

    LLC_var->fold = CPI_var->f3;

    return 1;
}

/**
 * Function for Center Point Iteration (CPI) algorithm.
 *
 * @param LLC_var: Pointer to LLC_param structure for system parameters.
 * @param CPI_var: Pointer to CPI_param structure for CPI algorithm state.
 * @param Vpv: PV voltage (current voltage measurement).
 * @param Ipv: PV current (current current measurement).
 * @return 1 if the iteration is complete (power change below threshold), otherwise 0.
 */
int CPI(struct LLC_param *LLC_var, struct CPI_param *CPI_var, double Vpv, double Ipv) {
    // Compute current power based on PV voltage and current.
    double Ppv = Vpv * Ipv;

    if (CPI_var->index == 3) {
        CPI_var->P3 = Ppv;
        LLC_var->fold = CPI_var->f4;
        CPI_var->index = 4;
    } else if (CPI_var->index == 4) {
        CPI_var->P4 = Ppv;
        LLC_var->fold = CPI_var->f5;
        CPI_var->index = 5;
    } else if (CPI_var->index == 5) {
        CPI_var->P5 = Ppv;

        // Find max and min power among P3, P4, and P5.
        double Pmax = fmax(fmax(CPI_var->P3, CPI_var->P4), CPI_var->P5);
        double Pmin = fmin(fmin(CPI_var->P3, CPI_var->P4), CPI_var->P5);

        // Identify which frequency corresponds to the max power.
        int idx = (CPI_var->P3 == Pmax) ? 1 : (CPI_var->P4 == Pmax) ? 2 : 3;

        // Adjust frequency values based on the max power index.
        if (idx == 1) {
            CPI_var->f1 = CPI_var->f4;
            CPI_var->f2 = CPI_var->f5;
        } else if (idx == 2) {
            CPI_var->f2 = CPI_var->f3;
            CPI_var->f3 = CPI_var->f4;
        } else if (idx == 3) {
            CPI_var->f1 = CPI_var->f3;
            CPI_var->f3 = CPI_var->f5;
        }

        // Update f4 and f5 as midpoints of the selected frequency range.
        CPI_var->f5 = (CPI_var->f3 + CPI_var->f2) / 2;
        CPI_var->f4 = (CPI_var->f1 + CPI_var->f3) / 2;

        // Reset frequency to f3 and restart iteration.
        LLC_var->fold = CPI_var->f3;
        CPI_var->index = 3;

        // If power change is below the threshold, stop iteration.
        if (fabs(Pmax - Pmin) < CPI_var->threshold) {
            return 1;
        }
    }

    return 0;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////////////////////
int initVscanning(struct Vscanning_param *Vscanning_var, double Vref1, double Vref2, double Vref3) {
    // Check for NULL pointer.
    if (Vscanning_var == NULL) {
        return 0;
    }

    // Check for invalid input values (NaN, infinity, or negative values if not allowed).
    if (Vref1 < 0 || Vref2 < 0 || Vref3 < 0) {
        return 0;
    }

    // Assigning the array pointer to structure.
    Vscanning_var->Vref[0] = Vref1;
    Vscanning_var->Vref[1] = Vref2;
    Vscanning_var->Vref[2] = Vref3;

    Vscanning_var->count = 0;

    return 1;
}


int initVcontrol(struct LLC_param *LLC_var, struct Vcontrol_param *Vcontrol_var, double Vref, double threshold) {
    // Check for NULL pointer.
    if (Vcontrol_var == NULL) {
        return 0;
    }

    // Check for invalid input values (NaN, infinity, or negative values if not allowed).
    if (threshold < 0) {
        return 0;
    }

    double fsw = 0;

    // Initialize starting frequency.
    solve_FHA_equation(Vref, 50, &fsw);

    // Initialize structure members
    Vcontrol_var->Vref = Vref;
    Vcontrol_var->threshold = threshold;
    Vcontrol_var->flag = 1;

    // Gain to needed to move thr.
    LLC_var->Gain = Vout/Vref;
    LLC_var->fold = fsw;
    return 1;
}



/**
 * Function to implement voltage control for an LLC resonant converter.
 *
 * This function compares the reference voltage with the actual PV voltage and
 * determines whether control adjustments are necessary. It uses different control
 * strategies based on the gain of the LLC converter.
 *
 * - If the difference between the reference voltage and the PV voltage is within
 *   the threshold, the function returns 1, indicating no control action is needed.
 * - If the gain of the LLC converter is less than 0.9, the function applies a
 *   control method using `V_CPI()`.
 * - Otherwise, it applies a sliding mode control method using `sliding_surface_control()`.
 *
 * @param1 LLC_var: Pointer to an LLC_param structure containing LLC converter parameters.
 * @param2 Vcontrol_var: Pointer to a Vcontrol_param structure containing control parameters.
 * @param3 Vpv: The measured PV panel voltage.
 *
 * @return Returns 1 if the voltage is within the threshold (no action needed),
 *         otherwise returns 0 after applying the appropriate control strategy.
 */
int Vcontrol(struct LLC_param *LLC_var, struct Vcontrol_param *Vcontrol_var, double Vpv) {
	if (fabs(Vcontrol_var->Vref - Vpv) < Vcontrol_var->threshold) {
		Vcontrol_var->flag = 0;
		return 1;
	}
	else {
		sliding_surface_control(LLC_var, Vcontrol_var, Vpv);
	}
	return 0;
}


/**
 * Function to track Vref when gain > 0.9.
 *
 * @param1 Vref: Reference voltage.
 * @param1 Vpv: PV voltage.
 */
void sliding_surface_control(struct LLC_param *LLC_var, struct Vcontrol_param *Vcontrol_var, double Vpv) {
	// Define the sliding surface as the error between measured and reference voltage.
    double s = Vpv - Vcontrol_var->Vref;

    // Adaptive gain based on magnitude of s
	int k;
	double abs_s = fabs(s);
	if (abs_s < 2.0) {
		k = 1e3;
	} else if (abs_s < 4.0) {
		k = 3e3;
	} else if (abs_s < 6.0) {
		k = 5e3;
	} else {
		k = 7e3;
	}

    // Define boundary layer to reduce chattering.
    double epsilon = 2.1;

    // Compute saturation function for sliding control.
    double sat_s = s / (epsilon + fabs(s));

    // Adjust the switching frequency based on sliding control.
    LLC_var->fold = LLC_var->fold - (k * sat_s);
//    LLC_var->fold = LLC_var->fold - 500;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////////////////////
void initKalmanFilter(struct Kalman_param *Kalman_var, float R, float Q) {
    Kalman_var->R = R;
    Kalman_var->Q = Q;
    Kalman_var->Pt_prev = 1;
}

double KalmanFilter(struct Kalman_param *Kalman_var, float SensorData) {
    Kalman_var->Xt_update = Kalman_var->Xt_prev;
    Kalman_var->Pt_update = Kalman_var->Pt_prev + Kalman_var->Q;

    Kalman_var->Kt = Kalman_var->Pt_update / (Kalman_var->Pt_update + Kalman_var->R);
    Kalman_var->Xt = Kalman_var->Xt_update + (Kalman_var->Kt * (SensorData - Kalman_var->Xt_update));
    Kalman_var->Pt = (1 - Kalman_var->Kt) * Kalman_var->Pt_update;

    Kalman_var->Xt_prev = Kalman_var->Xt;
    Kalman_var->Pt_prev = Kalman_var->Pt;

    Kalman_var->KalmanFilterData = Kalman_var->Xt;

    return Kalman_var->KalmanFilterData;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////
