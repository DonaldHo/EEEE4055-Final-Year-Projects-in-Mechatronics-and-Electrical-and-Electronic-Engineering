/**
  ******************************************************************************
  * @file     llc_converter.c
  ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "llc_converter.h"
/** Functions ----------------------------------------------------------------*/
///////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * Functions to solve phase shift.
 * Vin = Desired input voltage, Pin = Desired input and output power.
 * *Phi = Pointer of phi solve from gain Equation.
 */
int solveSSPSM(double Vin, double Pin, double *Phi) {
	// Calculations
	double G = Vout / Vin * N;
	double Rout = Vout * Vout / Pin;
	double Q = sqrt(Lr / Cr) / (N * N * Rout);

	double A = M_PI * Q * G * G + 2;
	double B = 2 * M_PI * Q * (G * G - G);
	double C = M_PI * Q * G * G - 2 * M_PI * Q * G - 2;

	// Solving quadratic equation A*x^2 + B*x + C = 0
	double discriminant = B * B - 4 * A * C;

	if (discriminant < 0) {
		printf("No real roots.\n");
		return 1;
	}

	double root1 = (-B + sqrt(discriminant)) / (2 * A);

	*Phi = acos(root1) * 180.0 / M_PI;

	// Check if a valid phase is found.
	if (*Phi > PhiMinimum && *Phi < PhiMaximum) {
	  return 1;  // Success.
	} else {
	  return 0;  // Failure.
	}
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
 * @param Phi: Initial Phi used for LLC converter.
 * @param deltaPhi: Step size for phi perturbation.
 * @param threshold: Threshold value for power change detection.
 * @return 1 if initialization is successful, otherwise 0 (invalid pointer or input values).
 */
int initPandO(struct LLC_param *LLC_var, struct PandO_param *PandO_var, double Vpv, double Ipv, double Phi, double deltaPhi, double threshold) {
    // Check for NULL pointer.
    if (PandO_var == NULL) {
        return 0;
    }

    // Check for invalid input values (NaN, infinity, or negative values if not allowed).
    if (isnan(Vpv) || isnan(Ipv) || isnan(deltaPhi) || isnan(threshold) || deltaPhi <= 0 || threshold < 0 || Phi < 0) {
        return 0;
    }

    // Initialize structure members
    PandO_var->Pold = Ipv * Vpv;
    PandO_var->Vold = Vpv;
    PandO_var->Iold = Ipv;
    PandO_var->deltaPhi = deltaPhi;
    PandO_var->threshold = threshold;
    LLC_var->PhiOld = Phi;

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

    double factor = 1.0 / (1.0 + exp(-fabs(Ppv - LLC_var->PhiOld)));

    // Check if the power change is below the threshold.
    if (fabs(Ppv - PandO_var->Pold) < PandO_var->threshold) {
        return 1;
    }

    // Power decreased.
    if ((Ppv - PandO_var->Pold) < 0) {
    	// Voltage decreased.
        if ((Vpv - PandO_var->Vold) < 0) {
        	LLC_var->PhiOld = LLC_var->PhiOld - PandO_var->deltaPhi*factor;
		// Voltage increased.
        } else {
        	LLC_var->PhiOld = LLC_var->PhiOld + PandO_var->deltaPhi*factor;
        }
    // Power increased.
    } else {
    	// Voltage decreased.
        if ((Vpv - PandO_var->Vold) < 0) {
        	LLC_var->PhiOld = LLC_var->PhiOld + PandO_var->deltaPhi*factor;
        // Voltage increased.
        } else {
        	LLC_var->PhiOld = LLC_var->PhiOld - PandO_var->deltaPhi*factor;
        }
    }

    // Update stored values for next iteration.
    PandO_var->Vold = Vpv;  // Store current voltage as previous voltage.
    PandO_var->Pold = Ppv;  // Store current power as previous power.

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

    double Phi = 0;

    // Initialize starting phase shift by assuming power is 50W.
	solveSSPSM(Vref, 50.0, &Phi);

    // Initialize structure members
    Vcontrol_var->Vref = Vref;
    Vcontrol_var->threshold = threshold;
    Vcontrol_var->flag = 1;

    // Gain to needed to move thr.
    LLC_var->PhiOld = Phi;
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

    double epsilon = 2.1;

	// Saturation function to reduce chattering
	double sat_s = s / (epsilon + fabs(s));

	// Adaptive gain based on magnitude of s
	int k;
	double abs_s = fabs(s);
	if (abs_s < 2.0) {
		k = 1;
	} else if (abs_s < 4.0) {
		k = 3;
	} else if (abs_s < 6.0) {
		k = 5;
	} else {
		k = 7;
	}

	// Update phase shift.
    LLC_var->PhiOld = LLC_var->PhiOld + (k * sat_s);
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
