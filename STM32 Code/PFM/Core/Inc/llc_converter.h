/*
 *@file: llc_converter.h
 */


#ifndef SRC_llc_converter_H_
#define SRC_llc_converter_H_


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "main.h"
#include "complex.h"
#include "math.h"


/* Private USER DEFINE ----------------------------------------------------------*/
/* USER CODE BEGIN DEFINE */
// LLC converter parameters.
#define Cr 3.0e-6
#define Lr 1.0e-6
#define Lm 8.0e-6
#define N (1.0 / 3.5)
#define Vout (120)


#define TH1 (15)
#define TH2 (10)

// LLC.
typedef struct LLC_param
{
	double Gain;
	double fold;
} LLC_var;

// P&O.
typedef struct PandO_param
{
	double Pold;
	double Vold;
	double Iold;
	double deltafreq;
	double threshold;
} PandO_var;

// CPI.
typedef struct CPI_param
{
	double f1, f2, f3, f4, f5;
	double P3, P4, P5;
	double threshold;
	int index;
} CPI_var;

// Voltage controller.
typedef struct Vcontrol_param
{
	double Vref;
	double threshold;
	int flag;
} Vcontrol_var;

// Voltage scanning.
typedef struct Vscanning_param
{
	double Vref[3];
	double Inn[3];
	int count;
} Vscanning_var;

// ADC.
typedef struct ADC_param
{
	uint32_t pointer;

	uint32_t *adc_buf_Vpv;
	uint32_t adc_buf_len_Vpv;

	uint32_t *adc_buf_Ipv;
	uint32_t adc_buf_len_Ipv;

	uint32_t *adc_buf4;
	uint32_t adc_buf_len4;
} ADC_var;

// Kalman filter.
typedef struct Kalman_param
{
	float SensorData, KalmanFilterData;
	float Xt, Xt_update, Xt_prev;
	float Pt, Pt_update, Pt_prev;
	float Kt, R, Q;
} Kalman_var;
/* USER CODE END DEFINE*/


/* Private functions----------------------------------------------*/
// FHA.
int solve_FHA_equation(double Vin, double Pin, double *fsw);
void compute_FHA_cubic_coefficients(double Vin, double Pin, double *a, double *b, double *c, double *d);
void solve_cubic(double a, double b, double c, double d, double complex roots[3]);
double find_max_real_sqrt(double complex roots[3]);

// Classical MPPT method.
int initPandO(struct LLC_param *LLC_var, struct PandO_param *PandO_var, double Vpv, double Ipv, double fsw, double deltafreq, double threshold);
int PandO(struct LLC_param *LLC_var, struct PandO_param *PandO_var, double Vpv, double Ipv);
int initCPI(struct LLC_param *LLC_var, struct CPI_param *CPI_var, double lowerBoundary, double upperBoundary, double threshold);
int CPI(struct LLC_param *LLC_var, struct CPI_param *CPI_var, double Vpv, double Ipv);

// Voltage tracking method.
int initVscanning(struct Vscanning_param *Vscanning_var, double Vref1, double Vref2, double Vref3);
int initVcontrol(struct LLC_param *LLC_var, struct Vcontrol_param *Vcontrol_var, double Vref, double threshold);
int Vcontrol(struct LLC_param *LLC_var, struct Vcontrol_param *Vcontrol_var, double Vpv);
void sliding_surface_control(struct LLC_param *LLC_var, struct Vcontrol_param *Vcontrol_var, double Vpv);

// Kalman filter.
void initKalmanFilter(struct Kalman_param *Kalman_var, float R, float Q);
double KalmanFilter(struct Kalman_param *Kalman_var, float SensorData);
/*---------------------------------------------------------------*/
#endif /* SRC_llc_converter_H_ */
