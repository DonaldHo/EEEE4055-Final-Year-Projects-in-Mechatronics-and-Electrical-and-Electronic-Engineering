% /////////////////////////////////////////////////////////////////
% A Smart Maximum Power Point Tracking for LLC Converter.
% /////////////////////////////////////////////////////////////////


% Clear all data.
clc; clear; close all;
	
	
% ////////////////////////////////////////////////////////////////
% Load data.
% ////////////////////////////////////////////////////////////////T
% Read Excel file.
data = readtable('Test.xlsx');

% Loop through all possibility


% Select which data to test.
N = 6;   
% 2 Error
% 12, 16, 17
% 21, 25, 26

% Irradiance values.
Irradiance1 = data.Irradiance1(N);
Irradiance2 = data.Irradiance2(N);
Irradiance3 = data.Irradiance3(N);

% Temperature.
T = data.Temperature(N);

% Input data for NN.
I1 = data.I1(N);
I2 = data.I2(N);
I3 = data.I3(N);

% Actual data.
Vactual = [data.VP1(N) data.VP2(N) data.VP3(N)];
Pactual = [data.P1(N) data.P2(N) data.P3(N)];
% ////////////////////////////////////////////////////////////////


% ////////////////////////////////////////////////////////////////
% Parameters for LLC resonance converter circuit.
% ////////////////////////////////////////////////////////////////
% PV input.
IR1 = Irradiance1;
IR2 = Irradiance2;
IR3 = Irradiance3;
Cpv = 100e-6;

% Resonance tank.
Cr = 3e-6;
Lr = 1e-6;
Lm = 8e-6;

% Transformer, voltage doubler, rectifier and output filter.
N = 3.5;
Cd = 33e-6;
Co = 200e-6;

% Output load.
Rleak = 1;
Vout = 120;
% ////////////////////////////////////////////////////////////////


% save('PSM_T25_100200700.mat', 'out'); % Save variable 'x' to 'mydata.mat'
