% ///////////////////////////////////////////////////////////////////////
% Script to control and collect data of a Simulink model.
% ///////////////////////////////////////////////////////////////////////


clc; clear all; close all;


% Initialize data storage variable.
data = [];


% Specify the Simulink model file name and model name.
filename = 'PandO.slx';
model = 'PandO';


% Open the Simulink model.
open_system(filename);


% Define the simulation stop time in seconds.
Stoptime = 10;


% Set the simulation stop time parameter in the model.
set_param(model, 'StopTime', num2str(Stoptime));


% Temperature.
T_Array = [50];
T = 0;

values = [1, 25:25:1000]';


% C(21+3-1, 3) = C(23, 3) = 1773
n = length(values);  % Number of unique values (21 in this case).
k = 3;  % We want combinations of 3.


% Pre-allocate array for efficiency
num_combinations = nchoosek(n + k - 1, k);  % Formula for combinations with replacement.
IrradianceArray = zeros(num_combinations, k);


% Generate combinations with replacement.
idx = 1;
for i = 1:n
    for j = i:n  % Start from i to avoid reverse duplicates.
        for m = j:n  % Start from j to avoid reverse duplicates.
            IrradianceArray(idx, :) = [values(i) values(j) values(m)];
            idx = idx + 1;
        end
    end
end


% Define the number of test cases (N) and the maximum irradiance value.
% N = 10;                          % Number of test cases.
% MaxIrradiance = 1000;            % Maximum irradiance (W/m^2).

% Set the random seed for reproducibility and generate irradiance values.
% rng(33);
% IrradianceArray = round(MaxIrradiance * rand(N, 3), 0);
IrradianceArray = [150 200 850; 250 700 1000; 250 350 850];

for i = 1:length(T_Array)
T = T_Array(i);

% Iterate through each row in the IrradianceArray to run the simulations. 
for j = 1:size(IrradianceArray, 1)
    fprintf("Temperature = %d, Percentage = %.2f\n", T, 100*j/size(IrradianceArray, 1));

    % Set simulation parameters for the current test case.
    Irradiance1 = IrradianceArray(j, 1);  % Irradiance for PV 1.
    Irradiance2 = IrradianceArray(j, 2);  % Irradiance for PV 2.
    Irradiance3 = IrradianceArray(j, 3);  % Irradiance for PV 3.

    % Initialize a flag to track simulation success.
    simulationSuccessful = false;

    % Set a counter for the number of attempts.
    attemptCount = 0;
    maxAttempts = 5;  % Maximum number of attempts.

    % Temporary data array for the current simulation
    data_tmp = [];
    
    Pin = [0 0 0];

    Power = [0 0 0];
    Voltage_LMax = [0 0 0];
    
    % Try running the simulation until it succeeds without error or reaches max attempts.
    while ~simulationSuccessful && attemptCount < maxAttempts
        try
            % Update the Simulink model parameters with the current irradiance values.
            set_param([model, '/Irradiance1'], 'Value', num2str(Irradiance1));
            set_param([model, '/Irradiance2'], 'Value', num2str(Irradiance2));
            set_param([model, '/Irradiance3'], 'Value', num2str(Irradiance3));
            
            % Run the simulation and capture the output.
            simOut = sim(model);
            t = simOut.tout;                           % Time data.
            scopeData = simOut.ScopeData.signals;      % Data from ScopeData.

            % ////////////////////////////////////////////////////////////////////
            % Retrieve values.
            Ppv = scopeData(1).values;    % Power values.
            Vpv = scopeData(2).values;    % Voltage values.
            Ipv = scopeData(3).values;    % Current values.
            State = scopeData(4).values;  % State state.

            % Find MPP 1.
            V1 = 0.4 * 12.64;
            I1 = calcI(Vpv, Ipv, V1);
            Pin(1) = V1*I1;
            Index1 = FindIndex(1, 2, State);
            Index2 = FindIndex(2, 3, State);
            if abs(Ppv(Index1) - Ppv(Index2)) < 0.0001
                Power(1) = Ppv(Index1);
                Voltage_LMax(1) = Vpv(Index1);
            else
                Power(1) = Ppv(Index2);
                Voltage_LMax(1) = Vpv(Index2);
            end

            % Find MPP 2.
            V2 = 0.4 + 1 * 12.64;
            I2 = calcI(Vpv, Ipv, V2);
            Pin(2) = V2*I2;
            Index3 = FindIndex(3, 4, State);
            Index4 = FindIndex(4, 5, State);
            if abs(Ppv(Index3) - Ppv(Index4)) < 0.0001
                Power(2) = Ppv(Index3);
                Voltage_LMax(2) = Vpv(Index3);
            else
                Power(2) = Ppv(Index4);
                Voltage_LMax(2) = Vpv(Index4);
            end

            % Find MPP 3.
            V3 = 0.4 + 2 * 12.64;
            I3 = calcI(Vpv, Ipv, V3);
            Pin(3) = V3*I3;
            Index5 = FindIndex(5, 6, State);
            Index6 = FindIndex(6, 7, State);
            if abs(Ppv(Index5) - Ppv(Index6)) < 0.0001
                Power(3) = Ppv(Index5);
                Voltage_LMax(3) = Vpv(Index5);
            else
                Power(3) = Ppv(Index6);
                Voltage_LMax(3) = Vpv(Index6);
            end

            % Combine all relevant data for the current test case.
            data_tmp = [T, Irradiance1, Irradiance2, Irradiance3, V1, V2, V3, I1, I2, I3, Pin(1), Pin(2), Pin(3), Voltage_LMax(1), Voltage_LMax(2), Voltage_LMax(3), Power(1), Power(2), Power(3)];

            % If the simulation is successful, set the flag to true.
            simulationSuccessful = true;

        catch ME
            % If an error occurs, increment the attempt counter and increase Irradiance1.
            warning('Simulation error encountered: %s. Incrementing Irradiance1.', ME.message);

            % Increment the attempt counter and Irradiance1 by 1.
            attemptCount = attemptCount + 1;
            Irradiance1 = Irradiance1 + 1;

            % Optional: Check if we've reached the max number of attempts.
            if attemptCount >= maxAttempts
                warning('Maximum number of attempts reached. Exiting loop.');

                % If simulation fails after trying the maximum number of attempts, store default data.
                data_tmp = [T, Irradiance1, Irradiance2, Irradiance3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
                break;  % Exit the loop after reaching the max attempts.
            end
        end
    end
    % Append the current test case data to the main data array.
    data = [data; data_tmp];
end
end

% Define column headers for the Excel file.
columnHeaders = {
    'Temperature', ...
    'Irradiance1', 'Irradiance2', 'Irradiance3', ...
    'V1', 'V2', 'V3', ...
    'I1', 'I2' , 'I3', ...
    'Pin1', 'Pin2' , 'Pin3', ...
    'VP1', 'VP2', 'VP3', ...
    'P1', 'P2', 'P3'
};

% Save the collected data to a .mat file.
outputFilenameMat = 'Results.mat';
save(outputFilenameMat, 'data');

% Save the collected data to an Excel file.
outputFilenameExcel = 'ST.xlsx';
dataTable = array2table(data, 'VariableNames', columnHeaders);
writetable(dataTable, outputFilenameExcel);

% Close the Simulink model without saving changes.
close_system(model, 0);