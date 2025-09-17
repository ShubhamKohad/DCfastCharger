%%Rectifier parameters
rectifier.ACVoltagePP = 415; % V % RMS value of Phase- phase voltage
rectifier.ACVoltagePN = rectifier.ACVoltagePP/sqrt(3); % V % RMS value of Phase- phase voltage
rectifier.ACVoltagePeak = rectifier.ACVoltagePN * sqrt(2); % V 
rectifier.DCCurrent = 700; % A
rectifier.DCVoltage = 800; % V

rectifier.SystemFrequency = 50; % Hz
rectifier.SwitchFrequency = 10e3; % Hz

rectifier.minVdcPossible = rectifier.ACVoltagePP*sqrt(2/3)/0.5; % V
rectifier.acCurrent = sqrt(2)*rectifier.DCCurrent*rectifier.DCVoltage/(sqrt(3)*rectifier.ACVoltagePP); % A

rectifier.maxLVal = 0.95*((rectifier.DCVoltage*0.5)-rectifier.ACVoltagePP*sqrt(2/3))/(2*pi*rectifier.SystemFrequency*rectifier.acCurrent); % H
rectifier.maxACcurrent = 100; % A
rectifier.minACcurrent = -100; % A
rectifier.maxACVoltage = 515; % V
rectifier.minACVoltage = -515; % V
rectifier.minDCVoltage = 0.5*rectifier.DCVoltage; % V

rectifier.lineInductance = 0.1e-3; % H
rectifier.lineResistance = 20e-3; % ohm
rectifier.lineT = rectifier.lineInductance/rectifier.lineResistance; % s 
rectifier.OutputCapacitance = 20e-3; % F
rectifier.a = 2; % constant

rectifier.VoltageSensorG = 1; % constant
rectifier.VoltageSensorT = 1/(10*rectifier.SwitchFrequency); % s
rectifier.CurrentSensorG = 1; % constant
rectifier.CurrentSensorT = 1/(10*rectifier.SwitchFrequency); % s
rectifier.G = rectifier.DCVoltage/2; % constant
rectifier.K = rectifier.ACVoltagePeak/rectifier.DCVoltage; % constant
rectifier.Td = 1/(2*rectifier.SwitchFrequency); % s
rectifier.Tphi = rectifier.Td + rectifier.CurrentSensorT; % s
rectifier.Tdel = (2*rectifier.Tphi) + rectifier.VoltageSensorT; % s

rectifier.controller.CurrentG = rectifier.lineInductance/...
    (2*rectifier.G*rectifier.CurrentSensorG*rectifier.Tphi); % constant
rectifier.controller.CurrentT = rectifier.lineT; % s
rectifier.controller.VoltageG = (rectifier.OutputCapacitance...
    *rectifier.CurrentSensorG)/...
    (rectifier.K*2*rectifier.VoltageSensorG*rectifier.Tdel); % constant
rectifier.controller.VoltageT = 4*rectifier.Tdel; % s
%%
%%DC-DC converter parameters
inverter.SwitchFrequency = 10e3; % Hz
inverter.controller.kp = 2; % constant
inverter.controller.ki = 1; % constant
inverter.inductance = 10e-6; % H
transformer.magnetizingL = 1; % H
transformer.windingFactor = 0.5; % constant
chopper.VoltageSensorG = 1; % constant
chopper.VoltageSensorT = 1/(10*inverter.SwitchFrequency); % s
chopper.CurrentSensorG = 1; % constant
chopper.CurrentSensorT = 1/(10*inverter.SwitchFrequency); % s
%%
%% batterypack parameters
battery.currentReference = 100; % A
battery.initialSOC = 0.2; % constant
battery.AHRating = 50; % Ah
battery.inductance = 5e-3; % H
battery.cellsInSeries = 100; % constant
battery.batteryStringsInParallel = 1; % constant
%%
% 1. Average - Used when model fidelity level do not require switching.
%    This is mainly used in the linearization of the solar PV system.
% Variant name/expression - Average
% Variant Condition - powerCircuit == 0
average=Simulink.Variant(' powerCircuit == 0 ');
% 2. Two level three-phase inverter
% Variant name/expression - Two Level
% Variant Condition - powerCircuit == 1
twoLevel=Simulink.Variant(' powerCircuit == 1 ');
% 3. Three Level three-phase inverter
% Variant name/expression - discrete
% Variant Condition - powerCircuit == 2
threeLevel=Simulink.Variant(' powerCircuit == 2 ');
% Variable powerCircuit = 0 for Average, powerCircuit = 1 for Two Level
% and powerCircuit = 2 for Three Level inverter

% Select the Required Power circuit
powerCircuit=0;
%%
simulation.numberOfCycles = 10; % constant
simulation.simTime = simulation.numberOfCycles/rectifier.SystemFrequency; % s