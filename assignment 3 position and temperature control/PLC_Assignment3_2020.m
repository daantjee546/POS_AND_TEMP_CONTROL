% ===================================
% PLC Assignments Tuning the Heater / Servo 
% ===================================
% IA-CS4   O.E. Figaroa  
% FHICT - T 
clear vars
close all
% Servo Position Transfer Function
 %num = 20;           % numerator
%den = [0.5 1 0];    % denumerator
% 
% Heater Transfer Function 
num = 5;             % numerator
 den = [256 100 5];   % denumerator

Ts= 0.01;            % sample time 10 ms
Gc=tf(num,den)       % Continuous transfer function of the process
format long
% discretize the process with Ts 
Gd = c2d(Gc,Ts)      % Discrete Transfer function ZOH 
%
% Preparing a Discrete PID with trapezodial I and BackwardEuler D
C01 = pid(1,1,1,'Ts',Ts,'IFormula','Trapezoidal','DFormula','BackwardEuler'); 
%
% Preparing a Discrete PID with Backward Euler I and BackwardEuler D
C02 = pid(1,1,1,'Ts',Ts,'IFormula','BackwardEuler','DFormula','BackwardEuler');
%
% Preparing a Discrete PID with Forward Euler I and ForwardEuler D
%C03 = pid(1,1,1,'Ts',Ts,'IFormula','ForwardEuler','DFormula','ForwardEuler'); % ZOH ; 
%
%
figure(),clf;
step(Gc); % Open loop response
title('Step response Heater.  s-domain');
legend('Open loop response');
ylabel ('Temperature [Celsius]');
grid on
% --
figure(), clf;
step(Gd,'k');       %
title('Step response Heater. z-domain');
ylabel ('Temperature [Celsius]');
grid on
% Following values can be plugged in in simulink or 
% in TwinCAT 3 functionblock:  FB_CTRL_TRANSFERFUNCTION_1
% Check your Project
[numz1,denz1] = tfdata(Gd,'v'); 
hold on
%tf(numz1,denz1)
Contr1 = pidtune(Gd,C01);   %
Contr2 = pidtune(Gd,C02);   %


% Responses of the obtained PID;
% type >> doc feedback  for more information 
step(feedback(Contr1*Gd,1),'r');
step(feedback(Contr2*Gd,1),'b--');

legend('Open-loop ','Trapez:I and BW-Euler:D PID.','BW-Euler:I and BW-Euler:D PID.','Location','east');%
ylabel ('Temperature [Celsius]');

% Display the PID controllers
Contr1
disp('---------------------- ');
Contr2
disp('----------------------- -------------------');
disp('You can als tune your PID by yourself using the following command:');
disp("pidTuner(Gd,'PID')")



