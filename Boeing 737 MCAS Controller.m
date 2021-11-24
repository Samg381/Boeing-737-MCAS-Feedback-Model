% Sam Gregg

clc
clear

syms K3

% Define state-space model
    A = [ -0.0442 18.7 -9.81 0;0 -2.18 0 0.2955;0 0 0 1;0.0079 -78.12 0 -6.08 ];
    B737Max = [ -0.06 20.53;0 0;0 0;-0.9 -3.327 ]; % Boeing 737 Max
    B737    = [ -0.06 11.2;0 0;0 0;-0.9 -1.815 ];  % Boeing 737
    C = [ 1 0 0 0;0 1 0 0;0 0 1 0;0 0 0 1 ];
    D = [ 0 0;0 0;0 0;0 0 ];
    
    APitch737Max = ss(A,B737Max,C,D)        % Boeing 737 Max System
    APitch737 = ss(A,B737,C,D)              % Boeing 737 System
    
    % Define 737 Max MCAS System
    B1 = B737Max(:,1);
    K = [ 0 0 K3 0 ];
    Acl = A-B1*K;

    DC = -inv(Acl)*B737Max;
    DC737Max =DC(3,2);           % DC Gain of closed loop Boeing 737 Max
    
    DC737 = dcgain(APitch737);   % DC Gain of Boeing 737
    DC737 = DC737(3,2);
    
    K3 = double(solve(DC737Max == DC737)); %K3 for equal thrust/pitch angle

    A737MaxCL = double(A-B737Max(:,1)*[0 0 K3 0]);
    A737MaxMCAS = ss(A737MaxCL,B737Max,C,0) % Boeing 737 Max MCAS System

% Plot the step response from the thrust command to the pitch angle 
% response for the 737, 737 Max and 737 Max with MCAS
    subplot(2,1,1)
    s = stepplot(APitch737Max(3,2),APitch737(3,2),A737MaxMCAS(3,2));
    title('Step Response (Pitch Angle from Thrust Command)')
    legend('Boeing 737 Max','Boeing 737', 'Boeing 737 MCAS')
    ylabel('Pitch Angle, Rad')

% Find the rise time, peak time, settling time, and percent overshoot 
% for a unit step using the stepinfo command for all three systems.
    S = stepinfo(APitch737);
    fprintf('Boeing 737 Stepinfo:\n')
    S(3,2)
    S = stepinfo(APitch737Max);
    fprintf('Boeing 737 Max Stepinfo:\n')
    S(3,2)
    S = stepinfo(A737MaxMCAS);
    fprintf('Boeing 737 Max MCAS Stepinfo:\n')
    S(3,2)

% Plot the frequency response from the thrust command to the 
% pitch angle output using the bode command for all three systems
    subplot(2,1,2)
    bode(APitch737(3,2))
    hold on
    bode(APitch737Max(3,2))
    hold on
    bode(A737MaxMCAS(3,2))
    title('Frequency Response (Pitch Angle from Thrust Command)')
    legend('Boeing 737 Max','Boeing 737', 'Boeing 737 MCAS')
    hold off