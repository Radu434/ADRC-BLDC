% Machine Parameters
p    = 4;              % Number of pole pairs
Rs   = 0.1;            % Stator resistance per phase           [Ohm]
Ls   = 8e-5;           % Stator self-inductance per phase, Ls  [H]
Ms   = 1e-5;           % Stator mutual inductance, Ms          [H]
psim = 0.0175;         % Maximum permanent magnet flux linkage [Wb]
Jm   = 0.002;          % Rotor inertia                         [Kg*m^2]

%% Control Parameters
Ts  = 5e-6;     % Fundamental sample time            [s]
Tsc = 1e-4;     % Sample time for inner control loop [s]
Vdc = 48;       % Maximum DC link voltage            [V]


% Tuning parameters
b0 = 50;         % Critical gain parameter   

wCL = 2*pi*40150; % Closed-Loop Bandwidth

kESO = 5;        % Observer bandwidth factor


% Discrete-time controller/observer gains
zCL = exp(-wCL*Ts); % Closed-loop pole in the z-domain.

zESO = exp(-kESO*wCL*Ts);

K = [(1-zCL)^2/Ts^2, (4-(1+zCL)^2)/(2*Ts)]; % Controller gains for tracking reference

L = [(1-zESO^3);
    3/(2*Ts)*(1-zESO)^2*(1+zESO); 
    1/Ts^2*(1-zESO)^3]; % Observer gains for estimating state and disturbance


% ESO matrix/vector A and b for N = 2
A_ESO = [1-L(1), Ts-L(1)*Ts, 1/2*Ts^2-1/2*L(1)*Ts^2; 
-L(2), 1-L(2)*Ts, Ts-1/2*L(2)*Ts^2; 
-L(3), -L(3)*Ts, 1-1/2*L(3)*Ts^2];

B_ESO = [1/2*b0*Ts^2 - 1/2*L(1)*b0*Ts^2; 
b0*Ts-1/2*L(2)*b0*Ts^2; 
-1/2*L(3)*b0*Ts^2];
