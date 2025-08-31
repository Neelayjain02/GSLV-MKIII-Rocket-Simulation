%% GSLV MkIII - Stage 3 Simulation (CE-20 Cryogenic Stage) with Cutoff & Pitch Program

clear; clc; close all;

%% Constants
Re = 6371e3;         % Earth radius (m)
g0 = 9.80665;        % Gravity (m/s^2)
rho0 = 1.225;        % Sea-level density (kg/m^3)
H = 8500;            % Scale height (m)

%% Stage-3 Parameters
Isp = 443;            % Specific impulse (s)
thrust = 200e3;       % Vacuum thrust (N)
m_prop = 27000;       % Propellant mass (kg)
m_dry  = 5000;        % Dry mass (kg)
m0_stage3 = m_prop + m_dry; 
mf_stage3 = m_dry;
burn_time = 720;      % Max burn duration (s)

% Effective exhaust velocity
ve = Isp * g0;

% Mass flow rate
mdot = (m0_stage3 - mf_stage3) / burn_time;

%% Initial conditions from Stage-2 cutoff
alt0 = 150.68e3;         % m
vel0 = 6.12e3;           % m/s
downrange0 = 665.63e3;   % m
gamma0 = 3.38 * pi/180;  % rad

% State vector [x_downrange, h_alt, v, gamma]
y0 = [downrange0, alt0, vel0, gamma0];

%% Simulation
tspan = [0 burn_time];
opts = odeset('RelTol',1e-6,'AbsTol',1e-6);

[t,Y] = ode45(@(t,y) eom_stage3_cutoff(t,y,thrust,ve,mdot,m0_stage3,Re,g0,rho0,H,burn_time), ...
              tspan, y0, opts);

%% Extract results
downrange = Y(:,1)/1e3; % km
alt = Y(:,2)/1e3;       % km
vel = Y(:,3)/1e3;       % km/s
gamma = Y(:,4)*180/pi;  % deg

%% Plots
figure;
subplot(3,1,1);
plot(t,alt,'LineWidth',1.5);
xlabel('Time (s)'); ylabel('Altitude (km)');
title('GSLV MkIII Stage-3 Simulation'); grid on;

subplot(3,1,2);
plot(t,vel,'LineWidth',1.5);
xlabel('Time (s)'); ylabel('Velocity (km/s)'); grid on;

subplot(3,1,3);
plot(downrange,alt,'LineWidth',1.5);
xlabel('Downrange (km)'); ylabel('Altitude (km)');
title('Trajectory'); grid on;

%% Print final results
alt_final = alt(end);
vel_final = vel(end);
gamma_final = gamma(end);
fprintf('--- Stage 3 Final Conditions ---\n');
fprintf('Altitude      = %.2f km\n', alt_final);
fprintf('Velocity      = %.2f km/s\n', vel_final);
fprintf('Downrange     = %.2f km\n', downrange(end));
fprintf('Flight Angle  = %.2f deg\n', gamma_final);

%% --- Dynamics function with cutoff and pitch program ---
function dydt = eom_stage3_cutoff(t,y,thrust,ve,mdot,m0,Re,g0,rho0,H,burn_time)

    % Extract states
    x = y(1);   % downrange (m)
    h = y(2);   % altitude (m)
    v = y(3);   % velocity (m/s)
    gamma = y(4); % flight path angle (rad)

    r = Re + h;

    % Mass depletion
    m = max(m0 - mdot*t, m0 - mdot*burn_time); % never below dry mass

    % Gravity
    g = g0*(Re^2)/(r^2);

    % Atmosphere (small drag, negligible at high altitudes)
    rho = rho0 * exp(-h/H);
    Cd = 0.3; A = 4;
    D = 0.5 * rho * v^2 * Cd * A;

    % --- Cutoff condition ---
    v_target = 10.2e3;   % target velocity for GTO (m/s)
    h_min = 180e3;       % min altitude at cutoff
    if v >= v_target && h >= h_min
        thrust_eff = 0;  % engine cutoff
    else
        thrust_eff = thrust;
    end

    % --- Simple pitch program ---
    gamma0 = 3.38*pi/180; % initial flight path angle
    gamma_target = 0;     % target angle at burnout
    gamma_dot = (gamma_target - gamma0)/burn_time; % rad/s
    gamma = gamma + gamma_dot * t; % approximate gradual tilt

    % Equations of motion
    dxdt = v * cos(gamma);
    dhdt = v * sin(gamma);
    dvdt = thrust_eff/m - g*sin(gamma) - D/m;
    dgamdt = -(g/v)*cos(gamma) + (v/r)*cos(gamma);

    dydt = [dxdt; dhdt; dvdt; dgamdt];
end
