%% GSLV MkIII - Stage 2 Simulation (L110 Core Stage)

clear; clc; close all;

%% Constants
Re = 6371e3;         % Earth radius (m)
g0 = 9.80665;        % Gravity (m/s^2)
rho0 = 1.225;        % Sea-level air density (kg/m^3)
H = 8500;            % Scale height (m)

%% Stage-2 Parameters (L110 core stage - Vikas engines)
Isp = 293;           % sec (vacuum ~ 293 s)
thrust = 1600e3;     % N (vacuum thrust, twin Vikas)
m0_stage2 = 119e3;   % kg (with propellant)
mf_stage2 = 29e3;    % kg (dry mass)
burn_time = 150;     % s

% Effective exhaust velocity
ve = Isp * g0;

% Mass flow rate
mdot = (m0_stage2 - mf_stage2) / burn_time;

%% Initial conditions from Stage-1 output
alt0 = 62e3;           % 62 km
vel0 = 2.6e3;          % 2.6 km/s
downrange0 = 90e3;     % 90 km
gamma0 = 20 * pi/180;  % ~20 deg

% State vector [x_downrange, h_alt, v, gamma]
y0 = [downrange0, alt0, vel0, gamma0];

%% Simulation time
tspan = [0 burn_time];

%% Dynamics
opts = odeset('RelTol',1e-6,'AbsTol',1e-6);

[t, Y] = ode45(@(t,y) eom_stage2(t,y,thrust,mdot,m0_stage2,mf_stage2,Re,g0,rho0,H), tspan, y0, opts);

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
title('GSLV MkIII Stage-2 (L110) Simulation'); grid on;

subplot(3,1,2);
plot(t,vel,'LineWidth',1.5);
xlabel('Time (s)'); ylabel('Velocity (km/s)'); grid on;

subplot(3,1,3);
plot(downrange,alt,'LineWidth',1.5);
xlabel('Downrange (km)'); ylabel('Altitude (km)');
title('Trajectory'); grid on;

%% --- Dynamics function ---
function dydt = eom_stage2(t,y,thrust,mdot,m0,mf,Re,g0,rho0,H)

    x = y(1);   % downrange (m)
    h = y(2);   % altitude (m)
    v = y(3);   % velocity (m/s)
    gamma = y(4); % flight path angle (rad)

    r = Re + h;

    % Mass depletion
    m = m0 - mdot*t;
    if m < mf
        m = mf;  % lock at dry mass after burnout
        thrust = 0; % no more thrust after prop burnt
    end

    % Gravity
    g = g0*(Re^2)/(r^2);

    % Atmosphere (drag)
    rho = rho0 * exp(-h/H);
    Cd = 0.4; A = 10; % assumed values
    D = 0.5 * rho * v^2 * Cd * A;

    % Equations of motion
    dxdt = v * cos(gamma);
    dhdt = v * sin(gamma);
    dvdt = (thrust - D)/m - g*sin(gamma);
    dgamdt = -(g/v)*cos(gamma) + (v/r)*cos(gamma);

    dydt = [dxdt; dhdt; dvdt; dgamdt];
end

%% Extract final conditions for Stage-3 initialization
alt_stage2_end      = Y(end,2);       % Final altitude (m)
vel_stage2_end      = Y(end,3);       % Final velocity (m/s)
downrange_stage2_end= Y(end,1);       % Final downrange (m)
gamma_stage2_end    = Y(end,4);       % Final flight path angle (rad)

% Remaining mass after stage 2
m_stage2_end = m0_stage2 - mdot * t(end);

% Display results
fprintf('--- Stage 2 Final Conditions ---\n');
fprintf('Altitude      = %.2f km\n', alt_stage2_end/1e3);
fprintf('Velocity      = %.2f km/s\n', vel_stage2_end/1e3);
fprintf('Downrange     = %.2f km\n', downrange_stage2_end/1e3);
fprintf('Flight Angle  = %.2f deg\n', gamma_stage2_end*180/pi);
fprintf('Final Mass    = %.2f kg\n', m_stage2_end);
