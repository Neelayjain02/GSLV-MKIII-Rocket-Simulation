function gslv_stage1_fixed
    % GSLV MkIII - Stage 1 Simulation (S200 boosters only)

    %% Physical Constants
    g0   = 9.80665;             % m/s^2, gravity at sea level
    Re   = 6371e3;              % Earth radius (m)
    mu   = 3.986e14;            % Earth gravitational parameter (m^3/s^2)

    %% Rocket Parameters (Stage 1 + full stack mass)
    thrust_total = 10300e3;     % N, total thrust (2x S200 boosters)
    Isp          = 274;         % s, effective ISP of S200 at sea level
    m0           = 640e3;       % kg, liftoff mass (entire vehicle full)
    m_boosters   = 237e3*2;     % kg, propellant mass in both boosters
    burn_time    = 130;         % s, booster burn duration

    mdot = thrust_total/(Isp*g0);   % mass flow rate (kg/s)

    %% Atmosphere Model (ISA, simplified)
    function [rho] = atmosphere(h)
        if h < 0
            h = 0;
        end
        if h < 11000
            T = 288.15 - 0.0065*h; % Troposphere
            p = 101325 * (T/288.15)^(-g0/(0.0065*287));
        elseif h < 25000
            T = 216.65;
            p = 22632 * exp(-g0*(h-11000)/(287*T));
        else
            T = 216.65 + 0.003*(h-25000);
            p = 2488 * (T/216.65)^(-g0/(0.003*287));
        end
        rho = p/(287*T);
    end

    %% Initial State
    h0 = 0;                  % altitude (m)
    v0 = 0;                  % velocity (m/s)
    gamma0 = 90*pi/180;      % flight path angle (rad) - vertical
    x0 = 0;                  % downrange distance
    m_init = m0;

    state0 = [h0; v0; gamma0; x0; m_init];

    %% Simulation Time
    tspan = [0 burn_time];

    %% Solve ODE
    opts = odeset('RelTol',1e-6,'AbsTol',1e-9);
    [t, Y] = ode45(@eom, tspan, state0, opts);

    h = Y(:,1);
    v = Y(:,2);
    gamma = Y(:,3);
    x = Y(:,4)/1000;   % km
    m = Y(:,5);

    %% Plots
    figure;
    subplot(3,1,1); plot(t,h/1000,'LineWidth',2); grid on;
    ylabel('Altitude (km)'); title('GSLV MkIII Stage-1 Simulation');
    
    subplot(3,1,2); plot(t,v/1000,'LineWidth',2); grid on;
    ylabel('Velocity (km/s)');
    
    subplot(3,1,3); plot(x,h/1000,'LineWidth',2); grid on;
    xlabel('Downrange (km)'); ylabel('Altitude (km)');
    title('Trajectory');

    %% Equations of Motion
    function dY = eom(t, Y)
        h     = Y(1);
        v     = Y(2);
        gamma = Y(3);
        x     = Y(4);
        m     = Y(5);

        if h < 0
            h = 0;
        end

        r = Re + h;
        g = mu/r^2;

        rho = atmosphere(h);
        Cd = 0.5;     % drag coefficient
        A  = 10;      % m^2, reference area
        D  = 0.5 * rho * v^2 * Cd * A;

        % Thrust only while propellant remains
        if m > m0 - m_boosters
            T = thrust_total;
            md = mdot;
        else
            T = 0;
            md = 0;
        end

        % Gravity turn (start after 10s)
        if t < 10
            gamma_dot = 0;
        else
            gamma_dot = -0.25*pi/180;  % rad/s pitch rate
        end

        % Prevent divide by zero
        if v < 1
            v = 1;
        end

        % Equations
        dhdt = v*sin(gamma);
        dvdt = (T - D)/m - g*sin(gamma);
        dgdt = (-g/v)*cos(gamma) + gamma_dot;
        dxdt = v*cos(gamma);
        dmdt = -md;

        dY = [dhdt; dvdt; dgdt; dxdt; dmdt];
    end
end
