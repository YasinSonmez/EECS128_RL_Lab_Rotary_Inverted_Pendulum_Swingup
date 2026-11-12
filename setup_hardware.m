%% Hardware model setup
% Run this script on the lab computer before opening or running the hardware model.

theta_limit = pi/2;
volt_limit = 6;
Ts = 0.01;
phi0 = pi;

open_system("rotpen_hardware");
