cd(fileparts(mfilename("fullpath")));

theta_limit = pi/2;
volt_limit = 6;
Ts = 0.01;
phi0 = pi;
period = 7;   % match the period the policy was trained with
cycles = 3;   % run cycles

open_system("rotpen_hardware_updown");
