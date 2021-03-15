% update problem configuration

% run this script from 'main' folder

%%
% delete old problem config
if isfile('../database/problem_config.mat')
    delete('../database/problem_config.mat');
end

% find directory of parent folder
current_dir = pwd;
sep = strfind(pwd,filesep);
parent_dir = current_dir(1:sep(end)-1);

src_dir = [parent_dir, '/src'];
database_dir = [parent_dir, '/src'];

%% Set default values for parameters
k = 3; % number of edges for approximating the friction cone
f_mu = 0.5; % coefficient of friction
f_gamma = 1.0; % coefficient of torsinal friction

% Approximated friction cone of finger contacts (contact normal: +Y direction)
S = [f_mu*cos(2*pi*[1:k]/k);... % approximation of friction cone
    ones(1,k);... % central axis of cone: pointing in the y+ direction
    f_mu*sin(2*pi*[1:k]/k)];

% Approximated friction cone of palm contacts (contact normal: -Z direction)
Sp = [f_mu*cos(2*pi*[1:k]/k);... % approximation of friction cone
    f_mu*sin(2*pi*[1:k]/k);...
    -ones(1,k)]; % this cone is the torsinal torque, so the central axis, Y, of the S, corresponds to a central axis in -Z, of Sp

epsilon = 1e-6; % numerical round-off error tolerance

%% Configuration for optimization
max_fun_evals = 100000;
max_iter = 5000;
tol_fun = 1e-4;
tol_x = 1e-6;

%% Select constraints
cstr.fc = true; % use force closure constraints

save([database_dir, '/problem_config.mat']);
disp('* Updated problem config saved.');
