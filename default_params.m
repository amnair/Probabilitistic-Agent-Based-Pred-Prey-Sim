function [p,d] = default_params
% Default parameter values for the predator and prey


%% General parameters
%Time span of simulation
p.param.t_span = [0 10];
d.param.t_span      = 'T';

% Fixed time step for solver
p.param.dt = 0.005; % s
d.param.dt = 'T';

% Scaling constants (ignore this part)
p.param.sL      = 1;   % m
d.param.sL      = 'L';

p.param.sT      = 1;      % s
d.param.sT      = 'T';

%% Prey parameters

% Initial body position & orientation
r = 0.085*rand(1);
th = 2*pi*rand(1);
[x0, y0] = pol2cart(th,r);
p.prey.x0           = x0;      % m
d.prey.x0           = 'L';

p.prey.y0           = y0;      % m
d.prey.y0           = 'L';

p.prey.theta0         = 2*pi*rand(1); % rad
d.prey.theta0         = '';

% Initial speed
p.prey.spd0           = 0e-2;    % m/s        PREY STAY STATIONARY WHEN THEY ARE NOT FAST STARTING
d.prey.spd0           = 'L/T';

% Escape parameters
% threshold distance for escape initiation (using a lognormal distribution)
p.prey.thrshEscape_mu    = -4.54594;        % m     Gotten from 73 samples of evasions. This abstracts the latency to respond
p.prey.thrshEscape_sigma    = 0.587484;
d.prey.thrshEscape_mu    = 'L';
d.prey.thrshEscape_sigma    = 'L';
d.prey.escape_threshold    = 'L';

% speed during escape response
p.prey.spdEscape      = 40e-2;       % m/s
d.prey.spdEscape      = 'L/T';

% Escape angle during escape (using a Burr distribution)
p.prey.escapeAngle_alpha   = 2.00143;   % rad    Taken from Kelsey's robopredator, 206 samples
p.prey.escapeAngle_c   = 3.38972;
p.prey.escapeAngle_k   = 4.04469;
d.prey.escapeAngle_alpha   = '';
d.prey.escapeAngle_c   = '';
d.prey.escapeAngle_k   = '';
d.prey.escapeAngle = '';

% response latency
p.prey.lat            = 8e-3;       % s     This is the duration of stage 1 where there is no kinematic displacement. Taken from 3D kinematic escape responses
d.prey.lat            = 'T';

% time to complete turn during escape (using a lognormal distribution)
p.prey.durEscape_mu      = -1.36936;      % s      Sampled over 63 strikes (doesn't include stage 1 which is estimated to be 8 ms)
p.prey.durEscape_sigma      = 0.552332;
d.prey.durEscape_mu      = 'T';
d.prey.durEscape_sigma      = 'T';
d.prey.escape_dur = 'T';

% Units for the random direction of escape
d.prey.chanceOfErr = '';



%% Predator parameters

% Initial body position & orientation
r = 0.085*rand(1);
th = 2*pi*rand(1);
[x0, y0] = pol2cart(th,r);
p.pred.x0             = x0;         % m
d.pred.x0             = 'L';

p.pred.y0               = y0;         % m
d.pred.y0               = 'L';

% Predator is always facing the prey
p.pred.theta0           = atan2((p.prey.y0-p.pred.y0), (p.prey.x0-p.pred.x0));      % rad  ALWAYS FACING PREY
d.pred.theta0           = '';

% Initial speed
p.pred.spd0             = 13e-2;      % m/s AVERAGE OF ALL THUNDERDOME EXPERIMENTS
d.pred.spd0             = 'L/T';

% Distance from prey for initiating a strike (using a burr distribution)
p.pred.strike_thresh_alpha  = 0.0096102;  % m        VALUE FROM THUNDERDOME; from 51 samples
p.pred.strike_thresh_c  = 3.34016;
p.pred.strike_thresh_k  = 2.19229;
d.pred.strike_thresh_alpha  = 'L';
d.pred.strike_thresh_c  = 'L';
d.pred.strike_thresh_k  = 'L';
d.pred.strike_threshold = 'L';

% Strike duration (using a lognormal distribution)
p.pred.strike_dur_mu     = -3.16594; % s        VALUE FROM THUNDERDOME;from >60 samples
p.pred.strike_dur_sigma     = 0.330798;
d.pred.strike_dur_mu     = 'T';
d.pred.strike_dur_sigma     = 'T';
d.pred.strike_duration = 'T';

% Sample rate of visual system
p.pred.vis_freq = 10; % 1/s
d.pred.vis_freq = '1/T';

% Probability to escape suction field of predator (described by a 3rd order polynomial)
p.pred.capture_coeff = [-1.056e6 3.036e4 -283.1 1.018]; 
d.pred.capture_coeff = '';

% Range of position error in tracking
p.pred.posError = 0.01; % percent (in ratio form) of locomotion
d.pred.posError = '';

% Delay in tracking for the predator 
p.pred.delay = 0.01; % s
d.pred.delay = 'T';


