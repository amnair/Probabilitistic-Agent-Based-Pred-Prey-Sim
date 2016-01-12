function [t_pred,t_prey,t,predatorState,preyState,predX,predY,predTheta,preyX,preyY,preyTheta,whichAgent,theta,p] =...
    fixedStepSolver(t_pred,t_prey,t,predatorState,preyState,termConditions,predX,predY,predTheta,preyX,preyY,preyTheta,escape_counter,p)
% Runs a fixed step time solver for when the predator and prey are both moving
%
% t_pred - predator's current elapsed time in state
% t_prey - prey's current elapsed time in state
% t - global time of simulation
% predatorState - current predator state
% preyState - current prey state
% termConditions - a 1x2 array of termination conditions for solver for the predator and prey respectively
% predX - predator's X position
% predY - predator's Y position
% predTheta - predator's orientation angle
% preyX - prey's X position
% preyY - prey's Y position
% preyTheta - prey's orientation angle
% escape_counter - variable that keeps track of how many escapes the prey has made in the whole simulation
% p - input simulation parameters (structure)
% whichAgent - variable determines which agent finished its current state (1 = predator, 2 = prey)
% theta -  angle between the prey and predator respectively

% Initialize parameters
dt = p.param.dt; % Fixed time step
escapeAng = p.prey.escapeAngle(escape_counter); % Array fo potential escape angles for prey
chanceOfErr = p.prey.chanceOfErr(escape_counter); % Array of instances where prey escape in the "right" and "wrong" direction
latency = p.prey.lat; % Prey escape latency
escapeSpeed = p.prey.spdEscape; % Prey escape speed
escape_dur = p.prey.escape_dur(escape_counter); % array of escape durations for the prey
predSpd = p.pred.spd0; % waiting prey speed (which is zero)
whichAgent = 0; % Iniitalize whichAgent variable
gapDistance = []; % Initiatlize distance variable that meeasure gap between both agents
trackingErr = p.pred.posError; % Predator's error in following prey

% Determine which set of teminating conditions are used in solver
if predatorState(end) == 'T' % Use time and distance conditions
    
    % Determine distance between predator and prey
    if t(end) > p.pred.delay && length(t) > 2
        delayedPreyX = interp1(t,preyX,t(end)-p.pred.delay);
        delayedPreyY = interp1(t,preyY,t(end)-p.pred.delay);
        gapDistance = norm([delayedPreyX-predX(end), delayedPreyY-predY(end)]);
    else
        gapDistance = norm([preyX(1)-predX(end), preyY(1)-predY(end)]);
    end
    
    while gapDistance(end) > termConditions(1)  && t_prey(end) < termConditions(2)
        
        % Update time
        t_pred(end+1) = t_pred(end) + dt;
        t_prey(end+1) = t_prey(end) + dt;
        t(end+1) = t(end) + dt;
        
        % Update position of predator
        if t(end-1) > p.pred.delay && length(t) > 2
            delayedPreyX = interp1(t(1:end-1),preyX,t(end-1)-p.pred.delay);
            delayedPreyY = interp1(t(1:end-1),preyY,t(end-1)-p.pred.delay);
            predTheta(end+1) = atan2(delayedPreyY-predY(end),delayedPreyX-predX(end));
        else
            predTheta(end+1) = atan2(preyY(1)-predY(end),preyX(1)-predX(end));
        end
        
        % Add noise to tracking
        [x_comp,y_comp] = pol2cart(predTheta(end),predSpd*dt);
        [xError, yError] = pol2cart(2*pi*rand(1),norm([x_comp y_comp])*trackingErr*rand(1)); % Error in tracking
        predX(end+1) = predX(end) + x_comp + xError;
        predY(end+1) = predY(end) + y_comp + yError;
        % Readjust direction due to error in locomotion
        if t(end-1) > p.pred.delay && length(t) > 2
            delayedPreyX = interp1(t(1:end-1),preyX,t(end-1)-p.pred.delay);
            delayedPreyY = interp1(t(1:end-1),preyY,t(end-1)-p.pred.delay);
            predTheta(end) = atan2(delayedPreyY-predY(end),delayedPreyX-predX(end));
        else
            predTheta(end) = atan2(preyY(1)-predY(end),preyX(1)-predX(end));
        end
        predatorState(end+1) = 'T';
        
        % Update position of prey
        if t_prey(end) > latency && p.prey.fastStarted == 0 % First step of prey escape
            
            % Prey makes turns
            % Find "correct" side
            [xRelPred,~] = coord_trans('global to body',preyTheta(end)-pi/2,[preyX(end) preyY(end)],predX(end),predY(end));
            correctSide = xRelPred/abs(xRelPred); % This is the "correct" direction to escape towards
            preyTheta(end+1) = escapeAng*correctSide*chanceOfErr + preyTheta(end); 
            
            % Update prey position
            spd = speed_func(t_prey(end)-latency, escapeSpeed, escape_dur, latency);
            preyX(end+1) = spd*(t_prey(end)-latency) * cos(preyTheta(end)) + preyX(end);
            preyY(end+1) = spd*(t_prey(end)-latency) * sin(preyTheta(end)) + preyY(end);
            preyState(end+1) = 'E';
            
            p.prey.fastStarted = 1; % Denote that prey has initiated a fast start
            
        elseif t_prey(end) > latency % Second step and onwards of prey escape
            
            % Update prey position
            preyTheta(end+1) =  preyTheta(end);
            spd = speed_func(t_prey(end)-latency, escapeSpeed, escape_dur, latency);
            preyX(end+1) = spd*dt * cos(preyTheta(end)) + preyX(end);
            preyY(end+1) = spd*dt * sin(preyTheta(end)) + preyY(end);
            preyState(end+1) = 'E';
            
        else % Prey isn't doing anything
            
            % Update prey position
            preyTheta(end+1) =  preyTheta(end);
            preyX(end+1) = preyX(end);
            preyY(end+1) = preyY(end);
            preyState(end+1) = 'E';
            
        end
        
        % Update distance between the two agents
        if t(end) > p.pred.delay && length(t) > 2
            delayedPreyX = interp1(t,preyX,t(end)-p.pred.delay);
            delayedPreyY = interp1(t,preyY,t(end)-p.pred.delay);
            gapDistance(end+1) = norm([delayedPreyX-predX(end), delayedPreyY-predY(end)]);
        else
            gapDistance(end+1) = norm([preyX(1)-predX(end), preyY(1)-predY(end)]);
        end
        
    end
    
    % Determine which conidtion ended solver
    if gapDistance(end) <= termConditions(1)
        
        whichAgent = 1; % Predator condition ended the solver
        
    else
        
        whichAgent = 2; % Prey condition ended the solver
        
    end
    
    
    % If terminated with some time left over (round off error of dt), resolve leftover time
    % Determine amount of leftover time
    indexDiff = length(predX) - length(gapDistance);
    lastPredIdx_local = find((gapDistance - termConditions(1))/predSpd > -1e-6,1,'last');
    leftOverTime_pred = (gapDistance(lastPredIdx_local) - termConditions(1))/predSpd;
    lastPredIdx = lastPredIdx_local + indexDiff; % Adjust in array indices
    
    lastPreyIdx_local = find((termConditions(2) - t_prey) > -1e-6,1,'last');
    leftOverTime_prey = termConditions(2) - t_prey(lastPreyIdx_local);
    lastPreyIdx = lastPreyIdx_local + indexDiff; % Adjust in array indices
    
    [leftOverTime,~] = min([leftOverTime_pred leftOverTime_prey]);
    
    if leftOverTime > 1e-5
        
        %Determine which index to use
        if whichAgent == 1
            
            lastIdx = lastPredIdx;
            
        else
            
            lastIdx = lastPreyIdx;
            
        end
        
        % Update position of predator and time vector (orientation constant)
        [x_comp,y_comp] = pol2cart(predTheta(lastIdx),predSpd*leftOverTime);
        predX(lastIdx+1) = predX(lastIdx) + x_comp;
        predY(lastIdx+1) = predY(lastIdx) + y_comp;
        t_pred(lastIdx-indexDiff+1) = t_pred(lastIdx-indexDiff) + leftOverTime;
        
        % Update position of prey and time vector
        spd = speed_func(t_prey(lastIdx-indexDiff)-latency, escapeSpeed, escape_dur, latency);
        preyX(lastIdx+1) = spd*leftOverTime * cos(preyTheta(lastIdx)) + preyX(lastIdx);
        preyY(lastIdx+1) = spd*leftOverTime * sin(preyTheta(lastIdx)) + preyY(lastIdx);
        t_prey(lastIdx-indexDiff+1) = t_prey(lastIdx-indexDiff) + leftOverTime;
        t(lastIdx+1) = t(lastIdx) + leftOverTime;
        
    end
    
    % Determine angle between agents
    if t(end) > p.pred.delay && length(t) > 2
        delayedPreyX = interp1(t,preyX,t(end)-p.pred.delay);
        delayedPreyY = interp1(t,preyY,t(end)-p.pred.delay);
        theta = atan2(delayedPreyY-predY(end),delayedPreyX-predX(end));
    else
        theta = atan2(preyY(1)-predY(end),preyX(1)-predX(end));
    end
    
else % Just use time conditions (predator is in Strike (S or E) state)
    
    while t_pred(end) < termConditions(1) && t_prey(end) < termConditions(2)
        
        % Update time
        t_pred(end+1) = t_pred(end) + dt;
        t_prey(end+1) = t_prey(end) + dt;
        t(end+1) = t(end) + dt;
        
        % Update position of predator (orientation constant)
        predTheta(end+1) = predTheta(end);
        [x_comp,y_comp] = pol2cart(predTheta(end),predSpd*dt);
        predX(end+1) = predX(end) + x_comp;
        predY(end+1) = predY(end) + y_comp;
        predatorState(end+1) = predatorState(end);
        
        % Update position of prey
        if t_prey(end) > latency && p.prey.fastStarted == 0 % First step of prey escape
            
            % Prey makes turns
            % Find "correct" side
            [xRelPred,~] = coord_trans('global to body',preyTheta(end)-pi/2,[preyX(end) preyY(end)],predX(end),predY(end));
            correctSide = xRelPred/abs(xRelPred); % This is the "correct" direction to escape towards
            preyTheta(end+1) = escapeAng*correctSide*chanceOfErr + preyTheta(end);
            
            % Update prey position
            spd = speed_func(t_prey(end)-latency, escapeSpeed, escape_dur, latency);
            preyX(end+1) = spd*(t_prey(end)-latency) * cos(preyTheta(end)) + preyX(end);
            preyY(end+1) = spd*(t_prey(end)-latency) * sin(preyTheta(end)) + preyY(end);
            preyState(end+1) = 'E';
            
            p.prey.fastStarted = 1; % Denote that prey has initiated a fast start
            
        elseif t_prey(end) > latency % Second step and onwards of prey escape
            
            % Update prey position
            preyTheta(end+1) =  preyTheta(end);
            spd = speed_func(t_prey(end)-latency, escapeSpeed, escape_dur, latency);
            preyX(end+1) = spd*dt * cos(preyTheta(end)) + preyX(end);
            preyY(end+1) = spd*dt * sin(preyTheta(end)) + preyY(end);
            preyState(end+1) = 'E';
            
        else % Prey isn't doing anything
            
            % Update prey position
            preyTheta(end+1) =  preyTheta(end);
            preyX(end+1) = preyX(end);
            preyY(end+1) = preyY(end);
            preyState(end+1) = 'E';
            
        end
        
    end
    
    % Determine which conidtion ended solver
    if t_pred(end) >= termConditions(1)
        
        whichAgent = 1; % Predator condition ended the solver
        
    else
        
        whichAgent = 2; % Prey condition ended the solver
        
    end
    
    % If terminated with some time left over (round off error od dt), resolve leftover time
    % Determine amount of leftover time
    indexDiff = length(predX) - length(t_pred);
    lastPredIdx_local = find((termConditions(1) - t_pred) > -1e-6,1,'last');
    leftOverTime_pred = termConditions(1) - t_pred(lastPredIdx_local);
    lastPredIdx = lastPredIdx_local + indexDiff;
    
    lastPreyIdx_local = find((termConditions(2) - t_prey) > -1e-6,1,'last');
    leftOverTime_prey = termConditions(2) - t_prey(lastPreyIdx_local);
    lastPreyIdx = lastPreyIdx_local + indexDiff;
    
    [leftOverTime,~] = min([leftOverTime_pred leftOverTime_prey]);
    
    if leftOverTime > 1e-5
        
        %Determine which index to use
        if whichAgent == 1
            
            lastIdx = lastPredIdx;
            
        else
            
            lastIdx = lastPreyIdx;
            
        end
        
        % Update position of predator and time vector (orientation constant)
        [x_comp,y_comp] = pol2cart(predTheta(lastIdx),predSpd*leftOverTime);
        predX(lastIdx+1) = predX(lastIdx) + x_comp;
        predY(lastIdx+1) = predY(lastIdx) + y_comp;
        t_pred(lastIdx-indexDiff+1) = t_pred(lastIdx-indexDiff) + leftOverTime;
        
        % Update position of prey and time vector
        spd = speed_func(t_prey(lastIdx-indexDiff)-latency, escapeSpeed, escape_dur, latency);
        preyX(lastIdx+1) = spd*leftOverTime * cos(preyTheta(lastIdx)) + preyX(lastIdx);
        preyY(lastIdx+1) = spd*leftOverTime * sin(preyTheta(lastIdx)) + preyY(lastIdx);
        t_prey(lastIdx-indexDiff+1) = t_prey(lastIdx-indexDiff) + leftOverTime;
        t(lastIdx+1) = t(lastIdx) + leftOverTime;
        
    end
    
    theta = []; % No need to determine angle between the predator and prey
    
end

end


function y = speed_func(t, amp, dur, latency)
% Function to determine the speed the prey is escaping at
%
% t - current time in solver
% amp - max speed limit of prey
% dur - duration of escape
% latency - period of time where there is no motion

% Parameters for upward slope
peak = 0.2 * dur;
m1 = 1/peak;
b1 = 1 - m1*(peak-latency);

% Parameters for downward slope
m2 = -1/(dur-peak);
b2 = -m2*(dur-latency);

% Curve describing the heavy tail hill
if t <= (peak-latency)
    k = m1*t + b1;
else
    k = m2*t + b2;
end

y = amp*sin(pi/2*k);

end
