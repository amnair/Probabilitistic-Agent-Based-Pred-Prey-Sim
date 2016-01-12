function R = simple_model(pIn, d)
% runs an instance of the predator prey simulation based off its inputs
%
% pIn - input parameters (structure)
% d - units/scaling parameters
% R - output parameters of simulation (structure)


%% Parameters

% Scaling constants (ignore this part)
sT = pIn.param.sT;
sL = pIn.param.sL;

% Convert parameter values, according to scaling constants (ignore this part)
p = convert_params(pIn,d);

% Keeping track of what number strike the predator is on
strike_counter = 1;

% Keeping track of what number escape the prey is on
escape_counter = 1;

% Set up simulation variables
t = 0;
predX = p.pred.x0;
predY = p.pred.y0;
predTheta = p.pred.theta0;
preyX = p.prey.x0;
preyY = p.prey.y0;
preyTheta = p.prey.theta0;
p.prey.fastStarted = 0; % Keeps track whether the prey committed to an escape or not during an instance of the escape state
captured = 0; % Simulation termination tracking variable

%Starting predator and prey states
predatorState = 'T'; % T = tracking; S = Start of Striking; Es = End of Striking
preyState = 'W'; % W = Waiting; E = Escaping

% Time tracking variables for elapsed time for each agent
preyElapsedTime = 0;
predatorElapsedTime = 0;

%% Run simulation

% Loop through simulation until prey is captured or the predator has
%striked more than 20 times or the prey has escaped 20 times
while captured == 0 && (strike_counter <= 20 && escape_counter <= 20)
    
    % Total distance between predator and prey
    % Delayed prey coordinates
    delayedPreyX = [];
    delayedPreyY = [];
    if t(end) > p.pred.delay && length(t) > 2 % If there are not enough time points...
        delayedPreyX = interp1(t,preyX,t(end)-p.pred.delay);
        delayedPreyY = interp1(t,preyY,t(end)-p.pred.delay);
        dist = norm([delayedPreyX-predX(end), delayedPreyY-predY(end)]);
    else
        dist = norm([preyX(1)-predX(end), preyY(1)-predY(end)]);
    end
    
    % Determine upcoming time event for predator (When it transitions to new state)
    if predatorState(end) == 'T' % If predator is currently tracking
        
        predEvent = (dist - p.pred.strike_threshold(strike_counter))/p.pred.spd0; % Time to strike assuming prey stays still
        
    elseif predatorState(end) == 'S' % If predator is starting to strike
        
        predEvent = p.pred.strike_threshold(strike_counter)/2; % The next transition is in the middle of th whole strike (S + E). Max mouth gape
        
    else % If predator is ending the strike
        
        predEvent = p.pred.strike_threshold(strike_counter)/2 ; % Wrapping up the next half of the strike
        
    end
    
    % Determine upcoming time event for prey (When it transitions to new state)
    if preyState(end) == 'W'
        
        preyEvent = (dist - p.prey.escape_threshold(escape_counter))/p.pred.spd0; % Time to escape assuming predator is swimming straight to prey
        
    else % Prey is escaping
        
        preyEvent = -100; % -100 will initiate a fixed step solver to take place
        
    end
    
    % If there are negative time values or a time value close to zero, adjust predator state
    if predEvent < 1e-10
        
        predatorState(end) = 'S'; % initiate a strike since already within threshold (or really close to)
        predEvent = p.pred.strike_threshold(strike_counter)/2;
        
    end
    
    % If there are negative time values (not -100) or a time value close to zero, adjust prey state
    if preyEvent < 0 && preyEvent ~= -100
        
        preyState(end) = 'E';
        preyEvent = -100;
        
    end
    
    
    %Determine closest time event between predator and prey
    if predatorState(end) == 'S' || predatorState(end) == 'E'
        pendingEvents = [predEvent-predatorElapsedTime preyEvent];
    else
        pendingEvents = [predEvent preyEvent];
    end
    
    % Choose soonest event in and determine which agent (predator or prey) will transition states
    if diff(pendingEvents) == 0 % Events happen at the same time!
        
        closestEvent = 100;
        
    else
        
        [closestEvent,whichAgent] = min(pendingEvents); % if closestEvent == -100, switch into fixed time step solver
        
    end
    
    % Resolve transition in states
    if closestEvent == -100 % Resolve fixed time step solver
        
        %Set up termination conditions for solver
        % Termination condition for predator
        if predatorState(end) == 'T'
            
            termConditions(1) = p.pred.strike_threshold(strike_counter);
            
        else
            
            termConditions(1) = predEvent;
            
        end
        
        % Termination condition for prey
        termConditions(2) = p.prey.escape_dur(escape_counter);
        
        % Run fixed step solver
        [t_pred,t_prey,t,predatorState,preyState,predX,predY,predTheta,preyX,...
            preyY,preyTheta,whichAgent,th,p] = fixedStepSolver...
            (predatorElapsedTime,preyElapsedTime,t,predatorState,preyState,...
            termConditions,predX,predY,predTheta,preyX,preyY,preyTheta,escape_counter,p);
        
        % Update elapsed time and agent state
        if whichAgent == 1
            
            % Update predator State
            if predatorState(end) == 'T'
                
                % Transition into strike mode only if prey is in capture zone
                if th-predTheta(end) >= -pi/2 && th-predTheta(end) <= pi/2
                    predatorState(end) = 'S';
                end
                
            elseif predatorState(end) == 'S'
                
                % Check if prey is captured
                dist = norm([preyX(end)-predX(end), preyY(end)-predY(end)]);
                if dist <= 0.01
                    probCapture = polyval(p.pred.capture_coeff,dist);
                    randNum = rand(1);
                    
                    if randNum <= probCapture
                        captured = 1;
                    end
                end
                
                predatorState(end) = 'E';
                
            else
                
                predatorState(end) = 'T';
                strike_counter = strike_counter + 1;
                
            end
            
            % Update elapsed time
            predatorElapsedTime = 0;
            preyElapsedTime = t_prey(end);
            
        else
            
            % Update prey state
            preyState(end) = 'W';
            escape_counter = escape_counter + 1;
            p.prey.fastStarted = 0;
            
            % Update elapsed time
            predatorElapsedTime = t_pred(end);
            preyElapsedTime = 0;
            
        end
        
    elseif closestEvent == 100 % Resolve both events simultaneously
        
        switch predatorState(end)
            
            case 'T' % Simultaneously Resolve transitions T->S (predator) and W->E (prey)
                
                % Creating input variables
                eventData = {p.pred.delay,p.pred.strike_threshold(strike_counter),p.pred.posError};
                
                % Resolve predator state
                [t,predX,predY,predTheta,preyX,preyY,preyTheta,...
                    predatorState,preyState] = resolvePredState...
                    (t,closestEvent,predX,predY,predTheta,preyX,preyY,preyTheta,...
                    predatorState,preyState,eventData);
                
                % Update prey state
                preyState(end+1) = 'E';
                
            case 'S' % Simultaneously Resolve transitions S->E (predator) and W->E (prey)
                
                % Creating input variable
                eventData = {p.pred.spd0 * closestEvent};
                
                % Resolve predator state
                [t,predX,predY,predTheta,preyX,preyY,preyTheta,...
                    predatorState,preyState,checkCaptured] = resolvePredState...
                    (t,closestEvent,predX,predY,predTheta,preyX,preyY,preyTheta,predatorState,preyState,eventData);
                
                % Check if prey is captured
                if checkCaptured == 1
                    dist = norm([preyX(end)-predX(end), preyY(end)-predY(end)]);
                    probCapture = polyval(p.pred.capture_coeff,dist);
                    randNum = rand(1);
                    if randNum <= probCapture
                        captured = 1;
                    end
                end
                
                % Update prey state
                preyState(end+1) = 'E';
                
            case 'E' % Simultaneously Resolve transitions S->T (predator) and W->E (prey)
                
                % Creating input variables
                eventData = {p.pred.spd0 * closestEvent};
                
                % Resolve predator state
                [t,predX,predY,predTheta,preyX,preyY,preyTheta,...
                    predatorState,preyState] = resolvePredState...
                    (t,closestEvent,predX,predY,predTheta,preyX,preyY,...
                    preyTheta,predatorState,preyState,eventData);
                
                % Update prey state
                preyState(end+1) = 'E';
                
        end
        
    else % Resolve one of the agents (whichever came sooner)
        
        if whichAgent == 1 % Predator State resolves
            
            % Save elapsed for prey
            predatorElapsedTime = 0;
            preyElapsedTime = closestEvent;
            
            switch predatorState(end)
                
                case'T' % Resolve the transition from targeting state when the prey is waiting
                    
                    % Creating input variables
                    eventData = {p.pred.delay,p.pred.strike_threshold(strike_counter),p.pred.posError};
                    
                    % Resolve predator state
                    [t,predX,predY,predTheta,preyX,preyY,preyTheta,...
                        predatorState,preyState] = resolvePredState...
                        (t,closestEvent,predX,predY,predTheta,preyX,preyY,preyTheta,...
                        predatorState,preyState,eventData);
                    
                case 'S' % Resolve the transition from strike state to capture while prey is waiting
                    
                    % Creating input variable
                    eventData = {p.pred.spd0 * closestEvent};
                    
                    [t,predX,predY,predTheta,preyX,preyY,preyTheta,...
                        predatorState,preyState,checkCaptured] = resolvePredState...
                        (t,closestEvent,predX,predY,predTheta,preyX,preyY,preyTheta,predatorState,preyState,eventData);
                    
                    % Check if prey is captured
                    if checkCaptured == 1
                        dist = norm([preyX(end)-predX(end), preyY(end)-predY(end)]);
                        probCapture = polyval(p.pred.capture_coeff,dist);
                        randNum = rand(1);
                        if randNum <= probCapture
                            captured = 1;
                        end
                    end
                    
                case 'E' % Resolve the transition from Strike state to targeting when the prey is waiting
                    
                    % Creating input variables
                    eventData = {p.pred.spd0 * closestEvent};
                    
                    % Resolve predator's state
                    [t,predX,predY,predTheta,preyX,preyY,preyTheta,...
                        predatorState,preyState] = resolvePredState...
                        (t,closestEvent,predX,predY,predTheta,preyX,preyY,...
                        preyTheta,predatorState,preyState,eventData);
                    
                    % Update the strike counter
                    strike_counter = strike_counter + 1;
                    
            end
            
        else
            
            % Save elapsed for predator
            predatorElapsedTime = closestEvent;
            preyElapsedTime = 0;
            
            if preyState(end) == 'W' % Transition from waiting to escaping
                
                % Creating input variables
                eventData = {predatorState(end),p.prey.escape_threshold(escape_counter),p.pred.spd0*closestEvent};
                
                % Resolve prey's state
                [t(end+1),predX(end+1),predY(end+1),predTheta(end+1),...
                    preyX(end+1),preyY(end+1),preyTheta(end+1),preyState(end+1),predatorState(end+1)]...
                    = resolvePreyState(t(end),closestEvent,predX(end),predY(end),predTheta(end),preyX(end),...
                    preyY(end),preyTheta(end),preyState(end),predatorState(end),eventData);
                
            end
        end
    end
end


%% Package Output

R.t = t;
R.pred.x = predX;
R.pred.y = predY;
R.pred.theta = predTheta;
R.pred.state = predatorState;
R.pred.numStrikes = strike_counter;
R.pred.strike_duration = p.pred.strike_duration;
R.pred.strike_threshold = p.pred.strike_threshold;
R.prey.x = preyX;
R.prey.y = preyY;
R.prey.theta = preyTheta;
R.prey.state = preyState;
R.prey.numEscapes = escape_counter-1;
R.prey.escape_threshold = p.prey.escape_threshold;
R.prey.escape_dur = p.prey.escape_dur;
R.prey.escapeAngle = p.prey.escapeAngle;
R.prey.chanceOfErr = p.prey.chanceOfErr;
R.prey.captured = captured;

