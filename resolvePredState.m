function [t,predX,predY,predTheta,preyX,preyY,preyTheta,predatorState,preyState,simNotif] = resolvePredState...
    (t,eventTime,predX,predY,predTheta,preyX,preyY,preyTheta,predatorState,preyState,eventData)
% Function that resolves any of the three predator states
%
% t - global time
% eventTime -  time duration of current state
% predX - predator's X position
% predY - predator's Y position
% predTheta - predator's orientation angle
% predatorState - current predator state
% eventData -  misc. data needed for a given state (T: predator delay, strike threshold, tracking error, S&E:distance traveled)
% preyX - prey's X position
% preyY - prey's Y position
% preyTheta - prey's orientation angle
% preyState - prey's current state
% simNotif - simulation notiftication to check whether the simulation should end

switch predatorState(end) % Resolve one of the states depending on what the current state is
    case 'T' % Tracking
        
        % Update time vector
        t(end+1) = t(end) + eventTime;
        
        % Update position
        if t(end-1) > eventData{1} && length(t) > 2
            delayedPreyX = interp1(t(1:end-1),preyX,t(end-1)-eventData{1});
            delayedPreyY = interp1(t(1:end-1),preyY,t(end-1)-eventData{1});
            predTheta(end+1) = atan2(delayedPreyY-predY(end),delayedPreyX-predX(end));
        else
            predTheta(end+1) = atan2(preyY(1)-predY(end),preyX(1)-predX(end));
        end
        
        % Adjust x,y position of predator and update prey position/orientation
        [x_comp,y_comp] = pol2cart(predTheta(end)+pi,eventData{2});
        [xError, yError] = pol2cart(2*pi*rand(1),norm([x_comp y_comp])*eventData{3}*rand(1)); % Error in tracking
        predX(end+1) = preyX(end) + x_comp + xError;
        predY(end+1) = preyY(end) + y_comp + yError;
        
        % Readjust direction due to error in locomotion
        if t(end-1) > eventData{1} && length(t) > 2
            delayedPreyX = interp1(t(1:end-1),preyX,t(end-1)-eventData{1});
            delayedPreyY = interp1(t(1:end-1),preyY,t(end-1)-eventData{1});
            predTheta(end) = atan2(delayedPreyY-predY(end),delayedPreyX-predX(end));
        else
            predTheta(end) = atan2(preyY(1)-predY(end),preyX(1)-predX(end));
        end
        
        % Transition into strike mode only if prey is in capture zone
        if t(end-1) > eventData{1} && length(t) > 2
            delayedPreyX = interp1(t(1:end-1),preyX,t(end-1)-eventData{1});
            delayedPreyY = interp1(t(1:end-1),preyY,t(end-1)-eventData{1});
            th = atan2(delayedPreyY-predY(end),delayedPreyX-predX(end));
        else
            th = atan2(preyY(1)-predY(end),preyX(1)-predX(end));
        end
        if th-predTheta(end) >= -pi/2 && th-predTheta(end) <= pi/2
            predatorState(end+1) = 'S';
        end
        
    case 'S' % First half of strike
        
        % Update time vector
        t(end+1) = t(end) + eventTime;
        
        % Adjust predator coodinates
        [x_comp,y_comp] = pol2cart(predTheta(end),eventData{1});
        predX(end+1) = predX(end) + x_comp;
        predY(end+1) = predY(end) + y_comp;
        predTheta(end+1) = predTheta(end);
        
        % Check if prey is captured
        [th,r] = cart2pol(preyX(end)-predX(end),preyY(end)-predY(end));
        if r <= 0.01 && (th-predTheta(end) >= -pi/2 && th-predTheta(end) <= pi/2)
           simNotif = 1; % In zone, will check if captured in caller function
        else
            simNotif = 0; % Not in zone
        end
        
        % Transition into the end of a strike
        predatorState(end+1) = 'E';
        
    case 'E' % Second have of the Strike
        
        % Update time vector
        t(end+1) = t(end) + eventTime;
        
        % Adjust predator coodinates
        [x_comp,y_comp] = pol2cart(predTheta(end),eventData{1});
        predX(end+1) = predX(end) + x_comp;
        predY(end+1) = predY(end) + y_comp;
        predTheta(end+1) = predTheta(end);
        
        % Transition into tracking again
        predatorState(end+1) = 'T';
        
end

%Update prey's current status
preyX(end+1) = preyX(end);
preyY(end+1) = preyY(end);
preyTheta(end+1) = preyTheta(end);
preyState(end+1) = preyState(end);


