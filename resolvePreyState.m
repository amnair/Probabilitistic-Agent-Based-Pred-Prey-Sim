function [t,predX,predY,predTheta,preyX,preyY,preyTheta,preyState,predatorState] = resolvePreyState...
    (t,eventTime,predX,predY,predTheta,preyX,preyY,preyTheta,preyState,predatorState,eventData)
% Function that resolves any of the three predator states
%
% t - global time
% eventTime -  time duration of current state
% predX - predator's X position
% predY - predator's Y position
% predTheta - predator's orientation angle
% predatorState - current predator state
% eventData -  misc. data needed for a given state (W: predator state, escape threshold, distance travelled)
% preyX - prey's X position
% preyY - prey's Y position
% preyTheta - prey's orientation angle
% preyState - prey's current state

switch preyState
    
    case 'W' % Waiting
        
        % Update time vector
        t = t + eventTime;
        
        %Determine where the predator is for this iteration
        if eventData{1} == 'T'
            
            predTheta = atan2(preyY-predY,preyX-predX);
            [x_comp,y_comp] = pol2cart(predTheta+pi,eventData{2});
            predX = preyX + x_comp;
            predY = preyY + y_comp;
            
        else % Somwhere in the strike state
            
            [x_comp,y_comp] = pol2cart(predTheta,eventData{3});
            predX = predX + x_comp;
            predY = predY + y_comp;
            
        end
        
        % Transition into escaping
        preyState = 'E';
        
end