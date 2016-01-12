close all;

% Simulation parameters
numSims = 1000; % Number of simulations
plotting = 0;

% Bounding conditions for prey probability distributions
escape_threshold_UB = 0.0368;
escape_threshold_LB = 0.0031;

escape_dur_UB = 0.7280;
escape_dur_LB = 0.0790;

escapeAngle_UB = 3.0641;
escapeAngle_LB = 0.1226;


% Iterate through a number of simulations
for i = 1:numSims
    
    % Load in default parameter values
    [p,d] = default_params;
    
    % Set rng seed
    rng('shuffle');
    R.rng = rng;
    
    %Initiate random variables for predator and prey (generate a surplus of random values)
    % Randomly generated strike durations
    strike_dur_pd = makedist('lognormal','mu',p.pred.strike_dur_mu,'sigma',p.pred.strike_dur_sigma);
    p.pred.strike_duration = random(strike_dur_pd,1,20);
    
    %Remove values that are greater or lower than observed values in thunderdome experiments
    outOfBoundIdx =  (p.pred.strike_duration > 0.075) | (p.pred.strike_duration < 0.005);
    if sum(outOfBoundIdx) > 0
        complete = 0;
        while complete == 0
            replacement = random(strike_dur_pd,1,sum(outOfBoundIdx));
            if sum((replacement > 0.075) | (replacement < 0.005)) == 0
                complete = 1;
            end
        end
        p.pred.strike_duration(outOfBoundIdx) = replacement;
    end
    
    % Randomly generated strike threshold
    strike_threshold_pd = makedist('burr','alpha',p.pred.strike_thresh_alpha,'c',p.pred.strike_thresh_c,'k',p.pred.strike_thresh_k);
    p.pred.strike_threshold = random(strike_threshold_pd,1,20);
    
    %Remove values that are greater or lower than observed values in thunderdome experiments
    outOfBoundIdx =  (p.pred.strike_threshold > 0.0177) | (p.pred.strike_threshold < 0.0022);
    if sum(outOfBoundIdx) > 0
        complete = 0;
        while complete == 0
            replacement = random(strike_threshold_pd,1,sum(outOfBoundIdx));
            if sum((replacement > 0.0177) | (replacement < 0.0022)) == 0
                complete = 1;
            end
        end
        p.pred.strike_threshold(outOfBoundIdx) = replacement;
    end
    
    % Randomly generated reaction distance
    escape_threshold_pd = makedist('lognormal','mu',p.prey.thrshEscape_mu,'sigma',p.prey.thrshEscape_sigma);
    p.prey.escape_threshold = random(escape_threshold_pd,1,20);
    
    % Remove values that are greater or lower than observed values in thunderdome experiments
    outOfBoundIdx =  (p.prey.escape_threshold > escape_threshold_UB) | (p.prey.escape_threshold < escape_threshold_LB);
    if sum(outOfBoundIdx) > 0
        complete = 0;
        while complete == 0
            replacement = random(escape_threshold_pd,1,sum(outOfBoundIdx));
            if sum((replacement > escape_threshold_UB) | (replacement < escape_threshold_LB)) == 0
                complete = 1;
            end
        end
        p.prey.escape_threshold(outOfBoundIdx) = replacement;
    end
    
    % Randomly generated escape angles for an escape response
    escapeAngle_pd = makedist('burr','alpha',p.prey.escapeAngle_alpha,'c',p.prey.escapeAngle_c,'k',p.prey.escapeAngle_k);
    p.prey.escapeAngle  = random(escapeAngle_pd,1,20);
    
    %Remove values that are greater or lower than observed values in thunderdome experiments
    outOfBoundIdx =  (p.prey.escapeAngle > 3.0641) | (p.prey.escapeAngle < 0.1226);
    if sum(outOfBoundIdx) > 0
        complete = 0;
        while complete == 0
            replacement = random(escapeAngle_pd,1,sum(outOfBoundIdx));
            if sum((replacement > 3.0641) | (replacement < 0.1226)) == 0
                complete = 1;
            end
        end
        p.prey.escapeAngle(outOfBoundIdx) = replacement;
    end
    
    % Randomly generated durations of an escape response
    escape_dur_pd = makedist('lognormal','mu',p.prey.durEscape_mu,'sigma',p.prey.durEscape_sigma);
    p.prey.escape_dur = random(escape_dur_pd,1,20);
    
    %Remove values that are greater or lower than observed values in thunderdome experiments
    outOfBoundIdx =  (p.prey.escape_dur > escape_dur_UB) | (p.prey.escape_dur < escape_dur_LB);
    if sum(outOfBoundIdx) > 0
        complete = 0;
        while complete == 0
            replacement = random(escape_dur_pd,1,sum(outOfBoundIdx));
            if sum((replacement > escape_dur_UB) | (replacement < escape_dur_LB)) == 0
                complete = 1;
            end
        end
        p.prey.escape_dur(outOfBoundIdx) = replacement;
    end
    
    % Randomly generated chances of escaping in the wrong direction
    chanceOfErr = rand(1,20);
    chanceOfErr = chanceOfErr <= 0.3041; % 206/296 samples go in the "correct" direction (from Kelsey's Robopredator exp.)
    chanceOfErr = chanceOfErr * -1;
    chanceOfErr(chanceOfErr == 0) = 1;
    p.prey.chanceOfErr = chanceOfErr;
    
    %Save initial set-up (for debugging purposes)
    R.pred.strike_duration = p.pred.strike_duration;
    R.pred.strike_threshold = p.pred.strike_threshold;
    R.prey.escape_threshold = p.prey.escape_threshold;
    R.prey.escape_dur = p.prey.escape_dur;
    R.prey.escapeAngle = p.prey.escapeAngle;
    R.prey.chanceOfErr = p.prey.chanceOfErr;
    save(['data/R' num2str(i) '.mat'],'R');
    
    % Run simulation
    R = simple_model(p,d);
    
    % Save the output
    R.rng = rng;
    save(['data/R' num2str(i) '.mat'],'R');
    
    % Display whether or not prey escaped
    if R.prey.captured == 0
        disp('prey successfully escaped')
    else
        disp(['the prey was captured at time = ' num2str(R.t(end)) ' with ' num2str(R.pred.numStrikes)  ' strikes and ' num2str(R.prey.numEscapes) ' escapes'])
    end
    
    %Visualize encounter
    if plotting == 1
        
        vis_results(R,'plot')
        
    end
    
end

