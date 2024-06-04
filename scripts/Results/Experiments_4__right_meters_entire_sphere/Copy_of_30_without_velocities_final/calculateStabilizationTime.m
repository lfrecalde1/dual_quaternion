function stabilizationTime = calculateStabilizationTime(signal, time, upper, lower)
    % Calculate the stabilization time of a signal
    %
    % :param signal: Vector of signal values
    % :param time: Vector of time values corresponding to the signal values
    % :param percentage: Percentage of the final value considered as stabilized range
    % :param finalSamples: Number of samples to consider for averaging the final value
    % :return: Stabilization time

    if nargin < 4
        finalSamples = 100; % Default number of samples to average for the final value
    end
    if nargin < 3
        percentage = 2; % Default stabilization range percentage
    end
    
    % Calculate the final value as the average of the last `finalSamples` samples
    %finalValue = mean(signal(end-finalSamples+1:end))
    
    % Calculate the stabilization bounds
    upperBound = upper;
    lowerBound = lower;
    
    % Initialize stabilization time
    stabilizationTime = NaN; % Use NaN to indicate that stabilization time hasn't been found
    
    % Iterate through the signal
    for i = 1:length(signal)
        if signal(i) <= upperBound && signal(i) >= lowerBound
            stabilizationTime = time(i);
            % Check if the signal stays within the bounds from this point onwards
            if all(signal(i:end) <= upperBound & signal(i:end) >= lowerBound)
                break; % Exit the loop if the signal has stabilized
            else
                % Reset stabilization time if it leaves the bounds later
                stabilizationTime = NaN;
            end
        end
    end
end