function plot_areaerrorbar_2(data, time, options)

    % Default options
    if(nargin<3)
        options.handle     = figure(1);
        %options.color_area = [128 193 219]./255;    % Blue theme
        %options.color_line = [ 52 148 186]./255;
        options.color_area = [243 169 114]./255;    % Orange theme
        options.color_line = [236 112  22]./255;
        options.alpha      = 0.5;
        options.line_width = 2;
        options.error      = 'std';
    end
    if(isfield(options,'x_axis')==0), options.x_axis = 1:size(data,2); end
    options.x_axis = options.x_axis(:);
    
    % Computing the mean and standard deviation of the data matrix
    data_mean = nanmean(data);
    data_std  = nanstd(data,0,1);
    
    % Type of error plot
    switch(options.error)
        case 'std', error = data_std;
        case 'sem', error = (data_std./sqrt(size(data,1)));
        case 'var', error = (data_std.^2);
        case 'c95', error = (data_std./sqrt(size(data,1))).*1.96;
    end
    
    % Plotting the result
    x_vector = [time, fliplr(time)];
    patch = fill(x_vector, [data_mean+error,fliplr(data_mean-error)], options.color_area);
    set(patch, 'edgecolor', 'none');
    set(patch, 'FaceAlpha', options.alpha);
    hold on;
    plot(time', data_mean, 'color', options.color_line, ...
        'LineWidth', options.line_width);
end