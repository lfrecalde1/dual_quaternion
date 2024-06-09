t = linspace(0,2*pi,1000); % values in time
x = cos(t); % a sinusoid
perturbation = 0.1*exp((-(t-5*pi/4).^2)/.01).*sin(200*t); % a perturbation
signal = x+perturbation; % a signal to plot
% plot signal on new figure
figure,plot(t,x+perturbation)
xlabel('time'),ylabel('signal')
% create a new pair of axes inside current figure
axes('position',[.65 .175 .25 .25])
box on % put box around new pair of axes
indexOfInterest = (t < 11*pi/8) & (t > 9*pi/8); % range of t near perturbation
plot(t(indexOfInterest),signal(indexOfInterest)) % plot on new axes
axis tight