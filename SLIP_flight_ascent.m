
% Initialize constants
g = 9.81;
h_apex = 0.8;
L0 = 0.5;

% Initialize initial conditions
x0 = 0.4189;
y0 = 0.5308;
xdot0 = 0.8154;
ydot0 = -0.1506;


% Calculate flight time
b = ydot0;
a = -g/2;
c = y0-h_apex;
%t_fa = (-b-sqrt(b^2-4*a*c))/(2*a)

t_fa = abs(ydot0/g);

% CoM trajectory calculation
t = linspace(0,t_fa,20);
x = x0 + xdot0*t;
y = y0 + ydot0*t-g/2*t.^2;

figure
plot(x,y)
xlabel('x-position [m]')
ylabel('y-position [m]')

