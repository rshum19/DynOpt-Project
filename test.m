

Leg.apex0 = 0.8;
Leg.L0 = 0.5;


% Initialize initial conditions: starts at the apex
x0 = 0;
y0 = h_apex;
xdot0 = 0.5;                % COM x-velocity [m/s]
ydot0 = 0;                  % COM y-velocity [m/s], 0 becuase @ apex
theta_td0 = degtorad(95);    % Touch-down angle  
X0 = [x0 y0 xdot0 ydot0 theta_td0];

[ COMtrajectory, Foottrajectory, stance_char] = SLIP_sim( Leg, X0 );


% Animation
figure
for i = 1:length(COMtrajectory.x)
    plot(COMtrajectory.x,COMtrajectory.y,'g:');
    hold on;
    plot(COMtrajectory.x(i),COMtrajectory.y(i),'o','MarkerSize',10,'MarkerFaceColor','b')
    plot([COMtrajectory.x(i),Foottrajectory.x(i)],[COMtrajectory.y(i),Foottrajectory.y(i)])
    axis([0,1,0,1])
    xlabel('x-positon [m]')
    ylabel('y-positon [m]')
    pause(0.2);
    hold off
end