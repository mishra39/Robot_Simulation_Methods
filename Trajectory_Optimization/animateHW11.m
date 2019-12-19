function animateHW11(x, dt)
% x: collection of state vectors at each time, similar to output from ode45
% dt: (difference in time between each row of xout, generated by calling
% ode45 with the argument [tstart:dt:tfinal];)

bRecord = 0;  % Uncomment this to save a video
if bRecord
    % Define video recording parameters
    Filename = 'current_animation';
    v = VideoWriter(Filename, 'MPEG-4');
    myVideo.Quality = 100;
    open(v);
end

% Define axis window
xmin = -2;
xmax = 2;
ymin = -2;
ymax = 2;

Fig = figure('Color', 'w');

% Create trace of trajectory and particle object
h = animatedline('LineStyle', ':', 'LineWidth', 1.5);
manipulator = [];
endeffector = [];

% Set up axes
axis equal
axis([xmin xmax ymin ymax])
xlabel('x')
ylabel('y')

% draw
base = [0;0];
L = 1;
for ii = 1:length(x)
    a = tic;
    
    set(gcf,'DoubleBuffer','on');
    
    q1 = x(ii,1);
    q2 = x(ii,2);
    joint1 = [L*cos(q1);L*sin(q1)];
    joint2 = [L*cos(q1) + L*cos(q1+q2);L*sin(q1) + L*sin(q1+q2)]; 
    delete(manipulator);
    delete(endeffector);
    manipulator = line([base(1), joint1(1), joint2(1)], [base(2), joint1(2), joint2(2)], 'Color', [0;0;0],'LineStyle','-');
    endeffector = line(joint2(1), joint2(2), 'Color', [1;0;0],'Marker','.', 'MarkerSize', 20);
    addpoints(h,joint2(1), joint2(2));
    
    drawnow limitrate
    
    if bRecord
        frame = getframe(gcf);
        writeVideo(v,frame);
    else
        pause(dt - toc(a)); % waits if drawing frame took less time than anticipated
    end
end

if bRecord
    close(v);
end