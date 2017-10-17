    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %INVERSE KINEMATICS FOR SPACE CONCORDIA ROBOTICS TEAM 2017%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%NOTE: The first angle in the code (a1a) is the angle that the base makes
%with the horizontal. Since the base is immobile, this angle is assumed to
%be constant. In fact, the properties of the base are not taken into
%account in this code but are simply added to the final figure. 
%For this reason, each angle has a higher coefficient than its
%corresponding link. E.g. link 1 corresponds to angle a2a.

%This code uses a closed form solution to determine the orientation of the
%joints of a planar RRR arm for its end effector to reach a given point.For
%more information about workings of the code, contact Maxim Kaller.

%INITIALIZATION OF VARIABLES

d1 = 0.1; %Length of base (could include rover length)
l1 = 0.5; %li = length of link i
l2 = 0.5; 
l3 = 0.2; 
amax2 = pi/2; %amaxi = maximum angle link i + 1 can reach
amin2 = 0; %amini = minimum angle link i + 1 can reach
amax3 = pi;
amin3 = -pi;
amax4 = pi;
amin = -pi;
X = 0.0; %Desired X position
Y = 1.3; %Desired Y position
%Z = 0.7; %Desired Z position (CURRENTLY USELESS)
%BULK OF THE CODE



beta = atan2(Y - d1 , X); %Calculates beta, which is the sum of the angles of all links
xn = X - l3 * cos(beta); %Calculates xn, which is the x coordinate of the wrist
yn = (Y - d1) - l3 * sin(beta); %Calculates yn, which is the y coordinate of the wrist

if sqrt(xn^2 + yn^2) > l1 + l2 %Check to see if point is too far
    disp('Error! Cannot reach point!');
    return
end

a3 = (xn^2 + yn^2 - l1^2 - l2^2)/(2 * l1 * l2); %Calculates the cosine of the angle of link 2
a3a = acos(a3); %Proper angle is reached. NOTE: Depending on speed of function acos()
                %an approximation or different calculation method should be
                %used to speed computation time
                
if a3a > amax3 | a3a < amin3 %Check to see if third angle is within bounds
    disp('Error! Angle 3 is not within bounds');
    return
end

aphi = (xn^2 + yn^2 + l1^2 - l2^2)/(2 * sqrt(xn^2 + yn^2) * l1); %Determine cos of phi
phi = acos(aphi); %Phi is the angle between the first link and the straight line
                  %from the base to the end effector

%Depending on the configuration of the arm, phi and beta can be used to
%find the second angle

if a3a <= 0
    a2a = beta + phi;
end

if a3a > 0
    a2a = beta - phi;
end

%If a2a is too small, calculations will restart with a3a being negative
if a2a > amax2 | a2a < amin2 
    disp(a2a);
    a3a = -a3a;
end

if a3a < 0
    a2a = beta + phi;
end

if a3a > 0
    a2a = beta - phi;
end

%Check to see if second angle is within bounds
if a2a > amax2 | a2a < amin2
    disp('Error! Angle 2 is not within bounds');
    return
end

%Knowing the sum of all angles (beta) and two of the three angles,
%the final angle (angle four) can be found:

a4a = beta - (a2a + a3a);

%The following line creates a circle around the specified point. This point
%is used to confirm that the end effector is well positioned:
scatter(X,Y, 200, 'LineWidth' , 2.5);

%The following code generates a graphical representation of the arm:
hold on;
plot( [0 0] , [0 d1] , 'k' , 'LineWidth' , 75);
plot( [0 l1 * cos(a2a)] , [ d1 d1 + l1 * sin(a2a)], 'k' , 'LineWidth' , 3);
plot( [l1 * cos(a2a) l1 * cos(a2a) + l2 * cos(a2a + a3a)] , [d1 + l1 * sin(a2a) d1 + l1 * sin(a2a) + l2 * sin(a2a + a3a)], 'k', 'LineWidth' , 3);
plot( [ l1 * cos(a2a) + l2 * cos(a2a + a3a) l1 * cos(a2a) + l2 * cos(a2a + a3a) + l3 * cos(a2a + a3a + a4a)] , [ d1 + l1 * sin(a2a) + l2 * sin(a2a + a3a) d1 + l1 * sin(a2a) + l2 * sin(a2a + a3a) + l3 * sin(a2a + a3a + a4a) ] , 'k' , 'LineWidth' , 3);

axis equal; %To avoid weird graphs. NOTE: X axis and Y axis may still be of different lengths.
            %This will make links seem smaller/bigger than they actually
            %are. Fear not. As long as the end effector touches the circle
            %it is all good.