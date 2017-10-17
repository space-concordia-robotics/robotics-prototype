 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
 %REPRESENTATION OF ARM WORKSPACE FOR SPACE CONCORDIA ROBOTICS TEAM 2017% 
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%This code uses the Monte Carlo method to find the planar workspace of a 3R
%arm. This configuration is similar to the one the team seeks to use for
%its own arm. The code basically generates a large cloud of points
%reachable by the arm to get an idea of the shape of the workspace


%INITIALIZATION OF VARIABLES

d1 = 0.5; %Length of base (could include rover length)
l1 = 1; %li = length of link i
l2 = 1; 
l3 = 0.25; 
amax = (4*pi)/6; %amaxi = maximum angle link i can reach
amin = pi/6; %amini = minimum angle link i can reach
amax2 = pi/2;
amin2 = -(4*pi/6);
amax3 = pi/2;
amin3 = -2* pi/3;
aa = (amax + amin) / 2; %aai is the angle that will be displayed for link i.
                        %For now it is set to be the average of the angle
                        %limits.
aa2 = (amax2 + amin2)  / 2;
aa3 = (amax3 + amin3) / 2;
adif = amax - amin; %adifi is the span of angles of joint i
adif2 = amax2 - amin2;
adif3 = amax3 - amin3;

%BULK OF THE CODE

hold on; %Simple command to make sure graphs are not erased upon the creation of new ones


%MAIN MONTE CARLO LOOP:
%This loop generates a very large amount of points in the arm's workspace.
%The more points generated, the more visible the workspace.

for i = 1:666666
    t2 = adif * rand + amin; %The following line finds a random angle for 
                             %joint 2 within its span.
    t3 = adif2 * rand + amin2;
    t4 = adif3 * rand + amin3;
    
    %The following 2 lines compute the x and y coordinates corresponding to
    %the angles generated:
    
    x(i) = l1 * cos(t2) + l2 * cos(t2 + t3) + l3 * cos(t2 + t3 + t4); 
    y(i) = d1 + l1 * sin(t2) + l2 * sin(t2 + t3) + l3 * sin( t2 + t3 + t4);
end

%This line is used to graph all the points found in the above loop:
scatter(x,y,0.75,'filled');

%The following lines add black circles where the arm joints should be:
scatter(0,d1, 100,'k', 'filled');
scatter(l1 * cos(aa),d1 + l1 * sin(aa), 75,'k', 'filled');
scatter(l1 * cos(aa) + l2 * cos(aa + aa2),d1 + l1 * sin(aa) + l2 * sin(aa + aa2), 75,'k', 'filled');

%The following lines plot a figure representing the arm's configuration:
plot( [0 0] , [0 d1] , 'k' , 'LineWidth' , 75);
plot( [0 l1 * cos(aa)] , [ d1 d1 + l1 * sin(aa)], 'k' , 'LineWidth' , 3);
plot( [l1 * cos(aa) l1 * cos(aa) + l2 * cos(aa + aa2)] , [d1 + l1 * sin(aa) d1 + l1 * sin(aa) + l2 * sin(aa + aa2)], 'k' , 'LineWidth' , 3);
plot( [ l1 * cos(aa) + l2 * cos(aa + aa2) l1 * cos(aa) + l2 * cos(aa + aa2) + l3 * cos(aa + aa2 + aa3)] , [ d1 + l1 * sin(aa) + l2 * sin(aa + aa2) d1 + l1 * sin(aa) + l2 * sin(aa + aa2) + l3 * sin(aa + aa2 + aa3) ] , 'k' , 'LineWidth' , 3);