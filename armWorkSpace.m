d1 = 0.5; 
l1 = 1; 
l2 = 1; 
l3 = 0.25; 
amax = (4 * pi)/6;
amin = pi;
amax2 = 0;
amin2 = -pi;
amax3 = pi;
amin3 = -2 * pi/3;
aa = (amax + amin) / 2;
aa2 = (amax2 + amin2)  / 2;
aa3 = (amax3 + amin3) / 2;
adif = amax - amin;
adif2 = amax2 - amin2;
adif3 = amax3 - amin3;
hold on;
for i = 1:100000
    t2 = adif * rand + amin;
    t3 = adif2 * rand + amin2;
    t4 = adif3 * rand + amin3;
    x(i) = l1 * cos(t2) + l2 * cos(t2 + t3) + l3 * cos(t2 + t3 + t4);
    y(i) = d1 + l1 * sin(t2) + l2 * sin(t2 + t3) + l3 * sin( t2 + t3 + t4);
end
scatter(x,y);
plot( [0 0] , [0 d1] , 'k' );
plot( [0 l1 * cos(aa)] , [ d1 d1 + l1 * sin(aa)], 'k' );
plot( [l1 * cos(aa) l1 * cos(aa) + l2 * cos(aa + aa2)] , [d1 + l1 * sin(aa) d1 + l1 * sin(aa) + l2 * sin(aa + aa2)], 'k' );
plot( [ l1 * cos(aa) + l2 * cos(aa + aa2) l1 * cos(aa) + l2 * cos(aa + aa2) + l3 * cos(aa + aa2 + aa3)] , [ d1 + l1 * sin(aa) + l2 * sin(aa + aa2) d1 + l1 * sin(aa) + l2 * sin(aa + aa2) + l3 * sin(aa + aa2 + aa3) ] , 'k');
