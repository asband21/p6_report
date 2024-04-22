


theta = [0, 0, 0, 0, 0, 0]
theta = [-0.9637444655047815, -1.464888111954071, -1.7108891010284424, -1.53567290425811, 1.5709656476974487, -0.9636171499835413]
r = [0, -0.6127, -0.57155, 0, 0, 0]
d = [0.1807, 0, 0, 0.17415, 0.11985, 0.11655]
alpha = [pi/2, 0, 0, pi/2, pi/2, 0]

%list = [0, 0, 0, 0, 0, 0];
%for i = 1:6
%    list(i) = [cos(theta(i)), -sin(theta(i)), 0, r(i); sin(theta(i))*cos(alpha(i)), cos(theta(i))*cos(alpha(i)), -sin(alpha(i)), -d(i)*sin(alpha(i)); sin(theta(i))*sin(alpha(i)), cos(theta(i))*sin(alpha(i)), cos(alpha(i)), d(i)*cos(alpha(i)); 0, 0, 0, 1]
%end
Tr1 = [cos(theta(1)), -sin(theta(1)), 0, r(1); sin(theta(1))*cos(alpha(1)), cos(theta(1))*cos(alpha(1)), -sin(alpha(1)), -d(1)*sin(alpha(1)); sin(theta(1))*sin(alpha(1)), cos(theta(1))*sin(alpha(1)), cos(alpha(1)), d(1)*cos(alpha(1)); 0, 0, 0, 1];
Tr2 = [cos(theta(2)), -sin(theta(2)), 0, r(2); sin(theta(2))*cos(alpha(2)), cos(theta(2))*cos(alpha(2)), -sin(alpha(2)), -d(2)*sin(alpha(2)); sin(theta(2))*sin(alpha(2)), cos(theta(2))*sin(alpha(2)), cos(alpha(2)), d(2)*cos(alpha(2)); 0, 0, 0, 1];
Tr3 = [cos(theta(3)), -sin(theta(3)), 0, r(3); sin(theta(3))*cos(alpha(3)), cos(theta(3))*cos(alpha(3)), -sin(alpha(3)), -d(3)*sin(alpha(3)); sin(theta(3))*sin(alpha(3)), cos(theta(3))*sin(alpha(3)), cos(alpha(3)), d(3)*cos(alpha(3)); 0, 0, 0, 1];
Tr4 = [cos(theta(4)), -sin(theta(4)), 0, r(4); sin(theta(4))*cos(alpha(4)), cos(theta(4))*cos(alpha(4)), -sin(alpha(4)), -d(4)*sin(alpha(4)); sin(theta(4))*sin(alpha(4)), cos(theta(4))*sin(alpha(4)), cos(alpha(4)), d(4)*cos(alpha(4)); 0, 0, 0, 1];
Tr5 = [cos(theta(5)), -sin(theta(5)), 0, r(5); sin(theta(5))*cos(alpha(5)), cos(theta(5))*cos(alpha(5)), -sin(alpha(5)), -d(5)*sin(alpha(5)); sin(theta(5))*sin(alpha(5)), cos(theta(5))*sin(alpha(5)), cos(alpha(5)), d(5)*cos(alpha(5)); 0, 0, 0, 1];
Tr6 = [cos(theta(6)), -sin(theta(6)), 0, r(6); sin(theta(6))*cos(alpha(6)), cos(theta(6))*cos(alpha(6)), -sin(alpha(6)), -d(6)*sin(alpha(6)); sin(theta(6))*sin(alpha(6)), cos(theta(6))*sin(alpha(6)), cos(alpha(6)), d(6)*cos(alpha(6)); 0, 0, 0, 1];

x = [0, 0, 0, 0, 0, 0, 0];
y = [0, 0, 0, 0, 0, 0, 0];
z = [0, 0, 0, 0, 0, 0, 0];

tt = Tr1;
x(2) = tt(1, 4);
y(2) = tt(2, 4);
z(2) = tt(3, 4);

tt = Tr1*Tr2
x(3) = tt(1, 4)
y(3) = tt(2, 4)
z(3) = tt(3, 4)

tt = Tr1*Tr2*Tr3;
x(4) = tt(1, 4);
y(4) = tt(2, 4);
z(4) = tt(3, 4);

tt = Tr1*Tr2*Tr3*Tr4;
x(5) = tt(1, 4);
y(5) = tt(2, 4);
z(5) = tt(3, 4);

tt = Tr1*Tr2*Tr3*Tr4*Tr5;
x(6) = tt(1, 4);
y(6) = tt(2, 4);
z(6) = tt(3, 4);

tt = Tr1*Tr2*Tr3*Tr4*Tr5*Tr6;
x(7) = tt(1, 4);
y(7) = tt(2, 4);
z(7) = tt(3, 4);

x
y
z
%trans0 =[cos(theta[0]), -sin(theta[0]), 0, r[0]; sin(theta[0])*cos(alpha[0]), cos(theta[0])*cos(alpha[0]), -sin(alpha[0-1]), -d[0]*sin(alpha[0-1]); sin(theta[0])*sin(alpha[0-1]), cos(theta[0])*sin(alpha[0]), cos(alpha[0]), d[0]*cos(alpha[0]); 0, 0, 0, 1]]
%trans1 =[cos(theta[1]), -sin(theta[1]), 0, r[1]; sin(theta[1])*cos(alpha[1]), cos(theta[1])*cos(alpha[1]), -sin(alpha[1-1]), -d[1]*sin(alpha[1-1]); sin(theta[1])*sin(alpha[1-1]), cos(theta[1])*sin(alpha[1]), cos(alpha[1]), d[1]*cos(alpha[1]); 0, 0, 0, 1]]
%trans2 =[cos(theta[2]), -sin(theta[2]), 0, r[2]; sin(theta[2])*cos(alpha[2]), cos(theta[2])*cos(alpha[2]), -sin(alpha[2-1]), -d[2]*sin(alpha[2-1]); sin(theta[2])*sin(alpha[2-1]), cos(theta[2])*sin(alpha[2]), cos(alpha[2]), d[2]*cos(alpha[2]); 0, 0, 0, 1]] 
%trans3 =[cos(theta[3]), -sin(theta[3]), 0, r[3]; sin(theta[3])*cos(alpha[3]), cos(theta[3])*cos(alpha[3]), -sin(alpha[3-1]), -d[3]*sin(alpha[3-1]); sin(theta[3])*sin(alpha[3-1]), cos(theta[3])*sin(alpha[3]), cos(alpha[3]), d[3]*cos(alpha[3]); 0, 0, 0, 1]] 
%trans4 =[cos(theta[4]), -sin(theta[4]), 0, r[4]; sin(theta[4])*cos(alpha[4]), cos(theta[4])*cos(alpha[4]), -sin(alpha[4-1]), -d[4]*sin(alpha[4-1]); sin(theta[4])*sin(alpha[4-1]), cos(theta[4])*sin(alpha[4]), cos(alpha[4]), d[4]*cos(alpha[4]); 0, 0, 0, 1]]
%trans5 =[cos(theta[5]), -sin(theta[5]), 0, r[5]; sin(theta[5])*cos(alpha[5]), cos(theta[5])*cos(alpha[5]), -sin(alpha[5-1]), -d[5]*sin(alpha[5-1]); sin(theta[5])*sin(alpha[5-1]), cos(theta[5])*sin(alpha[5]), cos(alpha[5]), d[5]*cos(alpha[5]); 0, 0, 0, 1]]

plot3(x,y,z)
xlabel('X')
ylabel('Y')
zlabel('Duration')
xlim([-1 1]);
ylim([-1 1]);
zlim([0 2])

grid on