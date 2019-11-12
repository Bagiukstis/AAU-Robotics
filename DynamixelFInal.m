%          Inverse manipulator dynamics for Dynamixel 3-DOF Robot
%
%                   Made by:
%
%                   Philip Lund Møller pmalle18@student.aau.dk
%                   Valdas Druskinis  vdrusk18@student.aau.dk
%
%
%
%                   Aalborg University 2019
%

%% Trajectory Generation%%
%%%%%%%%%%%%%%%%%%%%%%%%%%
%%Positions and variables 
old_pos = [49.76;0;5.380];
new_pos = [27.780; 
           0; 
           27.360];
tf = 3;
t = 0.5;

%% X trajectory
x_a0 = old_pos(1,1);
x_a1 = 0;
x_a2 = (3/tf^2)*(new_pos(1,1)-old_pos(1,1));
x_a3 = (-2/tf^3)*(new_pos(1,1)-old_pos(1,1));    

x_pos = x_a0 + x_a1*t + x_a2*t^2 + x_a3*t^3;
x_vel = x_a1 + 2*x_a2*t + 3*x_a3*t^2;
x_acc = 2*x_a2 + 6*x_a3*t;

%% Y trajectory
y_a0 = old_pos(2,1);
y_a1 = 0;
y_a2 = (3/tf^2)*(new_pos(2,1)-old_pos(2,1));
y_a3 = (-2/tf^3)*(new_pos(2,1)-old_pos(2,1));    

y_pos = y_a0 + y_a1*t + y_a2*t^2 + y_a3*t^3;
y_vel = y_a1 + 2*y_a2*t + 3*y_a3*t^2;
y_acc = 2*y_a2 + 6*y_a3*t;

%% Z trajectory
z_a0 = old_pos(3,1);
z_a1 = 0;
z_a2 = (3/tf^2)*(new_pos(3,1)-old_pos(3,1));
z_a3 = (-2/tf^3)*(new_pos(3,1)-old_pos(3,1));    

z_pos = z_a0 + z_a1*t + z_a2*t^2 + z_a3*t^3;
z_vel = z_a1 + 2*z_a2*t + 3*z_a3*t^2;
z_acc = 2*z_a2 + 6*z_a3*t;




%% Inverse Kinematics %%
%%%%%%%%%%%%%%%%%%%%%%%%

%Variables
L1 = 21.98;
L2 = 27.88;
L4 = sqrt(y_pos^2 + x_pos^2);
L5 = z_pos-5.38;
L3 = sqrt((L4^2)+(L5^2));

%Theta 1:
th1_t = atan2(y_pos,x_pos);

%Theta 2:
phi2 = acos((L1^2+L3^2-L2^2)/(2*L1*L3));
phi1 = atan2(L4,L5);
th2_t = (phi1-phi2);

%Theta 3:
phi3 = acos((L1^2+L2^2-L3^2)/(2*L1*L2));
th3_t = pi-phi3;


%% Forward Kinematics %%
%%%%%%%%%%%%%%%%%%%%%%%%

r1 = 5.38;
r2 = 21.98;
r3 = 27.78;

x = cos(th1_t)*(cos(th2_t)*(r2 + r3*cos(th3_t)) - r3*sin(th2_t)*sin(th3_t));
y = sin(th1_t)*(cos(th2_t)*(r2 + r3*cos(th3_t)) - r3*sin(th2_t)*sin(th3_t));
z = r1 - sin(th2_t)*(r2 + r3*cos(th3_t)) - r3*cos(th2_t)*sin(th3_t);


%% Vectors and Rotatins %%
%%%%%%%%%%%%%%%%%%%%%%%%%%
R1= [cos(th1_t), -sin(th1_t), 0;
     sin(th1_t),  cos(th1_t), 0;
     0,             0,          1];
  
R3= [cos(th3_t), -sin(th3_t), 0;
     sin(th3_t),  cos(th3_t), 0;
     0,             0,          1];
 
R2= [cos(th2_t), -sin(th2_t), 0;
                0, 0, 1;
    sin(th2_t), cos(th2_t), 0];

R12= R1*R2;
R13 = R12 * R3;

%Vectors
d2= [0.1855;0;0];
l=  [0.0339;0;0];
d3 = [0.142;0;0];

rvc2= (R1*R2)*d2; %To CoM grid 2 
rvc3= (R1*R2*R3)*d3; %To CoM grid 3
rv2 = (R1*R2)*(l+d2);%From CoM to end of servo grid 2

%% Jacobian Matrix %%
%%%%%%%%%%%%%%%%%%%%%
J = [ -sin(th1_t)*(cos(th2_t)*(r2 + r3*cos(th3_t)) - r3*sin(th2_t)*sin(th3_t)), -cos(th1_t)*(sin(th2_t)*(r2 + r3*cos(th3_t)) + r3*cos(th2_t)*sin(th3_t)), -cos(th1_t)*(r3*cos(th2_t)*sin(th3_t) + r3*cos(th3_t)*sin(th2_t));
  cos(th1_t)*(cos(th2_t)*(r2 + r3*cos(th3_t)) - r3*sin(th2_t)*sin(th3_t)), -sin(th1_t)*(sin(th2_t)*(r2 + r3*cos(th3_t)) + r3*cos(th2_t)*sin(th3_t)), -sin(th1_t)*(r3*cos(th2_t)*sin(th3_t) + r3*cos(th3_t)*sin(th2_t));
                                                              0,             r3*sin(th2_t)*sin(th3_t) - cos(th2_t)*(r2 + r3*cos(th3_t)),             r3*sin(th2_t)*sin(th3_t) - r3*cos(th2_t)*cos(th3_t)];
                                                          
%xd,yd,zd
cart_vel = [x_vel; y_vel; z_vel]

%xdd, ydd, zdd
cart_acc = [x_acc; y_acc; z_acc]

%Angular velocity
ang_vel = inv(J)*cart_vel;
th1d = ang_vel(1,1);
th2d = ang_vel(2,1);
th3d = ang_vel(3,1);

Jd = [ -cos(th1_t)*th1d*(cos(th2_t)*th2d*(r2 + r3*cos(th3_t)*th3d) - r3*sin(th2_t)*th2d*sin(th3_t)*th3d),  sin(th1_t)*th1d*(sin(th2_t)*th2d*(r2 + r3*cos(th3_t)*th3d) + r3*cos(th2_t)*th2d*sin(th3_t)*th3d),  sin(th1_t)*th1d*(r3*cos(th2_t)*th2d*sin(th3_t)*th3d + r3*cos(th3_t)*th3d*sin(th2_t)*th2d);
 -sin(th1_t)*th1d*(cos(th2_t)*th2d*(r2 + r3*cos(th3_t)*th3d) - r3*sin(th2_t)*th2d*sin(th3_t)*th3d), -cos(th1_t)*th1d*(sin(th2_t)*th2d*(r2 + r3*cos(th3_t)*th3d) + r3*cos(th2_t)*th2d*sin(th3_t)*th3d), -cos(th1_t)*th1d*(r3*cos(th2_t)*th2d*sin(th3_t)*th3d + r3*cos(th3_t)*th3d*sin(th2_t)*th2d);
                                                              0,                                                              0,                                                       0];                                                          

%Angular acceleration
ang_acc = inv(J)*(cart_acc-Jd*ang_vel);
th1dd = ang_acc(1,1);
th2dd = ang_acc(2,1);
th3dd = ang_acc(3,1);

%%Velocities and acceleration%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
z= [0;0;1];

w1 = th1d * z; %omega1
w2 = w1+ th2d* R12(:,3); %omega2
w3 = w2 + th3d * R13(:,3);%omega3

vc2= cross(w2,rvc2); %velocity to the CoM body 2
v2 = cross(w2,rv2); %total velocity body 2
vc3 = v2 + (cross(w2,rvc3)); %velocity to CoM body 3
%% Manipulator Dynamics %%
%%%%%%%%%%%%%%%%%%%%%%%%%%

%Masses
m1 = 0.249;
m2 = 0.3142;

%Inertia tensor 2:
I2xx = 0.00053933;
I2xy = 0.5*10^(-7);
I2xz = 0.397*10^(-5);
I2yx = 0.5*10^(-7);
I2yy = 0.00055977;
I2yz = 0.903*10^(-5);
I2zx = 0.397*10^(-5);
I2zy = 0.903*10^(-5);
I2zz = 0.00005433;

%Inertia tensor 3:
I3xx = 0.00054544;
I3xy = 0;
I3xz = 0;
I3yx = 0;
I3yy = 0.00063851;
I3yz = 0.00001013;
I3zx = 0;
I3zy = 0.00001013;
I3zz = 0.00016132;

h2 = transpose(rc2)*z; % Z part of vector rc2 for height (d2*sin(theta))
h2l = transpose(r2)*z;
h3 = transpose(rc3)*z; % Z part of vector rc3 

%Inertia tensors summed

I2 = [0.00053933 0.00000005 0.00000397;
    0.00000005 0.00055977 0.00000903;
    0.00000397 0.00000903 0.00005433];

I3 = [0.00054544 0 0;
    0 0.00063851 0.00001013;
    0 0.00001013 0.00016132];

d1 = 0.1859;
d2 = 0.0339;
d3 = 0.1422;

G2 = 9.81;
G3 = G2;
%% Energies and Lagrangian %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%KInetic energy
k2 = (1/2)*m1*(transpose(vc2)*vc2) + 1/2 * transpose(w2) * (I2*w2); %Body2
k3 = (1/2)*m2*(transpose(vc3)*vc3) + 1/2 * transpose(w3) * (I3*w3); %Body3
%Potential energy

u2 = m1*G2*h2; %Body 2
u3 = m2*G3*(h2l+h3); %Body 3

%Lagrangian

L = (k2-u2) + (k3-u3);
%% TORQUE GENERATION %%
%%%%%%%%%%%%%%%%%%%%%%%

Tau1 = (-th2d*(I2yz + I2zy)*sin(th1_t)/2 - th2d*(I2zx + I2xz)*cos(th1_t)/2 + ((th2d + th3d)*(-I3yz - I3zy)*sin(th1_t))/2 - ((th2d + th3d)*(I3xz + I3zx)*cos(th1_t))/2)*th1d + (-2*m1*th1d*cos(th2_t)*d1^2*sin(th2_t) - 2*(2*cos(th3_t)^2*d3^2 + 2*d3*(d1 + d2)*cos(th3_t) + (d1 + d2 + d3)*(d1 + d2 - d3))*m2*th1d*cos(th2_t)*sin(th2_t) - 2*th1d*cos(th2_t)^2*sin(th3_t)*d3*m2*(cos(th3_t)*d3 + d1 + d2) + 2*th1d*sin(th2_t)^2*sin(th3_t)*d3*m2*(cos(th3_t)*d3 + d1 + d2))*th2d + ((-4*cos(th3_t)*d3^2*sin(th3_t) - 2*d3*(d1 + d2)*sin(th3_t))*m2*th1d*cos(th2_t)^2 - 2*th1d*sin(th2_t)*cos(th3_t)*d3*m2*(cos(th3_t)*d3 + d1 + d2)*cos(th2_t) + 2*th1d*sin(th2_t)*sin(th3_t)^2*d3^2*m2*cos(th2_t) + 2*cos(th3_t)*d3^2*m2*th1d*sin(th3_t))*th3d + (m1*cos(th2_t)^2*d1^2 + I2zz + (2*cos(th3_t)^2*d3^2 + 2*d3*(d1 + d2)*cos(th3_t) + (d1 + d2 + d3)*(d1 + d2 - d3))*m2*cos(th2_t)^2 - 2*sin(th2_t)*sin(th3_t)*d3*m2*(cos(th3_t)*d3 + d1 + d2)*cos(th2_t) - cos(th3_t)^2*d3^2*m2 + d3^2*m2 + I3zz)*th1dd + (((I2yz + I2zy)*cos(th1_t))/2 - ((I2zx + I2xz)*sin(th1_t))/2 - ((-I3yz - I3zy)*cos(th1_t))/2 - ((I3xz + I3zx)*sin(th1_t))/2)*th2dd + (-((-I3yz - I3zy)*cos(th1_t))/2 - ((I3xz + I3zx)*sin(th1_t))/2)*th3dd - th2d^2*(I2xx - I2yy)*cos(th1_t)*sin(th1_t) + th2d^2*(I2xy + I2yx)*cos(th1_t)^2/2 + th2d*(-th2d*(I2xy + I2yx)*sin(th1_t) + th1d*(I2yz + I2zy))*sin(th1_t)/2 + th1d*th2d*(I2zx + I2xz)*cos(th1_t)/2 + (th2d + th3d)^2*(I3yy - I3xx)*cos(th1_t)*sin(th1_t) + (th2d + th3d)^2*(I3yx + I3xy)*cos(th1_t)^2/2 - ((th2d + th3d)*((th2d + th3d)*(I3yx + I3xy)*sin(th1_t) - th1d*(I3yz + I3zy))*sin(th1_t))/2 + th1d*(th2d + th3d)*(I3xz + I3zx)*cos(th1_t)/2

Tau2 = (2*th2d*(I2xx - I2yy)*cos(th1_t)*sin(th1_t) - th2d*(I2xy + I2yx)*cos(th1_t)^2 - ((-th2d*(I2xy + I2yx)*sin(th1_t) + th1d*(I2yz + I2zy))*sin(th1_t))/2 + th2d*(I2xy + I2yx)*sin(th1_t)^2/2 - th1d*(I2zx + I2xz)*cos(th1_t)/2 - 2*(th2d + th3d)*(I3yy - I3xx)*cos(th1_t)*sin(th1_t) - (th2d + th3d)*(I3yx + I3xy)*cos(th1_t)^2 + (((th2d + th3d)*(I3yx + I3xy)*sin(th1_t) - th1d*(I3yz + I3zy))*sin(th1_t))/2 + ((th2d + th3d)*(I3yx + I3xy)*sin(th1_t)^2)/2 - th1d*(I3xz + I3zx)*cos(th1_t)/2)*th1d + (((I2yz + I2zy)*cos(th1_t))/2 - ((I2zx + I2xz)*sin(th1_t))/2 - ((-I3yz - I3zy)*cos(th1_t))/2 - ((I3xz + I3zx)*sin(th1_t))/2)*th1dd + (-(I2xx - I2yy)*cos(th1_t)^2 - (I2xy + I2yx)*sin(th1_t)*cos(th1_t) + I2xx + (I3yy - I3xx)*cos(th1_t)^2 - (I3yx + I3xy)*sin(th1_t)*cos(th1_t) + d3^2*m2 + I3xx)*th2dd + ((I3yy - I3xx)*cos(th1_t)^2 - (I3yx + I3xy)*sin(th1_t)*cos(th1_t) + I3xx)*th3dd + m1*th1d^2*cos(th2_t)*d1^2*sin(th2_t) + (2*cos(th3_t)^2*d3^2 + 2*d3*(d1 + d2)*cos(th3_t) + (d1 + d2 + d3)*(d1 + d2 - d3))*m2*th1d^2*cos(th2_t)*sin(th2_t) + th1d^2*cos(th2_t)^2*sin(th3_t)*d3*m2*(cos(th3_t)*d3 + d1 + d2) - th1d^2*sin(th2_t)^2*sin(th3_t)*d3*m2*(cos(th3_t)*d3 + d1 + d2) + m1*G2*cos(th2_t)*d1 + m2*G3*(cos(th2_t)*(d1 + d2) + (-sin(th2_t)*sin(th3_t) + cos(th2_t)*cos(th3_t))*d3)

Tau3 = (-2*(th2d + th3d)*(I3yy - I3xx)*cos(th1_t)*sin(th1_t) - (th2d + th3d)*(I3yx + I3xy)*cos(th1_t)^2 + (((th2d + th3d)*(I3yx + I3xy)*sin(th1_t) - th1d*(I3yz + I3zy))*sin(th1_t))/2 + ((th2d + th3d)*(I3yx + I3xy)*sin(th1_t)^2)/2 - th1d*(I3xz + I3zx)*cos(th1_t)/2)*th1d + (-((-I3yz - I3zy)*cos(th1_t))/2 - ((I3xz + I3zx)*sin(th1_t))/2)*th1dd + ((I3yy - I3xx)*cos(th1_t)^2 - (I3yx + I3xy)*sin(th1_t)*cos(th1_t) + I3xx)*th2dd + ((I3yy - I3xx)*cos(th1_t)^2 - (I3yx + I3xy)*sin(th1_t)*cos(th1_t) + I3xx)*th3dd - ((-4*cos(th3_t)*d3^2*sin(th3_t) - 2*d3*(d1 + d2)*sin(th3_t))*m2*th1d^2*cos(th2_t)^2)/2 + th1d^2*sin(th2_t)*cos(th3_t)*d3*m2*(cos(th3_t)*d3 + d1 + d2)*cos(th2_t) - th1d^2*sin(th2_t)*sin(th3_t)^2*d3^2*m2*cos(th2_t) - cos(th3_t)*d3^2*m2*th1d^2*sin(th3_t) + m2*G3*(-sin(th2_t)*sin(th3_t) + cos(th2_t)*cos(th3_t))*d3

