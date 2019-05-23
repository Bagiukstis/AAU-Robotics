%Assigned values of UR5
alpha1= 0; alpha2= 90*pi/180; alpha3= 0; alpha4= 0; alpha5= 90*pi/180; alpha6= -90*pi/180;
a1= 0; a2= 0; a3= -425.00 ; a4= -392.25; a5= 0; a6= 0;
d1= 89.459; d2= 0; d3= 0; d4= 109.15; d5= 94.65; d6= 82.3;

%DH Parameters
L1=Link('d',d1 ,'a',  0,'alpha',  0,       'modified');
L2=Link('d',  0,'a', 0 ,'alpha', alpha2,'modified');
L3=Link('d',  0,'a', a3 ,'alpha',  0,       'modified');
L4=Link('d',d4 ,'a', a4 ,'alpha', 0,'modified');
L5=Link('d',d5 ,'a',  0,'alpha', alpha5,'modified');
L6=Link('d',d6 ,'a',  0,'alpha',alpha6 ,'modified');

%Forward Kinematics
UR5=SerialLink([L1 L2 L3 L4 L5 L6], 'name', 'UR5');

q=[10 20 30 40 50 60]*pi/180; %Join positions (converting degrees to radians)

T06= UR5.fkine(q);

%Inverse Kinematics

%theta1
P05= T06*[0;0;-d6;1];
P05x= P05(1);
P05y= P05(2);

fi1= atan2(P05y,P05x);
fi2= acos(d4/sqrt((P05x)^2 + (P05y)^2));

theta1= fi1+fi2+pi/2;

%theta5

theta5= acos((T06(1,4)*sin(theta1)-T06(2,4)*cos(theta1)-d4)/d6);

%theta6
T60= inv(T06);

theta6= atan2((-T60(2,1)*sin(theta1) + T60(2,2)*cos(theta1))/sin(theta5), (T60(1,1)*sin(theta1)-T60(1,2)*cos(theta1))/sin(theta5));

%theta 3 and 2
UR14=SerialLink([L2 L3 L4], 'name', 'UR5');
z=[20 30 40]*pi/180;
T14= UR14.fkine(z);

theta3= acos(((T14(1,4)^2)+(T14(3,4)^2)-(a3)^2-(a4)^2)/(2*a3*a4));

theta2= atan2((-T14(3,4)), (-T14(1,4)))-asin((sin(theta3)*(-a4))/sqrt(((-T14(3,4))^2)+((-T14(1,4))^2)));

%theta 4
UR12 = SerialLink([L2], 'name', 'UR5');
z=[20]*pi/180;
T12= UR12.fkine(z);

UR23 = SerialLink([L3], 'name', 'UR5');
z=[30]*pi/180;
T23= UR23.fkine(z);

T31g= T12*T23;
T31= inv(T31g);
T34= T31*T14;

theta4= atan2(T34(2,1), T34(1,1));
      
%Converting radians to degrees:

theta1deg= theta1*180/pi
theta2deg= theta2*180/pi
theta3deg= theta3*180/pi
theta4deg= theta4*180/pi
theta5deg= theta5*180/pi
theta6deg= theta6*180/pi

