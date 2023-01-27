function op=attitude_control(u)
global I tuning_parameter
k1=55;
k2=55;
k3=55;

Ix=I(1,1);
Iy=I(2,2);
Iz=I(3,3);

dphi=u(1);
dtht=u(2);
dpsi=u(3);

s1=u(4);
s2=u(5);
s3=u(6);

ism = [(Iy-Iz)/Ix*dpsi*dtht;
        (Iz-Ix)/Iy*dpsi*dphi; 
        (Ix-Iy)/Iz*dphi*dtht];

asm = ism + tuning_parameter*[dphi; dtht; dpsi];
K = diag([k1, k2, k3]);

beta = inv(I);

op  = beta \ (-asm - K*sign([s1; s2; s3]));
end

