function XYZ = rot_to_XYZ(R)
r11 = R(1,1); r12 = R(1,2); r13 =R(1,3);
r21 = R(2,1); r22 = R(2,2); r23 = R(2,3);
r31 = R(3,1); r32 = R(3,2); r33 = R(3,3);
% since there are 2 solns in general,
% we seperate 2 betas into beta1 and beta2
beta1 = atan2(-r31, sqrt(r11^2+r21^2));
beta2 = atan2(-r31,-sqrt(r11^2+r21^2));
alpha1 = atan2(r21/cos(beta1),r11/cos(beta1));
alpha2 = atan2(r21/cos(beta2),r11/cos(beta2));
gamma1 = atan2(r32/cos(beta1),r33/cos(beta1));
gamma2 = atan2(r32/cos(beta2),r33/cos(beta2));
XYZ = round([gamma1,beta1,alpha1;...
       gamma2,beta2,alpha2]./pi.*180);
