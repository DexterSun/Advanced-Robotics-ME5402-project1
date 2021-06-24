function rot = DH_rot(joint)

alpha = joint.DH(1);
a = joint.DH(2);
d = joint.DH(3);
theta = joint.DH(4);

rot = [cos(theta)  ,  -sin(theta)*cos(alpha)  ,  sin(theta)*sin(alpha)  ,  a*cos(theta);
       sin(theta)  ,   cos(theta)*cos(alpha)  , -cos(theta)*sin(alpha)  ,  a*sin(theta);
           0       ,         sin(alpha)       ,          cos(alpha)     ,      d       ;
           0       ,             0            ,              0          ,      1       ;];
    