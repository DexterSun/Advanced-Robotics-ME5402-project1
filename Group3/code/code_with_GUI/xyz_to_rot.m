function rot = xyz_to_rot(gamma,beta,alpha)
alpha = alpha/180*pi;
beta = beta/180*pi;
gamma = gamma/180*pi;
rot = [cos(alpha)*cos(beta) , cos(alpha)*sin(beta)*sin(gamma)-sin(alpha)*cos(gamma) , cos(alpha)*sin(beta)*cos(gamma)+sin(alpha)*sin(gamma);...
       sin(alpha)*cos(beta) , sin(alpha)*sin(beta)*sin(gamma)+cos(alpha)*cos(gamma) , sin(alpha)*sin(beta)*cos(gamma)-cos(alpha)*sin(gamma);...
             -sin(beta)     ,                      cos(beta)*sin(gamma)             ,                      cos(beta)*cos(gamma)           ];