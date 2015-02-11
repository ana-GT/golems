%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Generate derivative code for Jacobian and Hessian stuff

syms a b c e1 e2 px py pz ra pa ya 

% Local Variables
syms nx ny nz ox oy oz ax ay az

% Input per each point
syms x y z

% A very coarse way to enter the rotation matrix
nx = cos(ya)*cos(pa);
ny = sin(ya)*cos(pa);
nz = -sin(pa);

ox = cos(ya)*sin(pa)*sin(ra) - sin(ya)*cos(ra);
oy = sin(ya)*sin(pa)*sin(ra) + cos(ya)*cos(ra);
oz = cos(pa)*sin(ra);

ax = cos(ya)*sin(pa)*cos(ra) + sin(ya)*sin(ra);
ay = sin(ya)*sin(pa)*cos(ra) - cos(ya)*sin(ra);
az = cos(pa)*cos(ra);


% Equation of F with no translation/rotation considered
%F = ( ( (x/a)^(2))^(1.0/e2) + ( (y/b)^(2))^(1.0/e2) )^(e2 / e1) + ( (z/c)^(2))^(1.0/e1);    

F = ( ( ( (nx*x + ny*y + nz*z - px*nx - py*ny - pz*nz)/a  )^(2))^(1.0/e2) + ( ( (ox*x + oy*y + oz*z - px*ox - py*oy - pz*oz)/b  )^(2))^(1.0/e2) )^(e2 / e1) + ( ( (ax*x + ay*y + az*z - px*ax - py*ay - pz*az)/c  )^(2))^(1.0/e1);    

Fm = sqrt(a*b*c)*(F^e1-1);

Er = ( F^e1 - 1 )^2;

% Jacobian
J = [ diff(Fm,a), diff(Fm,b), diff(Fm,c), diff(Fm,e1), diff(Fm,e2), diff(Fm,px), diff(Fm,py), diff(Fm,pz), diff(Fm,ra), diff(Fm,pa), diff(Fm,ya) ];


% C Code Generation
ccode(Fm,'file','func_Eq.txt');
ccode(J,'file','jac_Eq.txt');

