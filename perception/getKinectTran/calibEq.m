%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Generate derivative code for Jacobian and Hessian stuff

syms tx ty tz ra pa ya 

% Local Variables
syms nx ny nz ox oy oz ax ay az

% Input per each point (point in Kinect and point in robot)
syms xk yk zk
syms xr yr zr

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

Fx = (nx*xk + ox*yk + ax*zk + tx) - xr;
Fy = (ny*xk + oy*yk + ay*zk + ty) - yr;
Fz = (nz*xk + oz*yk + az*zk + tz) - zr;

F = Fx*Fx + Fy*Fy + Fz*Fz;

% Jacobian
J = [ diff(F,ra), diff(F,pa), diff(F,ya), diff(F,tx), diff(F,ty), diff(F,tz) ];


% C Code Generation
ccode(F,'file','func_Eq.txt');
ccode(J,'file','jac_Eq.txt');

