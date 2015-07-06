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

% Normalized x,y,c
xi = (nx*x + ny*y + nz*z - px*nx - py*ny - pz*nz);
yi = (ox*x + oy*y + oz*z - px*ox - py*oy - pz*oz);
zi = (ax*x + ay*y + az*z - px*ax - py*ay - pz*az);

F_simple = ( ( (xi/a)^(2))^(1.0/e2) + ( (yi/b)^(2))^(1.0/e2) )^(e2 / e1) + ( (zi/c)^(2))^(1.0/e1);    
F = F_simple^(e1/2);

% Radial: Distance from points to approximation
Fr = sqrt(xi^2 + yi^2 + zi^2)*abs( 1 - 1.0/F );

% Duncan's paper (I think this is mainly Solina)
Fs = sqrt(a*b*c)*(F_simple^e1 - 1);

% Ichim
Fi = sqrt(a*b*c)*sqrt(xi^2 + yi^2 + zi^2)*(F_simple^e1-1);

% Chevalier
Fc = sqrt(xi^2 + yi^2 + zi^2)*(F-1);

% Option 5
F5 = sqrt(xi^2 + yi^2 + zi^2)*(F_simple^e1-1);

% Option 6
F6 = (1-1/F)*sqrt(xi^2 + yi^2 + zi^2)*(F_simple^e1-1);

% Different error measures
Er1 = (1 - F_simple)^2;
Er2 = (1 - F_simple^e1)^2;
Er4 = sqrt(xi^2 + yi^2 + zi^2)*abs(1 - 1.0/F);

% Jacobian
Jr = [ diff(Fr,a), diff(Fr,b), diff(Fr,c), diff(Fr,e1), diff(Fr,e2), diff(Fr,px), diff(Fr,py), diff(Fr,pz), diff(Fr,ra), diff(Fr,pa), diff(Fr,ya) ];
Js = [ diff(Fs,a), diff(Fs,b), diff(Fs,c), diff(Fs,e1), diff(Fs,e2), diff(Fs,px), diff(Fs,py), diff(Fs,pz), diff(Fs,ra), diff(Fs,pa), diff(Fs,ya) ];
Ji = [ diff(Fi,a), diff(Fi,b), diff(Fi,c), diff(Fi,e1), diff(Fi,e2), diff(Fi,px), diff(Fi,py), diff(Fi,pz), diff(Fi,ra), diff(Fi,pa), diff(Fi,ya) ];
Jc = [ diff(Fc,a), diff(Fc,b), diff(Fc,c), diff(Fc,e1), diff(Fc,e2), diff(Fc,px), diff(Fc,py), diff(Fc,pz), diff(Fc,ra), diff(Fc,pa), diff(Fc,ya) ];
J5 = [ diff(F5,a), diff(F5,b), diff(F5,c), diff(F5,e1), diff(F5,e2), diff(F5,px), diff(F5,py), diff(F5,pz), diff(F5,ra), diff(F5,pa), diff(F5,ya) ];
J6 = [ diff(F6,a), diff(F6,b), diff(F6,c), diff(F6,e1), diff(F6,e2), diff(F6,px), diff(F6,py), diff(F6,pz), diff(F6,ra), diff(F6,pa), diff(F6,ya) ];


% C Code Generation
ccode(Fr,'file','Fr.txt');
ccode(Jr,'file','Jr.txt');

ccode(Fs,'file','Fs.txt');
ccode(Js,'file','Js.txt');

ccode(Fi,'file','Fi.txt');
ccode(Ji,'file','Ji.txt');

ccode(Fc,'file','Fc.txt');
ccode(Jc,'file','Jc.txt');

ccode(F5,'file','F5.txt');
ccode(J5,'file','J5.txt');

ccode(F6,'file','F6.txt');
ccode(J6,'file','J6.txt');


ccode(Er1,'file','Er1.txt');
ccode(Er2,'file','Er2.txt');
%ccode(Er3,'file','Er3.txt');
ccode(Er4,'file','Er4.txt');

