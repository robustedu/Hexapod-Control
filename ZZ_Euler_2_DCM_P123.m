function [DCM,P_psi]=ZZ_Euler_2_DCM_P123(psi)

% (X,Y,Z)  -----------------------> (x,y,z)
% Euler angle psi=(alp,bet,gam) in 123 convention.
% A vector expressed in (x,y,z) as v, and expressed in (X,Y,Z) as V.
% v=DCM*V.
% omega=P_psi(psi)*[d_alp;d_bet;d_gam];

% input: psi, convention
% output: DCM, P_psi

ax=psi(1);ay=psi(2);az=psi(3);

cx=cos(ax);sx=sin(ax);
cy=cos(ay);sy=sin(ay);
cz=cos(az);sz=sin(az);
 
DCM =[
  cy*cz, cx*sz + cz*sx*sy, sx*sz - cx*cz*sy
 -cy*sz, cx*cz - sx*sy*sz, cz*sx + cx*sy*sz
     sy,           -cy*sx,            cx*cy];
 
P_psi =[
  cy*cz, sz, 0
 -cy*sz, cz, 0
     sy,  0, 1];
