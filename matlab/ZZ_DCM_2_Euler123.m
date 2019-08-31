function psi=ZZ_DCM_2_Euler123(DCM)
% (X,Y,Z)  -----------------------> (x,y,z)
% Euler angle psi=(alp,bet,gam) in 123 convention.
% A vector expressed in (x,y,z) as v, and expressed in (X,Y,Z) as V.
% v=DCM*V.
% omega=P_psi(psi)*[d_alp;d_bet;d_gam];

% input: psi, convention
% output: DCM, P_psi

% % Euler angle to DCM
% psi=30*randn(3,1)/180*pi
% ax=psi(1);ay=psi(2);az=psi(3);
% 
% cx=cos(ax);sx=sin(ax);
% cy=cos(ay);sy=sin(ay);
% cz=cos(az);sz=sin(az);
%  
% DCM =[
%   cy*cz, cx*sz + cz*sx*sy, sx*sz - cx*cz*sy
%  -cy*sz, cx*cz - sx*sy*sz, cz*sx + cx*sy*sz
%      sy,           -cy*sx,            cx*cy];
%  
% DCM

% DCM to Euler angle
% DCM=orth(rand(3,3))

psi=zeros(3,1);
psi(2)=asin(DCM(3,1));
psi(1)=-atan(DCM(3,2)/DCM(3,3));
psi(3)=-atan(DCM(2,1)/DCM(1,1));
% psi

