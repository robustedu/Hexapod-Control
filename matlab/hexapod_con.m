clear all force; close all; clc
% hexapod_con_pre
% break
load exam_con1

% Notation:
% base platform: (w)           moving platform: (u)
% head: (h)                    radius: (r)                 joint: (j)
% desired: (de)                begin: (be)
% translation: (xyz)           rotation -- Euler angle: (psi) in the 123 convention, i.e., psi = (pitch,roll,yaw)

control.all_algorithms={
    'operational-S plan',
    'operational-D plan',
    'target-S plan',
    'target-D plan'
    };

control.algorithm_case = 4;
control.algorithm_used = control.all_algorithms{control.algorithm_case};

%% Generate target.xyz/psi_in_MOVING based on a given reasonable target.xyz/psi
target.xyz = zeros(3,1); target.psi = zeros(3,1);
[xyz_in_MOVING0,psi_in_MOVING0] = generate_target_in_MOVING(target, hexapod, couch);

target.xyz = [4;3;5]/1000; target.psi = [1;2;-1.5]/180*pi;
t1 = (rand(3,1)-0.5)*8; t2 = (rand(3,1)-0.5)*4;
% save xyz_psi t1 t2
load xyz_psi
target.xyz = t1/1000; target.psi =  t2/180*pi;
[xyz_in_MOVING,psi_in_MOVING] = generate_target_in_MOVING(target, hexapod, couch);
NNN = length(control.T);
target.xyz_in_MOVING_all = repmat(xyz_in_MOVING,1,NNN);
target.psi_in_MOVING_all = repmat(psi_in_MOVING,1,NNN);

%% compute initial hexapod.moving.xyz/psi
Q.couch = ZZ_Euler_2_DCM_P123(couch.psi);
Q.moving_in_COUCH = ZZ_Euler_2_DCM_P123(hexapod.moving.psi_in_COUCH);
Q.moving = Q.moving_in_COUCH*Q.couch;
hexapod.moving.psi = Q2psi(Q.moving);
hexapod.moving.xyz = couch.xyz+Q.couch'*hexapod.moving.xyz_in_COUCH;

tim=tic;
for iii = 1:NNN
  
    % In real control, target.xyz/psi is the measurement of head position.
    % Here, we compute target.xyz/psi from target.xyz/psi_in_MOVING
    target.xyz_in_MOVING = target.xyz_in_MOVING_all(:,iii); target.psi_in_MOVING = target.psi_in_MOVING_all(:,iii);     
    
    % For iii>1, the updated hexapod.moving.xyz/psi from controller algorithm will be used.
    Q.moving = ZZ_Euler_2_DCM_P123(hexapod.moving.psi);
    Q.target_in_MOVING = ZZ_Euler_2_DCM_P123(target.psi_in_MOVING);
    Q.target = Q.target_in_MOVING*Q.moving;
    target.psi = Q2psi(Q.target);    
    target.xyz = hexapod.moving.xyz+Q.moving'*target.xyz_in_MOVING;
    
    %% Optical IR input: target.xyz/psi     
    
    % remove target.xyz/psi_in_MOVING, since it is not an input in real control. JUST make sure we do not use these variables in control algorithm.
    t1 = {'xyz_in_MOVING','psi_in_MOVING'}; target = rmfield(target, t1);
    
    %% start of control algorithm (compute next hexapod.moving.xyz/psi)
    [hexapod, con_on]= hex_controller(hexapod, target, control);
    
    % hexapod leg length
    t1 = repmat(hexapod.moving.xyz,1,6)+ZZ_Euler_2_DCM_P123(hexapod.moving.psi)'*hexapod.j_u2; % the joints position of the moving plate 
    ttt1 = repmat(hexapod.base.xyz_in_COUCH,1,6)+ZZ_Euler_2_DCM_P123(hexapod.base.psi_in_COUCH)'*hexapod.j_w2; 
    t2 = repmat(couch.xyz,1,6)+ZZ_Euler_2_DCM_P123(couch.psi)'*ttt1; % the joints position of the base plate 
    tt = t2-t1;
    hexapod.leg_length = sqrt(tt(1,:).*tt(1,:)+tt(2,:).*tt(2,:)+tt(3,:).*tt(3,:));    
    
    % save for plot and record.
    target.xyz_all(:,iii) = target.xyz; target.psi_all(:,iii) = target.psi;    
    hexapod.moving.xyz_all(:,iii) = hexapod.moving.xyz;
    hexapod.moving.psi_all(:,iii) = hexapod.moving.psi;
    hexapod.leg_length_all(:,iii) = hexapod.leg_length;
     
end
toc(tim)

plot_figs
