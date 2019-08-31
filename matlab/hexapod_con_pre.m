% 12:45 pm, 11/03   
close all; clear all; clc
% Notation:
% base platform: (w)           moving platform: (u)
% head: (h)                    radius: (r)                 joint: (j)
% desired: (de)                begin: (be)                 next:  (nx)
% translation: (xyz)           rotation -- Euler angle: (psi) in the 123 convention, i.e., psi = (pitch,roll,yaw)
CONVERSION = 'DCMtoEA123';

% Q2psi = @(x) (SpinCalc(CONVERSION,x,1e-10,1)'-round(SpinCalc(CONVERSION,x,1e-10,1)'/360)*360)/180*pi;
% Q2psi = @(x) mod(SpinCalc(CONVERSION,x,1e-10,1)',360)/180*pi;
% Q2psi = @(x)  SpinCalc(CONVERSION,x,1e-10,1)'/180*pi;
% Q2psi = @(x) ZZ_DCM_2_Euler123(x);
% Q2psi = @(x) ZZ_DCM_2_Euler123(x)-round(ZZ_DCM_2_Euler123(x)/2/pi)*2*pi;

% LINAC frame; couch frame; hexapod moving panel frame; target frame.
%% setting
hexapod.configuration =1;


%% load data of skull, neck and torso
% load Nancy_data
load Nancy_structure_data
Nancy.pivotA = Nancy.pivot1_position; Nancy.pivotB = Nancy.pivot2_position; % pivotA: head and neck; pivotA: neck and torso.
Nancy.torso_z_min = min(Nancy.torso.point(3,:)); Nancy.skull.z_min = min(Nancy.skull.point(3,:));

%% Mechanical parameters
hexapod.actuator_width = 0.03; hexapod.plate_thick = 0.007;

hexapod.joint_space = 10.95; % degree. The angle between two close joints.
hexapod.r_base = 0.075; hexapod.r_moving = hexapod.r_base; hexapod.r_j = 0.006;  % origin
hexapod.j_posi = 0.06329; hexapod.j_high = 0.0115; % origin

hexapod.joint_space = 13; % degree. The angle between two close joints.
hexapod.r_base = 0.083; hexapod.r_moving = hexapod.r_base; hexapod.r_j = 0.007;
hexapod.j_posi = 0.07; hexapod.j_high = 0.009; %?

hexapod.joint_space = 10.95; % degree. The angle between two close joints.
hexapod.r_base = 0.087; hexapod.r_moving = hexapod.r_base; hexapod.r_j = 0.007;
hexapod.j_posi = 0.075; hexapod.j_high = 0.009; %?

couch.top_thick = 0.02; couch.width = 0.54; ground_z = -1.3;
head_support.length = 0.28; head_support.width = 0.3;
pillow.length = 0.18; pillow.width = 0.18;
connect.couch_base = [hexapod.r_base*2; 0.03; 0.007];
head_support.couch_gap = 0.06;
pillow.couch_gap = 0.1;


% hexapod.moving_bottom_to_couch_max=-1000*ones(1,3); hexapod.moving_bottom_to_couch_min=1000*ones(1,3);
% hexapod.moving_bottom_to_isocenter_max=-1000*ones(1,3); hexapod.moving_bottom_to_isocenter_min=1000*ones(1,3);
% hexapod.leg_length_max=-1000*ones(1,6); hexapod.leg_length_min=1000*ones(1,6); hexapod.leg_force_max=0;
% hexapod.moving.xyz_in_COUCH_max = -1000*ones(1,3); hexapod.moving.xyz_in_COUCH_min =  1000*ones(1,3);
% hexapod.moving.psi_in_COUCH_max = -1000*ones(1,3); hexapod.moving.psi_in_COUCH_min =  1000*ones(1,3);
% hexapod.leg_force_max = -1000*ones(1,6); hexapod.leg_force_min = 1000*ones(1,6);

%% Bed and hexapod: set 1) the normal height of the hexapod; 2) the height of the couch. Base on 1) and 2), determine the thickness of the piollow.
hexapod.normal_high = 0.145; % from joint surface to joint surface
hexapod.normal_high = 0.197; % from joint surface to joint surface % for 50mm stroke. for leg length 175+25
% hexapod.normal_high = 0.217; %% [u_xyz,u_psi,err_t,ss]=ZZ_leg_length_to_6D_123(Lt_tar,u_xyz,u_psi,j_w,hexapod.j_u2,err_threshold)

Nancy.pivotB_y_in_COUCH_additional = 0.23; % for moving patient back setup.


i2=0;
% for et1=tmp.workspace*5, for et2=tmp.workspace*5, for et3=tmp.workspace*5, for et4=tmp.workspace*2.5, for et5=tmp.workspace*2.5, for et6=tmp.workspace*2.5

couch.xyz_normal = [0;-0.25;-0.15]; couch.psi_normal = [0;0;0*pi/6];
couch.psi = couch.psi_normal; couch.xyz = couch.xyz_normal;

switch hexapod.configuration
    case {1,4,7}
        hexapod.base_to_couch_edge = 0.2; % hexbase edge to the couch edge, y direction
        hexapod.base_to_couch_edge = 0.2-0.15; % hexbase edge to the couch edge, y direction
        if hexapod.configuration == 7, 
            hexapod.base_to_couch_edge = hexapod.base_to_couch_edge - Nancy.pivotB_y_in_COUCH_additional; 
        end
        hexapod.base.psi_in_COUCH = [0;0;0]; hexapod.base.xyz_in_COUCH = [0;-hexapod.r_base-hexapod.base_to_couch_edge;-couch.top_thick-hexapod.plate_thick-connect.couch_base(3)-hexapod.normal_high-2*hexapod.j_high-0.03];
        hexapod.moving.xyz_in_COUCH_normal = hexapod.base.xyz_in_COUCH+[0;0;hexapod.normal_high+2*hexapod.j_high];
        hexapod.moving.psi_in_COUCH_normal = [-pi;0;0];
        nt = 20; tt = linspace(0,4*pi,nt); 
    case {2,5,8}
        hexapod.base_to_couch_edge = 0.22-0.15; % hexbase edge to the couch edge, y direction
        hexapod.base.psi_in_COUCH = [pi;0;0]; hexapod.base.xyz_in_COUCH = [0;-hexapod.r_base-hexapod.base_to_couch_edge;-couch.top_thick-hexapod.plate_thick-connect.couch_base(3)];
        hexapod.moving.xyz_in_COUCH_normal = hexapod.base.xyz_in_COUCH+[0;0;-hexapod.normal_high-2*hexapod.j_high];
        hexapod.moving.psi_in_COUCH_normal = [0;0;0];
        nt = 120; tt = linspace(0,26*pi,nt);
    case {3,6,9}
        hexapod.base_to_couch_edge = 0.13-0.07; % hexbase edge to the couch edge, y direction
        hexapod.base.psi_in_COUCH = [-pi/2;0;0]; hexapod.base.xyz_in_COUCH = [0;-hexapod.base_to_couch_edge-2*hexapod.j_high-hexapod.plate_thick-hexapod.normal_high-0.02;-couch.top_thick-hexapod.r_base-connect.couch_base(2)];
        hexapod.moving.xyz_in_COUCH_normal = [0;hexapod.base.xyz_in_COUCH(2)+hexapod.normal_high+2*hexapod.j_high;hexapod.base.xyz_in_COUCH(3)];
        hexapod.moving.psi_in_COUCH_normal = [pi/2;0;0];
        nt = 60; tt = linspace(0,16*pi,nt);
end

hexapod.moving.xyz_in_COUCH = hexapod.moving.xyz_in_COUCH_normal;
hexapod.moving.psi_in_COUCH = hexapod.moving.psi_in_COUCH_normal;
helix_x = 1*hexapod.plate_thick*cos(tt); helix_y = 1*hexapod.plate_thick*sin(tt);

%% Joints in the base and moving panels
hexapod.j_w_direction = [0 120 240]+90; % low panel joints direction
t1 = hexapod.j_w_direction; 
t3 = pi/180*(t1([1 1 2 2 3 3])+[-1 1 -1 1 -1 1]*hexapod.joint_space);
t4 = hexapod.j_posi*cos(t3); t5 = hexapod.j_posi*sin(t3);
hexapod.j_w2 = [t4; t5; ones(1,6)*hexapod.j_high]; hexapod.c_w2 = [t4; t5; -ones(1,6)*hexapod.plate_thick];
t4 = hexapod.r_base*cos(t3); t5 = hexapod.r_base*sin(t3);
hexapod.base2 = [t4, t4; t5, t5; zeros(1,6), -ones(1,6)*hexapod.plate_thick];
t7=pi/180*[hexapod.j_w_direction(1), hexapod.j_w_direction(1)+180];
t4 = (hexapod.j_posi+hexapod.r_j)*cos(t7); t5 = (hexapod.j_posi+hexapod.r_j)*sin(t7);hexapod.base.spring_add = [t4;t5;zeros(1,2)];

t1 = hexapod.j_w_direction; t1 = t1(3:-1:1);
t3 = pi/180*(t1([1 1 2 2 3 3])-[-1 1 -1 1 -1 1]*hexapod.joint_space);
t3 = t3([2:6,1]); t4 = hexapod.j_posi*cos(t3); t5 = hexapod.j_posi*sin(t3);
hexapod.j_u2 = [t4; t5; ones(1,6)*hexapod.j_high]; hexapod.c_u2 = [t4; t5; -ones(1,6)*hexapod.plate_thick]; % joint positions of the top of the panel.
t4 = hexapod.r_moving*cos(t3); t5 = hexapod.r_moving*sin(t3);
hexapod.moving2 = [t4, t4; t5, t5; zeros(1,6), -ones(1,6)*hexapod.plate_thick];
t7=pi/180*[t1(1)-60, t1(1)-60+180];
t4 = (hexapod.j_posi+hexapod.r_j)*cos(t7); t5 = (hexapod.j_posi+hexapod.r_j)*sin(t7); hexapod.moving.spring_add = [t4;t5;zeros(1,2)];

Nancy.torso_z_min = min(Nancy.torso.point(3,:)); Nancy.skull.z_min = min(Nancy.skull.point(3,:));
Nancy.pivotB_y_in_COUCH = -0.04; % is the y coordinate of Nancy.pivotB in the couch frame.


% seperate neck points into two parts. One is in skull, the other is in torso.
tt = Nancy.neck.point(2,:); Nancy.neck_set_I_index = find(tt>1.48); Nancy.neck_set_II_index = find(tt<1.48);
Nancy.neck_set_I = Nancy.neck.point(:,Nancy.neck_set_I_index); Nancy.neck_set_II = Nancy.neck.point(:,Nancy.neck_set_II_index); 
% pt(:,Nancy.neck_set_I_index) = Nancy.neck_set_I; pt(:,Nancy.neck_set_II_index) = Nancy.neck_set_II;

% find diff_body_couch such tha Nancy.pivotB in couch coordinates is [0;Nancy.pivotB_y_in_COUCH;Nancy.pivotB(3)-Nancy.torso_z_min]
% That is, Nancy.pivotB+diff_body_couch = [0;Nancy.pivotB_y_in_COUCH;Nancy.pivotB(3)-Nancy.torso_z_min];
diff_body_couch = -Nancy.pivotB+[0;Nancy.pivotB_y_in_COUCH;Nancy.pivotB(3)-Nancy.torso_z_min];
Nancy.pivotB_in_COUCH = Nancy.pivotB+diff_body_couch;
tt = Nancy.torso.point; Nancy.torso.point_in_COUCH = tt+repmat(diff_body_couch,1,size(tt,2));
tt = Nancy.neck_set_II; Nancy.neck_set_II_in_COUCH = tt+repmat(diff_body_couch,1,size(tt,2));
tt = Nancy.others.point; Nancy.others.point_in_COUCH = tt+repmat(diff_body_couch,1,size(tt,2));

% The normal points coordinates in the hexapod moving panel frame.
t1 = ZZ_Euler_2_DCM_P123(hexapod.moving.psi_in_COUCH);
tt = Nancy.skull.point; tt1 = size(tt,2); Nancy.skull.point_in_COUCH = tt+repmat(diff_body_couch,1,tt1); Nancy.skull.point_in_MOVING_be = t1*(Nancy.skull.point_in_COUCH-repmat(hexapod.moving.xyz_in_COUCH,1,tt1));
tt = Nancy.neck_set_I; tt1 = size(tt,2); Nancy.neck_set_I_in_COUCH = tt+repmat(diff_body_couch,1,tt1); Nancy.neck_set_I_in_MOVING_be = t1*(Nancy.neck_set_I_in_COUCH-repmat(hexapod.moving.xyz_in_COUCH,1,tt1));
tt = Nancy.pivotA; tt1 = size(tt,2); Nancy.pivotA_in_COUCH = tt+repmat(diff_body_couch,1,size(tt,2)); Nancy.pivotA_in_MOVING_be = t1*(Nancy.pivotA_in_COUCH-repmat(hexapod.moving.xyz_in_COUCH,1,tt1));

%% define the position of tumor target.
target.xyz_in_BODY = Nancy.pivotA + [0; 80; 50]/1000 + 0*(2*rand(3,1)-1)/1000;

tt = target.xyz_in_BODY; target.xyz_in_COUCH = tt+repmat(diff_body_couch,1,size(tt,2));

% target.xyz_in_COUCH=[0; hexapod.moving.xyz_in_COUCH_normal(2)+0.04; 0.1]; % move CM of the target on the top of hexapod center
target.xyz_in_MOVING_normal = t1*(target.xyz_in_COUCH-repmat(hexapod.moving.xyz_in_COUCH,1,size(tt,2)));

target.psi_in_BODY = 0*(2*rand(3,1)-1)/180*pi - 0*[0;0;pi/18];
target.psi_in_COUCH = target.psi_in_BODY;
Q.target_in_COUCH = ZZ_Euler_2_DCM_P123(target.psi_in_COUCH);
Q.moving_in_COUCH = ZZ_Euler_2_DCM_P123(hexapod.moving.psi_in_COUCH_normal);
Q.target_in_MOVING = Q.target_in_COUCH*Q.moving_in_COUCH';
target.psi_in_MOVING_normal = Q2psi(Q.target_in_MOVING);
target.xyz_in_MOVING = target.xyz_in_MOVING_normal;
target.psi_in_MOVING = target.psi_in_MOVING_normal;


%% input head position data (with respect to moving panel).
fps = 8; nt1=2; nt2 = 2; nt3 = 3; nt4 = 1; nt5 = 6; % nt2: couch moving; nt3: no motion; nt4: target+hexapod moving; nt5: hexapod motion
fps = 4; nt1=0; nt2 = 1; nt3 = 0; nt4 = 14; nt5 = 1; 
fps = 5; nt1=0; nt2 = 0; nt3 = 0; nt4 = 10; nt5 = 0; 
% fps = 4; nt1=0; nt2 = 1; nt3 = 0; nt4 = 34; nt5 = 1; 
tt = zeros(3,(nt1+nt2+nt3+nt4+nt5)*fps); target.xyz_in_MOVING_all = tt; target.psi_in_MOVING_all = tt;
tt1 = 1*[5;5;5]/1000;  tt2 = 1.5*2.5*[-1;1;-1]/180*pi;
% tt1 = [et1;et2;et3]/1000;  tt2 = [et4;et5;et6]/180*pi;
for i=1:3
    target.xyz_in_MOVING_all(i,(nt1+nt2+nt3)*fps+1:(nt1+nt2+nt3+nt4)*fps) = linspace(0,tt1(i),nt4*fps);
    target.psi_in_MOVING_all(i,(nt1+nt2+nt3)*fps+1:(nt1+nt2+nt3+nt4)*fps) = linspace(0,tt2(i),nt4*fps);
end
target.xyz_in_MOVING_all(:,(nt1+nt2+nt3+nt4)*fps+1:end) = repmat(target.xyz_in_MOVING_all(:,(nt1+nt2+nt3+nt4)*fps),1,nt5*fps);
target.psi_in_MOVING_all(:,(nt1+nt2+nt3+nt4)*fps+1:end) = repmat(target.psi_in_MOVING_all(:,(nt1+nt2+nt3+nt4)*fps),1,nt5*fps);
target.xyz_in_MOVING_all = target.xyz_in_MOVING_all+repmat(target.xyz_in_MOVING,1,(nt1+nt2+nt3+nt4+nt5)*fps);
target.psi_in_MOVING_all = target.psi_in_MOVING_all+repmat(target.psi_in_MOVING,1,(nt1+nt2+nt3+nt4+nt5)*fps);


NNN = size(target.xyz_in_MOVING_all,2); %NNN=1;
hexapod.leg_length_all = zeros(6,NNN);
target.xyz_all = zeros(3,NNN); target.psi_all = zeros(3,NNN);
% hexapod.moving.xyz_in_COUCH_all = zeros(3,NNN); hexapod.moving.psi_in_COUCH_all = zeros(3,NNN);
% couch.xyz_all = zeros(3,NNN); couch.psi_all = zeros(3,NNN);
% hexapod.leg_force_all =zeros(6,NNN);

pn = 30; %[spx,spy,spz] = sphere(10);   
mmov(1:NNN)  =  struct('cdata', [], 'colormap', []);

%% LINAC
linac.width=1; linac.box1_y=1; linac.box2_z=1.25; linac.head_diameter = 0.77; 
linac.head_to_isocenter = 0.4; linac.head_high = 0.35; linac.box1_z = linac.head_to_isocenter + linac.head_high;
linac.position_box1 = [-linac.width/2 -linac.width/2 linac.width/2 linac.width/2
    linac.box1_y, linac.box1_y+0.36, linac.box1_y+0.36, linac.box1_y
    ones(1,4)*linac.box1_z];  % in the linac frame.
tt = linac.position_box1; tt(3,:) = -ones(1,4)*linac.box1_z;
linac.position_box1 = [linac.position_box1 tt]; %
linac.position_box2 = [-linac.width/2 -linac.width/2 linac.width/2 linac.width/2
    -linac.head_diameter/2-0.2, linac.position_box1(2,2), linac.position_box1(2,2), -linac.head_diameter/2-0.2
    ones(1,4)*linac.box2_z];  % in the linac frame.
tt = linac.position_box2; tt(3,:) = ones(1,4)*linac.box1_z;
linac.position_box2 = [linac.position_box2 tt]; %

t1 = linac.position_box1; t2 = linac.position_box2;
linac.position_box = [t2(:,[4 3]), t1(:,[7 8 4]), t2(:,8), t2(:,[1 2]), t1(:,[6 5 1]), t2(:,5)];

linac.gantry_angle_ini = -pi/6;
Q.linac=ZZ_Euler_2_DCM_P123([0 linac.gantry_angle_ini 0]);

%% couch
couch.top_length = 2.1;
couch.top_wH2 = [-couch.width/2 -couch.width/2 couch.width/2 couch.width/2
    -couch.top_length, 0, 0, -couch.top_length
    zeros(1,4)];  % in the couch frame.
tt = couch.top_wH2; tt(3,:) = -couch.top_thick;
couch.top_wH2 = [couch.top_wH2 tt]; % 6D couch position
% couch.top_wH2(2,:) = couch.top_wH2(2,:) +0.3;
couch.top_woH2 = couch.top_wH2; couch.top_woH2(2,:) = couch.top_woH2(2,:) +0.3; 

couch.box_to_isocenter = 0.7;
couch.box_length = 1.18;
couch.box = [-couch.width/2 -couch.width/2 couch.width/2 couch.width/2
    -couch.box_to_isocenter-couch.box_length, -couch.box_to_isocenter, -couch.box_to_isocenter, -couch.box_to_isocenter-couch.box_length
    zeros(1,4)];  % in the ground frame.
tt = couch.box; tt(3,:) = ground_z; couch.box = [couch.box tt]; % 6D couch box position
couch.box_connect = tt;  couch.box_connect(2,[1 4]) = 0.15;

%% connection between couch and base plate
tt1 = [-connect.couch_base(1)/2, -connect.couch_base(1)/2, connect.couch_base(1)/2, connect.couch_base(1)/2;
     -connect.couch_base(2)-hexapod.r_base, hexapod.r_base, hexapod.r_base, -connect.couch_base(2)-hexapod.r_base;
     ones(1,4)*(-hexapod.plate_thick-connect.couch_base(3))]; tt6 = tt1;
tt2 = tt1; tt2(3,:) = -hexapod.plate_thick; connect.couch_base_pos_A2 = [tt1, tt2]; % in hexapod base coordinate

connect.couch_base_pos_C2 = zeros(3,8);
switch hexapod.configuration
    case {1,4,7}
        tt7=-couch.top_thick-hexapod.base.xyz_in_COUCH(3)-connect.couch_base(3);
        connect.couch_base_pos_C2 = connect.couch_base_pos_A2; connect.couch_base_pos_C2(2,[2,3,6,7]) = -hexapod.base.xyz_in_COUCH(2);
        connect.couch_base_pos_C2(3,:) = connect.couch_base_pos_C2(3,:)-couch.top_thick-hexapod.base.xyz_in_COUCH(3)+hexapod.plate_thick;
        hexapod.base.spring = [hexapod.r_base*[-1 0 1]*0.75; (hexapod.r_base+1*hexapod.plate_thick)*ones(1,3); ones(1,3)*connect.couch_base_pos_C2(3,1)];
    case {2,5,8}
        tt7=-connect.couch_base(3)+0.05;
        hexapod.base.spring = [hexapod.r_base*[-1 0 1]*0.75; (-hexapod.r_base-1*hexapod.plate_thick)*ones(1,3); -ones(1,3)*hexapod.plate_thick];
        connect.couch_base_pos_A2(2,[1,4,5,8]) = hexapod.base.xyz_in_COUCH(2);
    case {3,6,9}
        tt7 = -hexapod.base.xyz_in_COUCH(2)-0.03;
        hexapod.base.spring = [hexapod.r_base*[-1 0 1]*0.75; connect.couch_base_pos_A2(2,1)*ones(1,3); ones(1,3)*(hexapod.normal_high + 2*hexapod.j_high + 5*hexapod.plate_thick)];
end
tt1 = tt6; tt1(2,[2,3]) = -connect.couch_base(2)-hexapod.r_base+connect.couch_base(3); tt1(3,:) = tt1(3,:)+connect.couch_base(3);
tt2 = tt1; tt2(3,:) = tt7; connect.couch_base_pos_B2 = [tt1, tt2]; % in hexapod base coordinate

% hexapod.base.spring(1,:) = hexapod.r_base*[-1 0 1]*2;

hexapod.mm_thr_ps = 0.005; hexapod.an_thr_ps = 0.3/180*pi;
hexapod.mm_thr_ps = 0.003; hexapod.an_thr_ps = 0.3/180*pi;
% hexapod.mm_thr_ps = 0.001; hexapod.an_thr_ps = 0.3/180*pi; %R01
% hexapod.mm_thr_ps = 2*hexapod.mm_thr_ps; hexapod.an_thr_ps = 2*hexapod.an_thr_ps;
% hexapod.mm_thr_ps = 100*hexapod.mm_thr_ps; hexapod.an_thr_ps = 100*hexapod.an_thr_ps;
hexapod.mm_thr = hexapod.mm_thr_ps/fps; hexapod.an_thr = hexapod.an_thr_ps/fps; 
T = (0:NNN-1)/fps;
control.fps = fps;
control.T = T;

target.xyz_accuracy_request=0.3; target.psi_accuracy_request=0.2;

save exam_con1 target hexapod couch control
