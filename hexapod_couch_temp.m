% 12:45 pm, 11/03   
close all; clear all; clc
% Notation:
% base platform: (w)           moving platform: (u)
% head: (h)                    radius: (r)                 joint: (j)
% desired: (de)                begin: (be)                 next:  (nx)
% translation: (xyz)           rotation -- Euler angle: (psi) in the 123 convention, i.e., psi = (pitch,roll,yaw)
CONVERSION = 'DCMtoEA123';
 
% LINAC frame; couch frame; hexapod moving panel frame; target frame.

Q2psi = @(x) (SpinCalc(CONVERSION,x,1e-12,1)'-round(SpinCalc(CONVERSION,x,1e-12,1)'/360)*360)/180*pi;
% Q2psi = @(x) ZZ_DCM_2_Euler123(x)-round(ZZ_DCM_2_Euler123(x)/2/pi)*2*pi;

%% setting
% hexapod or couch only
couchonly0_hexapod1 = 1; % hexapod
couchonly0_hexapod1 = 0; % couch
drawing_system1_No0 = 1;
% drawing_system1_No0 = 0;

disp_hex1_part0 = 1; % show hexapod
% disp_hex1_part0 = 0; % show part of hexapod only
disp_body1_No0 = 1; % show body
disp_body1_No0 = 0; 
disp_linac1_No0 = 1; % show LINAC
disp_linac1_No0 = 0;
disp_axes1_No0 = 1; % show axes
% disp_axes1_No0 = 0;
visible_switch = 'On'; % show animation
% visible_switch = 'Off';
hexapod.configuration =1;

xyzlim = [-0.4  0.4  -.62  0.3  -0.47  0.2];
% xyzlim = [-1.1  1.1  -2.3  1.5  -1.3  1.4];

xyz_thre=0.3; psi_thre=0.2; 

case_all={
    'operational-S plan',
    'operational-D plan',
    'target-S plan',
    'target-D plan',
    };

sel_meth = 4;
con_case = case_all{sel_meth};

tmp.workspace=linspace(-1,0,1);
tmp.workspace=linspace(-1,1,2);

line_width1 = 2;
line_width2 = 2;
line_width3 = 2;
line_width4 = 1;
line_width5 = 1;
xyz_linewidth = 1;
fig = figure(1);
set(gcf,'position',get(0,'screensize')); set(gcf,'renderer','zbuffer');
set(gcf,'Visible',visible_switch);
% set(fig,'Position',[0 0 2000 1500]);

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
pillow.thick = Nancy.skull.z_min-Nancy.torso_z_min+connect.couch_base(3)+0.012; % the last number is the overlap
head_support.couch_gap = 0.06;
pillow.couch_gap = 0.1;

hexapod.moving_bottom_to_couch_max=-1000*ones(1,3); hexapod.moving_bottom_to_couch_min=1000*ones(1,3);
hexapod.moving_bottom_to_isocenter_max=-1000*ones(1,3); hexapod.moving_bottom_to_isocenter_min=1000*ones(1,3);
hexapod.leg_length_max=-1000*ones(1,6); hexapod.leg_length_min=1000*ones(1,6); hexapod.leg_force_max=0;
hexapod.moving.xyz_in_COUCH_max = -1000*ones(1,3); hexapod.moving.xyz_in_COUCH_min =  1000*ones(1,3);
hexapod.moving.psi_in_COUCH_max = -1000*ones(1,3); hexapod.moving.psi_in_COUCH_min =  1000*ones(1,3);
hexapod.leg_force_max = -1000*ones(1,6); hexapod.leg_force_min = 1000*ones(1,6);

%% Bed and hexapod: set 1) the normal height of the hexapod; 2) the height of the couch. Base on 1) and 2), determine the thickness of the piollow.
hexapod.normal_high = 0.145; % from joint surface to joint surface
hexapod.normal_high = 0.197; % from joint surface to joint surface % for 50mm stroke. for leg length 175+25
% hexapod.normal_high = 0.217; %% [u_xyz,u_psi,err_t,ss]=ZZ_leg_length_to_6D_123(Lt_tar,u_xyz,u_psi,j_w,hexapod.j_u2,err_threshold)

Nancy.pivotB_y_in_COUCH_additional = 0.23; % for moving patient back setup.

tim=tic;

i2=0;
% for et1=tmp.workspace*5, for et2=tmp.workspace*5, for et3=tmp.workspace*5, for et4=tmp.workspace*2.5, for et5=tmp.workspace*2.5, for et6=tmp.workspace*2.5

couch.xyz_normal = [0;-0.25;-0.15]; couch.psi_normal = [0;0;0*pi/6];
if hexapod.configuration == 7, couch.xyz_normal(2) = couch.xyz_normal(2)- Nancy.pivotB_y_in_COUCH_additional; end
couch.psi_nx = couch.psi_normal; couch.xyz_nx = couch.xyz_normal;

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
hexapod.moving.xyz_in_COUCH_nx = hexapod.moving.xyz_in_COUCH_normal;
hexapod.moving.psi_in_COUCH_nx = hexapod.moving.psi_in_COUCH_normal;
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
if hexapod.configuration == 7, Nancy.pivotB_y_in_COUCH = Nancy.pivotB_y_in_COUCH + Nancy.pivotB_y_in_COUCH_additional; end% is the y coordinate of Nancy.pivotB in the couch frame.

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
t1 = ZZ_Euler_2_DCM_P123(hexapod.moving.psi_in_COUCH_nx);
tt = Nancy.skull.point; tt1 = size(tt,2); Nancy.skull.point_in_COUCH = tt+repmat(diff_body_couch,1,tt1); Nancy.skull.point_in_MOVING_be = t1*(Nancy.skull.point_in_COUCH-repmat(hexapod.moving.xyz_in_COUCH_nx,1,tt1));
tt = Nancy.neck_set_I; tt1 = size(tt,2); Nancy.neck_set_I_in_COUCH = tt+repmat(diff_body_couch,1,tt1); Nancy.neck_set_I_in_MOVING_be = t1*(Nancy.neck_set_I_in_COUCH-repmat(hexapod.moving.xyz_in_COUCH_nx,1,tt1));
tt = Nancy.pivotA; tt1 = size(tt,2); Nancy.pivotA_in_COUCH = tt+repmat(diff_body_couch,1,size(tt,2)); Nancy.pivotA_in_MOVING_be = t1*(Nancy.pivotA_in_COUCH-repmat(hexapod.moving.xyz_in_COUCH_nx,1,tt1));

%% define the position of tumor target.
target.xyz_in_BODY = Nancy.pivotA + [0; 80; 50]/1000 + 0*(2*rand(3,1)-1)/1000;
if couchonly0_hexapod1 == 0, target.xyz_in_BODY = Nancy.pivotA + [100; -550; 0]/1000 + 0*(2*rand(3,1)-1)/1000; end % lung tumor

tt = target.xyz_in_BODY; target.xyz_in_COUCH = tt+repmat(diff_body_couch,1,size(tt,2));
% target.xyz_in_COUCH=[0; hexapod.moving.xyz_in_COUCH_normal(2)+0.04; 0.1]; % move CM of the target on the top of hexapod center
target.xyz_in_MOVING_normal = t1*(target.xyz_in_COUCH-repmat(hexapod.moving.xyz_in_COUCH_nx,1,size(tt,2)));

target.psi_in_BODY = 0*(2*rand(3,1)-1)/180*pi - 0*[0;0;pi/18];
target.psi_in_COUCH = target.psi_in_BODY;
Q.target_in_COUCH = ZZ_Euler_2_DCM_P123(target.psi_in_COUCH);
Q.moving_in_COUCH = ZZ_Euler_2_DCM_P123(hexapod.moving.psi_in_COUCH_normal);
Q.target_in_MOVING = Q.target_in_COUCH*Q.moving_in_COUCH';
target.psi_in_MOVING_normal = Q2psi(Q.target_in_MOVING);
target.xyz_in_MOVING = target.xyz_in_MOVING_normal;
target.psi_in_MOVING = target.psi_in_MOVING_normal;

if disp_body1_No0 == 1
    t1 = ZZ_Euler_2_DCM_P123(target.psi_in_MOVING);
    tt = Nancy.skull.point_in_MOVING_be; tt1 = size(tt,2); Nancy.skull.point_in_TARGET = t1*(tt-repmat(target.xyz_in_MOVING,1,tt1));
    tt = Nancy.neck_set_I_in_MOVING_be; tt1 = size(tt,2); Nancy.neck_set_I_in_TARGET = t1*(tt-repmat(target.xyz_in_MOVING,1,tt1));
    tt = Nancy.pivotA_in_MOVING_be; tt1 = size(tt,2); Nancy.pivotA_in_TARGET = t1*(tt-repmat(target.xyz_in_MOVING,1,tt1));
end

%% input head position data (with respect to moving panel).
fps = 8; nt1=2; nt2 = 2; nt3 = 3; nt4 = 1; nt5 = 6; % nt2: couch moving; nt3: no motion; nt4: target+hexapod moving; nt5: hexapod motion
fps = 2; nt1=0; nt2 = 1; nt3 = 0; nt4 = 1; nt5 = 1; 
fps = 6; nt1=0; nt2 = 1; nt3 = 0; nt4 = 14; nt5 = 1; 
fps = 4; nt1=0; nt2 = 1; nt3 = 0; nt4 = 14; nt5 = 1; 
fps = 4; nt1=0; nt2 = 1; nt3 = 0; nt4 = 34; nt5 = 5; 
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


% % % %% input volunteer data
% % % nt1=0; nt2 = 1; nt3 = 0; nt4 = 14; nt5 = 1; 
% % % fnfn=17; eval(['load D6_uncorr', int2str(fnfn)]); fps=floor(size(hh_2,2)/max(TT)); nt4=floor(max(TT));
% % % tt = zeros(3,(nt1+nt2+nt3+nt4+nt5)*fps); target.xyz_in_MOVING_all = tt; target.psi_in_MOVING_all = tt;
% % % target.xyz_in_MOVING_all(1:3,(nt1+nt2+nt3)*fps+1:(nt1+nt2+nt3+nt4)*fps) = hh_2(1:3,1:nt4*fps)/1000;
% % % target.psi_in_MOVING_all(1:3,(nt1+nt2+nt3)*fps+1:(nt1+nt2+nt3+nt4)*fps) = hh_2(4:6,1:nt4*fps)/180*pi;
% % % target.xyz_in_MOVING_all(:,(nt1+nt2+nt3+nt4)*fps+1:end) = repmat(target.xyz_in_MOVING_all(:,(nt1+nt2+nt3+nt4)*fps),1,nt5*fps);
% % % target.psi_in_MOVING_all(:,(nt1+nt2+nt3+nt4)*fps+1:end) = repmat(target.psi_in_MOVING_all(:,(nt1+nt2+nt3+nt4)*fps),1,nt5*fps);
% % % target.xyz_in_MOVING_all = target.xyz_in_MOVING_all+repmat(target.xyz_in_MOVING,1,(nt1+nt2+nt3+nt4+nt5)*fps);
% % % target.psi_in_MOVING_all = target.psi_in_MOVING_all+repmat(target.psi_in_MOVING,1,(nt1+nt2+nt3+nt4+nt5)*fps);

% fnfn=17;
% eval(['load D6_uncorr', int2str(fnfn)]); %T=TT;
% fps=round(size(hh_2,2)/max(TT)); 
% nt1=0; nt2 = 1; nt3 = 0; nt4=10; nt5=0;
% target.xyz_in_MOVING_all=[]; target.psi_in_MOVING_all=[];
% target.xyz_in_MOVING_all(1:3,(nt1+nt2+nt3)*fps+1:(nt1+nt2+nt3+nt4)*fps) = hh_2(1:3,1:nt4*fps)/1000;
% target.psi_in_MOVING_all(1:3,(nt1+nt2+nt3)*fps+1:(nt1+nt2+nt3+nt4)*fps) = hh_2(4:6,1:nt4*fps)/180*pi;
% target.xyz_in_MOVING_all = target.xyz_in_MOVING_all+repmat(target.xyz_in_MOVING,1,(nt1+nt2+nt3+nt4+nt5)*fps);
% target.psi_in_MOVING_all = target.psi_in_MOVING_all+repmat(target.psi_in_MOVING,1,(nt1+nt2+nt3+nt4+nt5)*fps);

NNN = size(target.xyz_in_MOVING_all,2); %NNN=1;
hexapod.leg_length_all = zeros(6,NNN);
target.xyz_all = zeros(3,NNN); target.psi_all = zeros(3,NNN);
hexapod.moving.xyz_in_COUCH_all = zeros(3,NNN); hexapod.moving.psi_in_COUCH_all = zeros(3,NNN);
couch.xyz_all = zeros(3,NNN); couch.psi_all = zeros(3,NNN);
hexapod.leg_force_all =zeros(6,NNN);

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
    
%% support+pillow
switch hexapod.configuration
    case {1,4,7}
        if hexapod.configuration == 7, 
            head_support.length = head_support.length + Nancy.pivotB_y_in_COUCH_additional;
            pillow.couch_gap = pillow.couch_gap + Nancy.pivotB_y_in_COUCH_additional;
        end 
        t1 = hexapod.moving.xyz_in_COUCH_normal(3);
        t3 = -hexapod.moving.xyz_in_COUCH_normal(2)+head_support.couch_gap; t4 = t3+head_support.length; % y: length
        tt1 = [-head_support.width/2, -head_support.width/2, head_support.width/2, head_support.width/2
            -t4,-t3,-t3,-t4
            t1,t1,t1,t1];
        tt2 = tt1; tt2(3,:) = tt2(3,:)+couch.top_thick; tt2(2,[2,3])=tt2(2,[2,3])-couch.top_thick; head_support.pos2 = [tt1, tt2]; % in the moving plate frame
 
        t1 = hexapod.moving.xyz_in_COUCH_normal(3);
        t3 = -hexapod.moving.xyz_in_COUCH_normal(2)+pillow.couch_gap; t4 = t3+pillow.length; % y: length
        tt1 = [-pillow.width/2, -pillow.width/2, pillow.width/2, pillow.width/2
            -t4,-t3,-t3,-t4
            t1,t1,t1,t1];
        tt2 = tt1; tt2(3,:) = tt2(3,:)-pillow.thick; pillow.pos2 = [tt1, tt2];
 
        t1 = -hexapod.moving.xyz_in_COUCH_normal(2)+0.025; t2 = t1+hexapod.r_moving*0.1;
        tt1 = tt6; tt1(2,:) = -[t1, 0, 0, t1];
        tt2 = tt1; tt2(3,:) = tt2(3,:)+connect.couch_base(3); tt2(2,:) = -[t2, 0, 0, t2];
        connect.moving_support_A2 = [tt1, tt2];
        hexapod.moving.spring = [hexapod.r_base*[-1 0 1]*0.75; ones(1,3)*(-hexapod.r_base-1*hexapod.plate_thick); ones(1,3)*connect.moving_support_A2(3,1)]; % in the moving frame
    case {2,5,8}
        t1 = hexapod.normal_high+hexapod.plate_thick+2*hexapod.j_high+connect.couch_base(3); % z: high
        t3 = -hexapod.moving.xyz_in_COUCH_normal(2)+head_support.couch_gap; t4 = t3+head_support.length; % y: length
        tt1 = [-head_support.width/2, -head_support.width/2, head_support.width/2,head_support.width/2
            t3,t4,t4,t3
            t1,t1,t1,t1];
        tt2 = tt1; tt2(3,:) = tt2(3,:)+couch.top_thick; tt1(2,[1 4])=tt1(2,[1 4])+pillow.thick/2; 
        head_support.pos2 = [tt1, tt2]; % in end effector frame
 
        t1 = hexapod.normal_high+hexapod.plate_thick+2*hexapod.j_high+couch.top_thick; % z: high
        t3 = -hexapod.moving.xyz_in_COUCH_normal(2)+pillow.couch_gap; t4 = t3+pillow.length; % y: length
        tt1 = [-pillow.width/2, -pillow.width/2, pillow.width/2, pillow.width/2
            t3,t4,t4,t3
            t1,t1,t1,t1];
        tt2 = tt1; tt2(3,:) = tt2(3,:)+pillow.thick; pillow.pos2 = [tt1, tt2];
 
        t1 = hexapod.r_moving*1; t2 = t1+hexapod.r_moving*0.1; 
        tt1 = tt6; tt1(2,:) = [-t1*sin(pi/3), t1, t1, -t1*sin(pi/3)];
        tt2 = tt1; tt2(3,:) = tt2(3,:)+connect.couch_base(3); tt1(2,:) = [-t1*sin(pi/3), t2, t2, -t1*sin(pi/3)];
        connect.moving_support_A2 = [tt1, tt2];
        hexapod.moving.spring = [hexapod.r_base*[-1 0 1]*0.75; ones(1,3)*connect.moving_support_A2(2,2); ones(1,3)*(-hexapod.plate_thick)]; % in the moving frame
    case {3,6,9}
        t1 = -hexapod.moving.xyz_in_COUCH_normal(3); % high
        t3 = hexapod.moving.xyz_in_COUCH_normal(2)-head_support.couch_gap; t4 = t3-head_support.length; % y: length
        tt1 = [-head_support.width/2, -head_support.width/2, head_support.width/2, head_support.width/2
            t1,t1,t1,t1
            t4,t3,t3,t4];
        tt2 = tt1; tt2(2,:) = tt2(2,:)-couch.top_thick; tt2(3,[2 3])=tt2(3,[2 3])-pillow.thick/2; head_support.pos2 = [tt1, tt2]; % in end effector frame
 
        t1 = -hexapod.moving.xyz_in_COUCH_normal(3); % z: high
        t3 = hexapod.moving.xyz_in_COUCH_normal(2)-pillow.couch_gap; t4 = t3-pillow.length; % y: length
        tt1 = [-pillow.width/2, -pillow.width/2, pillow.width/2, pillow.width/2
             t1,t1,t1,t1
             t4,t3,t3,t4];
        tt2 = tt1; tt2(2,:) = tt2(2,:)+pillow.thick; pillow.pos2 = [tt1, tt2];
 
        t1 = -hexapod.moving.xyz_in_COUCH_normal(2); t2 = t1+connect.couch_base(3);
        tt1 = tt6; tt1(2,:) = [-hexapod.r_base/3, 0, 0, -hexapod.r_base/3]+hexapod.r_base*0.0; tt1(3,:) = tt1(3,:)-t1;
        tt2 = tt1; tt2(3,:) = tt2(3,:)+t2; 
        connect.moving_support_A2 = [tt1, tt2];
        hexapod.moving.spring = [hexapod.r_base*[-1 0 1]*0.75; ones(1,3)*connect.moving_support_A2(2,1); ones(1,3)*(-5*hexapod.plate_thick)]; % in the moving frame
%         hexapod.moving.spring = zeros(3,3);
end

% t1 = hexapod.r_base/5; t2 = hexapod.r_base/3;
axis_direction = inline('[  0 t1 0  0  t1+t2*1   0   t2;  0 0  t1 0   0   t1+t2  t2; 0 0  0  t1  t2  t2  t1+t2]','t1','t2');

hexapod.mm_thr_ps = 0.005; hexapod.an_thr_ps = 0.3/180*pi;
% hexapod.mm_thr_ps = 0.001; hexapod.an_thr_ps = 0.3/180*pi; %R01
% hexapod.mm_thr_ps = 2*hexapod.mm_thr_ps; hexapod.an_thr_ps = 2*hexapod.an_thr_ps;
% hexapod.mm_thr_ps = 100*hexapod.mm_thr_ps; hexapod.an_thr_ps = 100*hexapod.an_thr_ps;
hexapod.mm_thr = hexapod.mm_thr_ps/fps; hexapod.an_thr = hexapod.an_thr_ps/fps; 
T = (1:NNN)/fps;


con_on = 0;
vmax=[hexapod.mm_thr*ones(3,1);hexapod.an_thr*ones(3,1)];
for iii = 1:NNN     
    % input target.xyz_in_MOVING/psi. In clinic, target.xyz/psi is measured, and is used for computing target.in_MOVING.
    target.xyz_in_MOVING = target.xyz_in_MOVING_all(:,iii); target.psi_in_MOVING = target.psi_in_MOVING_all(:,iii); 

    % save hexapod.moving.xyz_in_COUCH/psi and couch.xyz/psi for plot.
    hexapod.moving.xyz_in_COUCH_all(:,iii) = hexapod.moving.xyz_in_COUCH_nx; 
    hexapod.moving.psi_in_COUCH_all(:,iii) = hexapod.moving.psi_in_COUCH_nx; 
    couch.xyz_all(:,iii) = couch.xyz_nx; couch.psi_all(:,iii) = couch.psi_nx;    
 
    % define hexapod.moving.xyz_in_COUCH/psi_be and couch.xyz/psi for next step compuation.
    hexapod.moving.xyz_in_COUCH_be = hexapod.moving.xyz_in_COUCH_nx; hexapod.moving.psi_in_COUCH_be = hexapod.moving.psi_in_COUCH_nx;
    couch.xyz_be = couch.xyz_nx; couch.psi_be = couch.psi_nx;
 
    % compute target.xyz/psi for plot (The calculation will not be neccessary in clinic.)
    Q.target_in_MOVING=ZZ_Euler_2_DCM_P123(target.psi_in_MOVING); Q.moving_in_COUCH = ZZ_Euler_2_DCM_P123(hexapod.moving.psi_in_COUCH_be); 
    Q.couch = ZZ_Euler_2_DCM_P123(couch.psi_be);
    Q.target = Q.target_in_MOVING*Q.moving_in_COUCH*Q.couch;
    target.psi_be = Q2psi(Q.target);
    target.xyz_in_COUCH_be = hexapod.moving.xyz_in_COUCH_be+Q.moving_in_COUCH'*target.xyz_in_MOVING;
    target.xyz_be = couch.xyz_nx+Q.couch'*target.xyz_in_COUCH_be;
    target.xyz_all(:,iii) = target.xyz_be; target.psi_all(:,iii) = target.psi_be;    

    err_xyz = norm(target.xyz_be)*1000; err_psi = norm(target.psi_be)*180/pi;

    % compute hexapod.moving.xyz/psi
    Q.moving = Q.moving_in_COUCH*Q.couch;
    hexapod.moving.psi = Q2psi(Q.moving);
    hexapod.moving.xyz = couch.xyz_be+Q.couch'*hexapod.moving.xyz_in_COUCH_be;
    hexapod.joints_of_moving = repmat(hexapod.moving.xyz,1,6)+Q.moving'*hexapod.j_u2;

    if couchonly0_hexapod1 == 1, couch.top = couch.top_wH2; cxyz = couch.xyz_be; cpsi = couch.psi_be; end; %with hexapod
    if couchonly0_hexapod1 == 0, couch.top = couch.top_woH2; cxyz = couch.xyz_be; cpsi = couch.psi_be; end % without hexapod
 
    %% drawing    
    if drawing_system1_No0
        show_system
    end
 
    %% force computation
    % the origin of the moving plate is the pivot for the torque calculation.
    tmp.head_to_moving_center = target.xyz_be-hexapod.moving.xyz; Nancy.head_weight = [0;0;-5];
    tmp.force_on_moving = Nancy.head_weight;
    tmp.torque_on_moving = cross(tmp.head_to_moving_center,Nancy.head_weight);
   
    tt7 = [hexapod.j_u2, hexapod.moving.spring];
    tttt = repmat(hexapod.moving.xyz,1,9)+ZZ_Euler_2_DCM_P123(hexapod.moving.psi)'*tt7;
    tmp.j_ut = tttt(:,1:6); tmp.moving_spring = tttt(:,7:9);

    tt7 = [hexapod.j_w2, hexapod.base.spring];
    tt8 = repmat(hexapod.base.xyz_in_COUCH,1,9)+ZZ_Euler_2_DCM_P123(hexapod.base.psi_in_COUCH)'*tt7;
    ttt = repmat(cxyz,1,9)+ZZ_Euler_2_DCM_P123(cpsi)'*tt8;        
    tmp.j_wt = ttt(:,1:6); tmp.base_spring = ttt(:,7:9);

    tt = tmp.j_wt-tmp.j_ut;
    hexapod.leg_length_all(:,iii) = sqrt(tt(1,:).*tt(1,:)+tt(2,:).*tt(2,:)+tt(3,:).*tt(3,:));
    
    if any(hexapod.configuration == [4,5,6])
        for i3 = [1 3]
            tmp.moving_spring_to_moving_center = tmp.moving_spring(:,i3)-hexapod.moving.xyz; 
            t1 = tmp.moving_spring(:,i3)-tmp.base_spring(:,i3); 
            if hexapod.configuration == 4
                tmp.spring_force =[0;0;5]/2*1.9;
            elseif hexapod.configuration == 5
                tmp.spring_force = -t1/norm(t1)*5/2*1.9;
            else
                tmp.spring_force = -t1/norm(t1)*5/2*0.8; %tmp.spring_force =[0;0;5]/2*0.8;
            end
            tmp.spring_torque_on_moving = cross(tmp.moving_spring_to_moving_center,tmp.spring_force);
            tmp.force_on_moving = tmp.force_on_moving + tmp.spring_force;
            tmp.torque_on_moving = tmp.torque_on_moving + tmp.spring_torque_on_moving;
        end
    end    
    hexapod.force_on_leg = ZZ_static_force(hexapod.moving.xyz,hexapod.moving.psi,tmp.j_wt,hexapod.j_u2,tmp.force_on_moving,tmp.torque_on_moving)';
    max_leg_force = max(abs(hexapod.force_on_leg));
    hexapod.leg_force_all(:,iii)=hexapod.force_on_leg';
 
    %% control algorithm
    if (iii<(nt1+nt2)*fps)&&(iii>nt1*fps)
        % couch control
        Q.couch_de =  Q.moving_in_COUCH' * Q.target_in_MOVING'; 
        couch.xyz_de = -Q.couch_de'*target.xyz_in_COUCH_be;
        couch.psi_de = Q2psi(Q.couch_de);
 
        tt1 = couch.xyz_de-couch.xyz_be;
        tt2 = couch.psi_de-couch.psi_be;
        couch.xyz_nx = couch.xyz_be+tt1/((nt1+nt2)*fps-iii); %+0.5*(2*rand(3,1)-1)/1000;
        couch.psi_nx = couch.psi_be+tt2/((nt1+nt2)*fps-iii); %+0.2*(2*rand(3,1)-1)/180*pi;
        Q.linac=ZZ_Euler_2_DCM_P123([0 linac.gantry_angle_ini-pi/72*iii*0 0]);
        
    elseif (iii>(nt1+nt2+nt3)*fps)
        % hexapod control
        % compute hexapod.moving.xyz_in_COUCH/psi_de
        Q.moving_in_COUCH_de = Q.target_in_MOVING'*Q.couch';
        hexapod.moving.xyz_in_COUCH_de = -Q.couch*couch.xyz_be - Q.moving_in_COUCH_de'*target.xyz_in_MOVING;
        hexapod.moving.psi_in_COUCH_de = Q2psi(Q.moving_in_COUCH_de);
        t3=[hexapod.moving.xyz_in_COUCH_de-hexapod.moving.xyz_in_COUCH_be; hexapod.moving.psi_in_COUCH_de-hexapod.moving.psi_in_COUCH_be]./vmax; 
        kappa=max([abs(t3);1]);
        
        if con_on
            if kappa>1
                switch con_case
                    case {'operational-D plan'}
                        t1 = hexapod.moving.xyz_in_COUCH_de-hexapod.moving.xyz_in_COUCH_be;
                        t2 = min(max(t1,-hexapod.mm_thr),hexapod.mm_thr);
                        t3 = hexapod.moving.psi_in_COUCH_de-hexapod.moving.psi_in_COUCH_be;
                        t4 = min(max(t3,-hexapod.an_thr),hexapod.an_thr);
                        hexapod.moving.xyz_in_COUCH_nx = hexapod.moving.xyz_in_COUCH_be + t2;
                        hexapod.moving.psi_in_COUCH_nx = hexapod.moving.psi_in_COUCH_be + t4;
                        
                    case {'operational-S plan'}
                        t1 = hexapod.moving.xyz_in_COUCH_de-hexapod.moving.xyz_in_COUCH_be;
                        t2 = hexapod.moving.psi_in_COUCH_de-hexapod.moving.psi_in_COUCH_be;
                        hexapod.moving.xyz_in_COUCH_nx = hexapod.moving.xyz_in_COUCH_be + t1/kappa;
                        hexapod.moving.psi_in_COUCH_nx = hexapod.moving.psi_in_COUCH_be + t2/kappa;                        

                    case {'target-S plan'}
                        f_rg=@(x) ZZ_Euler_2_DCM_P123(x)'*target.xyz_in_MOVING;
                        [A, rg0]=appro_r_psi(f_rg,hexapod.moving.psi_in_COUCH_be,hexapod.an_thr*ones(3,1));
                        f_gpsi=@(x) Q2psi(Q.target_in_MOVING*ZZ_Euler_2_DCM_P123(x));
                        [B, gpsi0]=appro_r_psi(f_gpsi,hexapod.moving.psi_in_COUCH_be,hexapod.an_thr*ones(3,1));

                        AA=[eye(3), A; zeros(3,3), B];
                        t1=AA\(-[target.xyz_be; target.psi_be]);
                        t2=max(abs(t1)./vmax);
                        mu=min(1/t2,1);
                        t3=t1*mu;
                        hexapod.moving.xyz_in_COUCH_nx = hexapod.moving.xyz_in_COUCH_be + t3(1:3);
                        hexapod.moving.psi_in_COUCH_nx = hexapod.moving.psi_in_COUCH_be + t3(4:6);

                    case {'target-D plan','target-DT plan','target-DA plan','target-DS plan'}
                        f_rg=@(x) ZZ_Euler_2_DCM_P123(x)'*target.xyz_in_MOVING;
                        [A, rg0]=appro_r_psi(f_rg,hexapod.moving.psi_in_COUCH_be,hexapod.an_thr*ones(3,1));
                        f_gpsi=@(x) Q2psi(Q.target_in_MOVING*ZZ_Euler_2_DCM_P123(x));
                        [B, gpsi0]=appro_r_psi(f_gpsi,hexapod.moving.psi_in_COUCH_be,hexapod.an_thr*ones(3,1));

                    weighh=9;
                    do=[(-hexapod.moving.xyz_in_COUCH_be-rg0)*1000; (-gpsi0)*180/pi*weighh];
                    Ao=[1000*eye(3), A*1000; zeros(3,3), B*180/pi*weighh];
                    do=double(do); Ao=double(Ao);
                    AtA = Ao'*Ao; Ab = Ao'*do;

                    blx=-vmax; bux=vmax;
                    blx=double(blx);bux=double(bux);
                    %fcn     = @(x) norm( Ao*x - do)^2;
                    %grad2    = @(x) 2*( AtA*x - Ab );
                    %fun     = @(x)fminunc_wrapper( x, fcn, grad2); 
                    %opts    = struct( 'factr', 1e5, 'pgtol', 1e-5, 'm', 10, 'maxIts',50,'printEvery',500);
                    %[xk, ~, info] = lbfgsb(fun, blx, bux, opts ); %xk'*1000

                    options = optimoptions('quadprog','Algorithm','interior-point-convex','Display','off');
                    %options = optimoptions('quadprog','Algorithm','interior-point-convex');
                    [xk,~,~,outt] = quadprog(AtA,-Ab,[],[],[],[],blx,bux,zeros(6,1),options);
%                     itttt=outt.iterations

                        hexapod.moving.xyz_in_COUCH_nx = hexapod.moving.xyz_in_COUCH_be + xk(1:3);
                        hexapod.moving.psi_in_COUCH_nx = hexapod.moving.psi_in_COUCH_be + xk(4:6);

                end
            else % kappa==1
                hexapod.moving.xyz_in_COUCH_nx = hexapod.moving.xyz_in_COUCH_de;
                hexapod.moving.psi_in_COUCH_nx = hexapod.moving.psi_in_COUCH_de;
            end
        else %con_on==0
            hexapod.moving.xyz_in_COUCH_nx = hexapod.moving.xyz_in_COUCH_be;
            hexapod.moving.psi_in_COUCH_nx = hexapod.moving.psi_in_COUCH_be;
        end
    end
     
%     if (err_xyz<0.15)&(err_psi<0.1) con_on = 0; end;
%     if (err_xyz>0.3)+(err_psi>0.2) con_on = 1; end;
    if (err_xyz<xyz_thre*0.4)&(err_psi<psi_thre*0.4) con_on = 0; end;
    if (err_xyz>xyz_thre*0.8)+(err_psi>psi_thre*0.8) con_on = 1; end;
    con_on=1;
 
    %% plot
    set(gcf, 'Color', 'w'); axis off; axis equal; axis(xyzlim); set(gca,'XTick',[],'YTick',[],'ZTick',[]);
%     view([115,20]); % 
    view([115,12]); % 
%     view([112,10]); % 
%     view([90,0]); % 
    drawnow; hold off; colormap copper; brighten(1); lighting phong; material shiny
 
%     if iii==1; pause; end
         
    mmov(iii) = getframe(gca);
% %     ffrr = getframe(gca); im  =  frame2im(ffrr);
% %     [imind,cm]  =  rgb2ind(im,256);
% %     outfile  =  'temp.gif';
% %     if iii == 1
% %         imwrite(imind,cm,outfile,'gif','DelayTime',0,'loopcount',inf);
% %     else
% %         imwrite(imind,cm,outfile,'gif','DelayTime',0,'writemode','append');
% %     end
     
end
 

% orient(fig,'landscape')
% print(fig,'LandscapePage.pdf','-dpdf')
% 
% fig.PaperPositionMode = 'manual';
% orient(fig,'landscape')
% print(fig,'LandscapePage_ExpandedFigure.pdf','-dpdf')
% 
% set(gcf,'Paperunits','inches');
% orient(fig,'landscape');
% set(gcf,'PaperPosition', [0 0 7 5])
% print -depsc2 -painters Plot6.eps


hexapod.leg_length_max=max([hexapod.leg_length_max; max(hexapod.leg_length_all')]);
hexapod.leg_length_min=min([hexapod.leg_length_min; min(hexapod.leg_length_all')]);
hexapod.stroke_max6 = hexapod.leg_length_max-hexapod.leg_length_min;
hexapod.leg_force_max= max([hexapod.leg_force_max; max(hexapod.leg_force_all')]);
hexapod.leg_force_min= min([hexapod.leg_force_min; min(hexapod.leg_force_all')]);

hexapod.moving.xyz_in_COUCH_max=max([hexapod.moving.xyz_in_COUCH_max; max(hexapod.moving.xyz_in_COUCH_all')]);
hexapod.moving.xyz_in_COUCH_min=min([hexapod.moving.xyz_in_COUCH_min; min(hexapod.moving.xyz_in_COUCH_all')]);
hexapod.moving.psi_in_COUCH_max=max([hexapod.moving.psi_in_COUCH_max; max(hexapod.moving.psi_in_COUCH_all')]);
hexapod.moving.psi_in_COUCH_min=min([hexapod.moving.psi_in_COUCH_min; min(hexapod.moving.psi_in_COUCH_all')]);



% i2=i2+1
% [et1,et2,et3,et4,et5,et6]
tt='hexapod.leg_length_max'; disp(tt); eval(['disp(' tt ')']);
tt='hexapod.leg_length_min'; disp(tt); eval(['disp(' tt ')']);
tt='hexapod.stroke_max6'; disp(tt); eval(['disp(' tt ')']);
tt='hexapod.moving_bottom_to_couch_max'; disp(tt); eval(['disp(' tt ')']);
tt='hexapod.moving_bottom_to_couch_min'; disp(tt); eval(['disp(' tt ')']);
tt='hexapod.moving_bottom_to_isocenter_max'; disp(tt); eval(['disp(' tt ')']);
tt='hexapod.moving_bottom_to_isocenter_min'; disp(tt); eval(['disp(' tt ')']);
tt='hexapod.leg_force_max'; disp(tt); eval(['disp(' tt ')']);
tt='hexapod.leg_force_min'; disp(tt); eval(['disp(' tt ')']);

tt='hexapod.moving.xyz_in_COUCH_max'; disp([tt,'_mm']); eval(['disp(' tt '*1000)']);
tt='hexapod.moving.xyz_in_COUCH_min'; disp([tt,'_mm']); eval(['disp(' tt '*1000)']);
tt='hexapod.moving.psi_in_COUCH_max'; disp([tt,'_deg']); eval(['disp(' tt '*180/pi)']);
tt='hexapod.moving.psi_in_COUCH_min'; disp([tt,'_deg']); eval(['disp(' tt '*180/pi)']);



% pause
% end, end, end, end, end, end

toc(tim)
 
% 
% fpps = 12;
% mov  =  VideoWriter('hexapod.avi'); mov.FrameRate = fpps; open(mov);
% for k = 1:NNN
%     writeVideo(mov,mmov(k));
% end
% close(mov);
 
figure(2); set(gcf,'units','normalized','outerposition',[0 0 1 1])
subplot(2,5,6); plot(T,target.psi_all*180/pi); ylabel('target \alpha\beta\gamma (deg)') 
xlabel('Time (sec)'); grid on; legend('pitch','roll','yaw');xlim([nt1+nt2+nt3,max(T)]);
tt = target.xyz_all;
subplot(2,5,1); plot(T,tt*1000); ylabel(' target xyz (mm)');xlim([nt1+nt2+nt3,max(T)]);
xlabel('Time (sec)'); grid on; legend('X','Y','Z')
title('target position')

        tt=target.xyz_all(:,(nt1+nt2+nt3)*fps+1:end); tt1=sqrt(sum(tt.*tt))*1000;
        tt=target.psi_all(:,(nt1+nt2+nt3)*fps+1:end); tt2=sqrt(sum(tt.*tt))*180/pi;
        N_out_mm=sum((tt1>xyz_thre))
        N_out_an=sum((tt2>psi_thre))
        N_out_thre=sum((tt1>xyz_thre)|(tt2>psi_thre))

 
tt=couch.psi_all-repmat(couch.psi_normal,1,NNN);
subplot(2,5,7); plot(T,tt*180/pi); ylabel('couch \alpha\beta\gamma(deg)');
xlabel('Time (sec)'); grid on; legend('pitch','roll','yaw');xlim([0,max(T)]);
tt = couch.xyz_all-repmat(couch.xyz_normal,1,NNN);
subplot(2,5,2); plot(T,tt*1000); ylabel('couch xyz (mm)');xlim([0,max(T)]);
xlabel('Time (sec)'); grid on; legend('X','Y','Z')
title('couch motion')
 
tt = target.psi_in_MOVING_all - repmat(target.psi_in_MOVING_normal,1,NNN);
subplot(2,5,8); plot(T,tt'*180/pi); ylabel('head motion \alpha\beta\gamma (deg)');
xlabel('Time (sec)'); grid on; legend('pitch','roll','yaw');xlim([0,max(T)]);
tt = target.xyz_in_MOVING_all - repmat(target.xyz_in_MOVING_normal,1,NNN);
subplot(2,5,3); plot(T,tt'*1000); ylabel('head motion xyz (mm)');
xlabel('Time (sec)'); grid on; legend('X', 'Y','Z');xlim([0,max(T)]);
title('head motion with respect to the moving plate')
 
tt = hexapod.moving.psi_in_COUCH_all - repmat(hexapod.moving.psi_in_COUCH_normal,1,NNN);
subplot(2,5,9); plot(T,tt'*180/pi); ylabel('hexapod motion \alpha\beta\gamma (deg)');
xlabel('Time (sec)'); grid on; legend('pitch','roll','yaw'); xlim([0,max(T)]);
tt = hexapod.moving.xyz_in_COUCH_all - repmat(hexapod.moving.xyz_in_COUCH_normal,1,NNN);
subplot(2,5,4); plot(T,tt'*1000); ylabel('hexapod motion xyz (mm)');
xlabel('Time (sec)'); grid on; legend('X', 'Y','Z'); xlim([0,max(T)]);
title('hexapod motion')
  
tt = hexapod.leg_length_all; %-repmat(hexapod.leg_length_all(:,1),1,NNN);
subplot(2,5,5); plot(T,tt'*1000); 
xlabel('Time (sec)'); ylabel('Control input -- Lengths of legs (mm)');xlim([0,max(T)]);
grid on; legend('1','2','3','4','5','6'); xlim([0,max(T)]);
title('leg length')
 
tt = hexapod.leg_force_all; %
subplot(2,5,10); plot(T,tt'); 
xlabel('Time (sec)'); ylabel('Legs force input (kg)'); xlim([0,max(T)]);
grid on; legend('1','2','3','4','5','6');
title('Leg forces')
% max_force_i=max(abs(tt(:)));
