function [hexapod, con_on]= hex_controller(hexapod, target, control) 

% computing next hexapod.moving.xyz/psi is the objective of this sub-fun.

vmax=[hexapod.mm_thr*ones(3,1);hexapod.an_thr*ones(3,1)];

% In real control, there is always small position error. The following is
% to avoid robot correction when there is only a small error.
t1 = norm(target.xyz)*1000; t2 = norm(target.psi)*180/pi;
if (t1<target.xyz_accuracy_request*0.4)&&(t2<target.psi_accuracy_request*0.4), con_on = 0; end;
if (t1>target.xyz_accuracy_request*0.8)||(t2>target.psi_accuracy_request*0.8), con_on = 1; end;

con_on = 1; % robot keep moving even for a small position error.

% compute next hexapod.moving.xyz/psi
if con_on
    
    % compute target.xyz/psi in the end-effector(i.e., MOVING)
    Q.moving = ZZ_Euler_2_DCM_P123(hexapod.moving.psi);
    Q.target = ZZ_Euler_2_DCM_P123(target.psi);
    Q.target_in_MOVING = Q.target * Q.moving';
    target.xyz_in_MOVING = Q.moving*(target.xyz - hexapod.moving.xyz);    
    hexapod.joints_of_moving = repmat(hexapod.moving.xyz,1,6)+Q.moving'*hexapod.j_u2;

    % compute hexapod.moving.xyz/psi_de so that the target position is zeros(6,1).
    Q.moving_de = Q.target_in_MOVING';
    hexapod.moving.xyz_de = -Q.moving'*target.xyz_in_MOVING;
    hexapod.moving.psi_de = Q2psi(Q.moving_de);
    t2 = hexapod.moving.psi_de-hexapod.moving.psi; t2 = t2-round(t2/2/pi)*2*pi;
    t3 = [hexapod.moving.xyz_de-hexapod.moving.xyz; t2]./vmax;
    kappa = max(abs(t3));
    
    if kappa>1 % need more than one move to compensate.
        switch control.algorithm_used

            case {'operational-S plan'}
                t1 = hexapod.moving.xyz_de-hexapod.moving.xyz; 
                t2 = hexapod.moving.psi_de-hexapod.moving.psi; t2 = t2-round(t2/2/pi)*2*pi;
                hexapod.moving.xyz = hexapod.moving.xyz + t1/kappa;
                hexapod.moving.psi = hexapod.moving.psi + t2/kappa;

            case {'operational-D plan'}
                t1 = hexapod.moving.xyz_de-hexapod.moving.xyz;
                t2 = min(max(t1,-hexapod.mm_thr),hexapod.mm_thr);
                t3 = hexapod.moving.psi_de-hexapod.moving.psi; t3 = t3-round(t3/2/pi)*2*pi;
                t4 = min(max(t3,-hexapod.an_thr),hexapod.an_thr);
                hexapod.moving.xyz = hexapod.moving.xyz + t2;
                hexapod.moving.psi = hexapod.moving.psi + t4;

            case {'target-S plan'}
                f_rg=@(x) ZZ_Euler_2_DCM_P123(x)'*target.xyz_in_MOVING;
                [A, rg0]=appro_r_psi(f_rg,hexapod.moving.psi,hexapod.an_thr*ones(3,1));
                f_gpsi=@(x) Q2psi(Q.target_in_MOVING*ZZ_Euler_2_DCM_P123(x));
                [B, gpsi0]=appro_r_psi2(f_gpsi,hexapod.moving.psi,hexapod.an_thr*ones(3,1));

                AA=[eye(3), A; zeros(3,3), B];
                t1=AA\(-[target.xyz; target.psi]);
                t2=max(abs(t1)./vmax);
                mu=min(1/t2,1);
                t3=t1*mu;
                hexapod.moving.xyz = hexapod.moving.xyz + t3(1:3);
                hexapod.moving.psi = hexapod.moving.psi + t3(4:6);

            case {'target-D plan'}
                f_rg=@(x) ZZ_Euler_2_DCM_P123(x)'*target.xyz_in_MOVING;
                [A, rg0]=appro_r_psi(f_rg,hexapod.moving.psi,hexapod.an_thr*ones(3,1));
                f_gpsi=@(x) Q2psi(Q.target_in_MOVING*ZZ_Euler_2_DCM_P123(x));
                [B, gpsi0]=appro_r_psi2(f_gpsi,hexapod.moving.psi,hexapod.an_thr*ones(3,1));

                weighh=9;
                do=[(-hexapod.moving.xyz-rg0)*1000; (-gpsi0)*180/pi*weighh];
                Ao=[1000*eye(3), A*1000; zeros(3,3), B*180/pi*weighh];
                do=double(do); Ao=double(Ao);
                AtA = Ao'*Ao; Ab = Ao'*do; 

                blx = -vmax; bux = vmax;
                
                % lbfgs
                fcn     = @(x) norm( Ao*x - do)^2;
                grad2    = @(x) 2*( AtA*x - Ab );
                fun     = @(x)fminunc_wrapper( x, fcn, grad2); 
                opts    = struct( 'factr', 1e5, 'pgtol', 1e-5, 'm', 10, 'maxIts',50,'printEvery',500);
                [xk, ~, info] = lbfgsb(fun, blx, bux, opts ); %xk'*1000

%                 % quadprog
%                 options = optimoptions('quadprog','Algorithm','interior-point-convex','Display','off');
%                 [xk,~,~,outt] = quadprog(AtA,-Ab,[],[],[],[],blx,bux,zeros(6,1),options);

                hexapod.moving.xyz = hexapod.moving.xyz + xk(1:3);
                hexapod.moving.psi = hexapod.moving.psi + xk(4:6);
        end

    else % kappa<=1
        hexapod.moving.xyz = hexapod.moving.xyz_de;
        hexapod.moving.psi = hexapod.moving.psi_de;
    end    
end

% keep hexapod.moving.psi is within [-pi, pi]
hexapod.moving.psi = hexapod.moving.psi - round(hexapod.moving.psi/2/pi)*2*pi;
