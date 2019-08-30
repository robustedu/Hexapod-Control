close all
figure(1); set(gcf,'units','normalized','outerposition',[0 0 1 1])

tt = (target.xyz_in_MOVING_all-repmat(xyz_in_MOVING0,1,NNN))*1000;
subplot(2,5,1); plot(control.T,tt); ylabel(' target xyz (mm)');xlim([0,max(control.T)]);
xlabel('Time (sec)'); grid on; legend('X','Y','Z')
title('target (-const) in end-effector frame ')
tt = (target.psi_in_MOVING_all-repmat(psi_in_MOVING0,1,NNN))*180/pi; tt = tt-round(tt/180)*180;
subplot(2,5,6); plot(control.T,tt); ylabel('target \alpha\beta\gamma (deg)') 
xlabel('Time (sec)'); grid on; legend('pitch','roll','yaw');xlim([0,max(control.T)]);

tt = hexapod.moving.xyz_all*1000; tt=tt-repmat(tt(:,1),1,NNN);
subplot(2,5,2); plot(control.T,tt); ylabel(' end effector xyz (mm)');xlim([0,max(control.T)]);
xlabel('Time (sec)'); grid on; legend('X','Y','Z')
title('end effector (-const) in LINAC frame')
tt = hexapod.moving.psi_all*180/pi; tt=tt-repmat(tt(:,1),1,NNN);  tt = tt-round(tt/360)*360;
subplot(2,5,7); plot(control.T,tt); ylabel(' end effector \alpha\beta\gamma (deg)') 
xlabel('Time (sec)'); grid on; legend('pitch','roll','yaw');xlim([0,max(control.T)]);

tt = target.xyz_all*1000;
subplot(2,5,3); plot(control.T,tt); ylabel(' target xyz (mm)');xlim([0,max(control.T)]);
xlabel('Time (sec)'); grid on; legend('X','Y','Z')
title('target in LINAC frame')
tt = target.psi_all*180/pi;
subplot(2,5,8); plot(control.T,tt); ylabel('target \alpha\beta\gamma (deg)') 
xlabel('Time (sec)'); grid on; legend('pitch','roll','yaw');xlim([0,max(control.T)]);

tt1 = target.xyz_all*1000; tt2 = target.psi_all*180/pi;
ttt1 = sqrt(sum(tt1.*tt1)); ttt2 = sqrt(sum(tt2.*tt2));
tt = [ttt1; ttt2; ttt1+ttt2];
subplot(2,5,4); plot(control.T,tt); ylabel(' error (mm/deg)');xlim([0,max(control.T)]);
xlabel('Time (sec)'); grid on; legend('tranation','rotation','all')
title('target position error')
tt = hexapod.leg_length_all*1000;
subplot(2,5,9); plot(control.T,tt); ylabel(' leg length (mm)');xlim([0,max(control.T)]);
xlabel('Time (sec)'); grid on;
title('Leg length')


tt = target.xyz_all*1000;
subplot(2,5,5); plot3(tt(1,:),tt(2,:),tt(3,:)); hold on; plot3(tt(1,:),tt(2,:),tt(3,:),'o'); 
plot3(0,0,0,'b+','markersize',20);
xlabel(' x (mm)'); ylabel(' y (mm)'); zlabel(' z (mm)'); grid on; axis equal; axis tight
title('target in LINAC frame')
tt = target.psi_all*180/pi;
subplot(2,5,10); plot3(tt(1,:),tt(2,:),tt(3,:)); hold on; plot3(tt(1,:),tt(2,:),tt(3,:),'o'); 
plot3(0,0,0,'b+','markersize',20);
xlabel(' pitch (deg)'); ylabel(' roll (deg)'); zlabel(' yaw (deg)'); grid on; axis equal; axis tight
title('target in LINAC frame (angles)')
