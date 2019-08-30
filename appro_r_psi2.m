function [A, g_psi0]=appro_r_psi2(f_g_psi, u_psi, du_psi)

t3=[];
g_psi0=f_g_psi(u_psi);
ttt=[repmat(u_psi,1,3)+diag(du_psi), repmat(u_psi,1,3)-diag(du_psi)]; % [u_psi-du_psi, u_psi+du_psi];
for i=1:6
    g_psi=f_g_psi(ttt(:,i))-g_psi0; g_psi = g_psi-round(g_psi/2/pi)*2*pi;
    t3=[t3 g_psi];
end
t1=repmat(du_psi',3,1)*2;
A=(t3(:,1:3)-t3(:,4:6))./t1;
% [sum(All(:,2:4)')
% sum(All(:,5:7)')
% (-sum(All(:,2:4)')+sum(All(:,5:7)'))/2]
% A*du_psi
% All(:,8:9)'
