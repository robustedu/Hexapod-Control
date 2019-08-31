function [xyz_in_MOVING,psi_in_MOVING] = generate_target_in_MOVING(target, hexapod, couch)

% Generate target.xyz/psi_in_MOVING based on a given reasonable target.xyz/psi

Q.target = ZZ_Euler_2_DCM_P123(target.psi);
Q.couch = ZZ_Euler_2_DCM_P123(couch.psi);
Q.moving_in_COUCH = ZZ_Euler_2_DCM_P123(hexapod.moving.psi_in_COUCH);
Q.moving = Q.moving_in_COUCH*Q.couch;

hexapod_moving.xyz = couch.xyz+Q.couch'*hexapod.moving.xyz_in_COUCH;

Q.target_in_MOVING = Q.target*Q.moving';
psi_in_MOVING = Q2psi(Q.target_in_MOVING);
xyz_in_MOVING = Q.moving*(target.xyz - hexapod_moving.xyz);
