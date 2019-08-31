function psi = Q2psi(x)

% from DCM to Euler angle of EA123 (rad)

% Q2psi = @(x) (SpinCalc(CONVERSION,x,1e-10,1)'-round(SpinCalc(CONVERSION,x,1e-10,1)'/360)*360)/180*pi;

CONVERSION = 'DCMtoEA123';
psi = SpinCalc(CONVERSION,x,1e-10,1)';
psi = (psi-round(psi/360)*360)/180*pi;


