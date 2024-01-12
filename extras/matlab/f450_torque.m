function torque = f450_torque (m1, m2, m3, m4)
  Cr = 46.48; % throttle percentage to rpm
  Ct = 2e-06; % thrust coefficient [N * RPM^-2]
  Cq = 2.1e-07; % torque coefficient [N * m * RPM^-2]
  omega = [m1 * Cr * 100; m2 * Cr * 100; m3 * Cr * 100; m4 * Cr * 100];
  omega2 = omega.^2;
  dx = 0.185; % arm length [m]
  Mt = [Ct,Ct,Ct,Ct;
        -dx*Ct, -dx*Ct, dx*Ct, -dx*Ct;
        -dx*Ct, -dx*Ct, dx*Ct, dx*Ct;
        -Cq, Cq, -Cq, Cq;];

  torque = Mt * omega2;
endfunction
