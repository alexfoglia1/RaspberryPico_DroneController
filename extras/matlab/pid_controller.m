function [pid_u, ek, ik] = pid_controller (ysp, y, kp, ki, kd, dt, sat, ikm1, ekm1)
    %  updates actual values
    ek = ysp - y;
    ik = ikm1 + (ek * dt);
    dk = (ek - ekm1) / dt;

    %
    P = ek * kp;
    I = ik * ki;
    D = dk * kd;
    ftmp = P + I + D;

    pid_u = minMax(ftmp, -sat, sat);

endfunction


function out = minMax(in, min, max)
  if in < min
    out = min;
  elseif in > max
    out = max;
  else
    out = in;
  endif
endfunction
