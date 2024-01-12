function [pid_u, ykm1, ikm1, dkm1] = pid_controller (ysp, y, kp, ki, kt, sat, ad, bd, ykm1, ikm1, dkm1)
    %  updates actual values
    error = ysp - y;
    %
    P = error * kp;
    D = dkm1 * ad - (y - ykm1) * bd;
    ftmp = P + ikm1 + D;
    pid_u = minMax(ftmp, -sat, sat);
    %
    I = ikm1 + error * ki;
    I += (pid_u - ftmp) * kt;
    %
    ykm1 = y;
    ikm1 = I;
    dkm1 = D;

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
