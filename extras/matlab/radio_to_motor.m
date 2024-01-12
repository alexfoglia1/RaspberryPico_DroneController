function msignal = radio_to_motor (rsignal, armed_threshold)
   rsignal_percentage = (rsignal - 1000) / 1000;
   msignal = 1000 + armed_threshold + rsignal_percentage * (1000 - armed_threshold);
endfunction
