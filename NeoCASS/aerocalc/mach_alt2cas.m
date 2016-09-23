function cas = mach_alt2cas(mach, alt)

  [rho, p, T, A, mu] = ISA_h(alt);
  dp_over_p = mach2dp_over_p(mach);
  dp = dp_over_p * p;
  cas = dp2cas(dp);

end