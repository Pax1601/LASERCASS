function str = run_optim_Zst_sk_panel_10(niter, bk_sol, Zs, tbs, NSPAR, esw, msl, FS, M, Mt, rpitch, spitch, Acap, tweb, output)
%
  NITN = 4; KTRIAL = 10; PERT_AMPL = 10.0;
%
  Nsec = length(FS);
%
  save(output)
  if (NITN>0)
    CONV_HISTO = zeros(Nsec, NITN+1);
  end
%
  if (niter==1)
    XBK_SK(1) = 1.0e-3; 
    XBK_SK(2) = 1.0e-4;
    XBK_SK(3) = 2.0e-3;
    XBK_SK(4) = 0.3;
    XBK_SK(5) = 0.3;
    XBK_SK(6) = tbs(1);
  else
    XBK_SK(1) = bk_sol.tskin(1); 
    XBK_SK(2) = bk_sol.Astr(1); 
    XBK_SK(3) = bk_sol.trib(1); 
    XBK_SK(4) = bk_sol.D1_rib(1); 
    XBK_SK(5) = bk_sol.D2_rib(1); 
    XBK_SK(6) = bk_sol.RP(1); 
  end
  str.tskin   = zeros(Nsec,1);
  str.Astr    = zeros(Nsec,1);
  str.Nstr    = zeros(Nsec,1);
  str.trib    = zeros(Nsec,1);
  str.D1_rib  = zeros(Nsec,1);
  str.D2_rib  = zeros(Nsec,1);
  str.RP      = zeros(Nsec,1);

%
  if (min(Zs/NSPAR)<spitch)
    fprintf('\n\t### Warning: stringer pitch exceeds minimum wing chord.');
    fprintf('\n\t             Two stringers will be used for each cell.');
  end
  str.Nstr = (round(Zs/(NSPAR-1)/spitch)+1)*(NSPAR-1) - (NSPAR-2);
  index = find(str.Nstr<NSPAR);
  if (~isempty(index))
    str.Nstr(index) = NSPAR;
  end
%
  for i=1:Nsec
  fprintf('\nSection %d', i);
%
    [SOL, flag, fun, CUNS] = optim_Zst_sk_panel_10(Zs(i), tbs(i), NSPAR, Acap(i), tweb(i), str.Nstr(i), rpitch, esw, msl, 0.0, FS(i), M(i), Mt(i), [], XBK_SK);
    if (~isempty(CUNS))
      fprintf('\n\n### Warning: optimization failed.');
      fprintf('\n    Constraints will be relaxed. ');
      SOL = XBK_SK;
    end
    K=1;
    while ( (~isempty(CUNS)) && (K<=KTRIAL) )
      fprintf('\n\n\t\tTrial %d: \n', K);
      CEVAL = CUNS;
      [SOL, flag, fun, CUNS] = optim_Zst_sk_panel_10(Zs(i), tbs(i), NSPAR, Acap(i), tweb(i), ...
                    str.Nstr(i), rpitch, esw, msl, ...
                    0.0, FS(i), M(i), Mt(i),...
                    CEVAL, SOL);
      [SOL, flag, fun, CUNS] = optim_Zst_sk_panel_10(Zs(i), tbs(i), NSPAR, Acap(i), tweb(i), ...
                    str.Nstr(i), rpitch, esw, msl, ...
                    0.0, FS(i), M(i), Mt(i),...
                    [], SOL);
      K = K + 1;
      if (isempty(CUNS))
        fprintf('feasible.');
      else
        SOL = PERT_AMPL .* SOL;
      end
        fprintf('NOT feasible.');
    end
%
    if ( (~isempty(CUNS)) && (K==11) )
      XBK_SK = PERT_AMPL .* XBK_SK;
      error('Unable to find an initial feasible solution.');
    end
    XBK_SK = SOL;
%           
    if (NITN>0)
      XL(1) = 0.001; 
      XL(2) = 2.5e-5;
      XL(3) = tbs(i)/240;
      XL(4) = 0.3;
      XL(5) = 0.38;
      XL(6) = rpitch*2;
      coeff = [];
      for NIT = 2:NITN+1
        coeff = [coeff, 1+0.1*NIT];
      end    
  %
      SUB_BK      = zeros(6, NITN+1);
      SUB_BK(:,1) = XBK_SK;
      OBJ_BK      = zeros(1, NITN+1);    
      OBJ_BK(1) = fun;
      for NIT = 1:NITN
%
        fprintf('\n\tPerturbation %d: ', NIT);
        XPERT = XL .* coeff(NIT);
        XPERT(end) = XL(end) / coeff(NIT);
        [SOL, flag, fun, CUNS] = optim_Zst_sk_panel_10(Zs(i), tbs(i), NSPAR, Acap(i), tweb(i), ...
                      str.Nstr(i), rpitch, esw, msl, ...
                      0.0, FS(i), M(i), Mt(i),...
                      [], XBK_SK);
        if (flag>0)
          SUB_BK(:,NIT+1)   = SOL;
          OBJ_BK(NIT+1)     = fun;
          fprintf('feasible.');
        else
          OBJ_BK(NIT+1)     = Inf;
          fprintf('NOT feasible.');
        end
      end
      [~, index] = min(OBJ_BK);
      SOL = SUB_BK(:,index);
      CONV_HISTO(i, :) = OBJ_BK;
    end
    XBK_SK = SOL;
    str.tskin(i)  = SOL(1);
    str.Astr(i)   = SOL(2);
    str.trib(i)   = SOL(3);
    str.D1_rib(i) = SOL(4);
    str.D2_rib(i) = SOL(5);
    str.RP(i)     = SOL(6);
    str.histo    = CONV_HISTO;
end
%
end