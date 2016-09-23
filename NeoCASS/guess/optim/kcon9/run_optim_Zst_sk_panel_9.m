function [str, strw] = run_optim_Zst_sk_panel_9(niter, bksk, bksp, Zs, tbs, esw, msl, Nax, FS, M, Mt, rpitch, spitch, output)
%
  NITN = 4; KTRIAL = 20; PERT_AMPL = 10.0;
%
  Nsec = length(FS);
%
  save(output);
  if (NITN>0)
    CONV_HISTO = zeros(Nsec, NITN+1);
    XL = 0.001 .* ones(3,1); XL(3) = 1.0e-5;
  end
  if (niter==1)
    XBK_SK(1) = 1.0e-3; 
    XBK_SK(2) = 1.0e-3;
    XBK_SK(3) = 1.0e-4;
  else
    XBK_SK(1) = bksk.tskin(1); 
    XBK_SK(2) = bksp.tw(1); 
    XBK_SK(3) = bksk.Astr(1); 
  end
  str.tskin = zeros(Nsec,1);
  strw.tw   = zeros(Nsec,1);
  str.Astr  = zeros(Nsec,1);
  str.Nstr  = zeros(Nsec,1);
%
  if (min(Zs)<spitch)
    fprintf('\n\t### Warning: stringer pitch exceeds minimum wing chord.');
    fprintf('\n\t             Two stringers will be used.');
  end
  str.Nstr = round(Zs/spitch)+1;
  index = find(str.Nstr<2);
  if (~isempty(index))
    str.Nstr(index) = 2;
  end
%
  for i=1:Nsec
  fprintf('\nSection %d', i);
%
    [SOL, flag, fun, CUNS] = optim_Zst_sk_panel_9(Zs(i), tbs(i), str.Nstr(i), rpitch, esw, msl, Nax(i), 0.0, FS(i), M(i), Mt(i), [], XBK_SK);
    if (~isempty(CUNS))
      fprintf('\n\n### Warning: optimization failed.');
      fprintf('\n    Constraints will be relaxed. ');
      SOL = XBK_SK;
    end
    K=1;
    while ( (~isempty(CUNS)) && (K<=KTRIAL) )
      fprintf('\n\n\t\tTrial %d: \n', K);
      CEVAL = CUNS; 
      [SOL, flag, fun, CUNS] = optim_Zst_sk_panel_9(Zs(i), tbs(i), str.Nstr(i), rpitch, esw, msl, ...
                    Nax(i), 0.0, FS(i), M(i), Mt(i), CEVAL, SOL);
      [SOL, flag, fun, CUNS] = optim_Zst_sk_panel_9(Zs(i), tbs(i), str.Nstr(i), rpitch, esw, msl, ...
                    Nax(i), 0.0, FS(i), M(i), Mt(i), [], SOL);
      K = K + 1;
      if (isempty(CUNS))
        fprintf('feasible.');
      else
        SOL = PERT_AMPL .* SOL;
        fprintf('NOT feasible.');
      end
    end
%
    if ( (~isempty(CUNS)) && (K==11) )
      XBK_SK = PERT_AMPL .* XBK_SK;
      error('Unable to find an initial feasible solution.');
    end
    XBK_SK = SOL;
%           

    if (NITN>0)
      
      coeff = [];
      for NIT = 2:NITN+1
        coeff = [coeff, 1+0.1*NIT];
      end    
  %
      SUB_BK      = zeros(3, NITN+1);
      SUB_BK(:,1) = XBK_SK;
      OBJ_BK      = zeros(1, NITN+1);    
      OBJ_BK(1) = fun;

      for NIT = 1:NITN
        fprintf('\n\tPerturbation %d: ', NIT);
        XPERT = XL .* coeff(NIT);
        XPERT(end) = XL(end) / coeff(NIT);
        [SOL, flag, fun, CUNS] = optim_Zst_sk_panel_9(Zs(i), tbs(i), str.Nstr(i), rpitch, esw, msl, ...
                      Nax(i), 0.0, FS(i), M(i), Mt(i), [], XPERT);
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
    str.tskin(i) = SOL(1);
    str.Astr(i)  = SOL(3);
    strw.tw(i)   = SOL(2);
    str.histo    = CONV_HISTO;

end
%
end