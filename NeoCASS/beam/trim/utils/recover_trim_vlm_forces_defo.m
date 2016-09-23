function F0 = recover_trim_vlm_forces_defo(F0, SOL, CPaero, DEFO, CREF, BREF, SREF, VREF, ADOF)

  VREF2 = 2*VREF;
  UXVEC = zeros(5,1);
  UXVEC(1) = SOL.Alpha * pi/180;
  UXVEC(2) = SOL.Betha * pi/180;
  UXVEC(3) = SOL.P * CREF/VREF2;
  UXVEC(4) = SOL.Q * BREF/VREF2;
  UXVEC(5) = SOL.R * BREF/VREF2;
  Ftot = F0;
  for k=1:5
    Ftot = Ftot + CPaero.State(:,k).* UXVEC(k);
  end
  for k=1:length(SOL.Control)
    Ftot =  Ftot + CPaero.Control(:,k).* SOL.Control(k) * pi/180;
  end

  Ftot =  Ftot + CPaero.Defo * DEFO;

  naer = length(Ftot) / 3;
  F0 = zeros(naer, 3);
  for k=1:naer
    offset = (ADOF(k)-1)*3;
    F0(k,1) = Ftot(offset+1);
    F0(k,2) = Ftot(offset+2);
    F0(k,3) = Ftot(offset+3);
  end
