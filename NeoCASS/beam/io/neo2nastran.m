function neo2nastran_v3(varargin)
%
% neo2nastran_v3(beam_model)
% neo2nastran_v3(filenameNeo)
% neo2nastran_v3(beam_model, filenameNastran)
% neo2nastran_v3(filenameNeo, filenameNastran)
%
% Converts a NeoCASS model in a Nastran model.
%
% The input NeoCASS model can be provided either as a beam_model structure or 
% as the name of the Smartcad input file (filenameNeo).
%
% The Nastran model will be saved in a file with the name specified by 
% the input parameter filenameNastran. It filenameNastran is not specified 
% a default name will be used, based on the name of the Smartcad file that 
% generated the beam_model, and with the "_str" string appended.
%
%
%-------------------------------------------------------------------------------
% 19-04-2016 v3.0 Federico Fonte
%

% Get beam_model
if isstruct(varargin{1})
	beam_model = varargin{1};
else
	filenameNeo = varargin{1};
	beam_model = load_nastran_model(filenameNeo);
end

if length(varargin)<2 || isempty(varargin{2})
	dotpos = find(beam_model.Param.FILE == '.')-1;
	headname = beam_model.Param.FILE(1:dotpos);
	filenameNas = [headname, '_str.nas'];
else
	filenameNas = varargin{2};
end

Info = beam_model.Info;
fid = fopen(filenameNas, 'w');


nrbe2 = 0;


fprintf(fid, '$\n$ Warning: remember to include NASTRAN BARMASS=1 \n$          at the begin of NASTRAN file.\n$');

fprintf(fid,'$-------2-------3-------4-------5-------6-------7-------8-------9-------10\n');
fprintf(fid,'$ Node definition                                                         \n');
fprintf(fid,'$-------2-------3-------4-------5-------6-------7-------8-------9-------10\n');

select = 1:(Info.ngrid - Info.nbar);

writeGRID(fid, beam_model.Node.ID(select), [], beam_model.Node.Coord(select,:), beam_model.Node.CD(select));

fprintf(fid,'$-------2-------3-------4-------5-------6-------7-------8-------9-------10\n');
fprintf(fid,'$ Material properties                                                     \n');
fprintf(fid,'$-------2-------3-------4-------5-------6-------7-------8-------9-------10\n');

for i=1:Info.nmat

  MID = beam_model.Mat.ID(i);
  E   = beam_model.Mat.E(i);
  G   = [];
  NU  = beam_model.Mat.nu(i);
  RHO = beam_model.Mat.Rho(i);
  writeMAT1(fid, MID, E, G, NU, RHO)

end

fprintf(fid,'\n$-------2-------3-------4-------5-------6-------7-------8-------9-------10');
fprintf(fid,'\n$ Bar properties');
fprintf(fid,'\n$-------2-------3-------4-------5-------6-------7-------8-------9-------10');

for i=1:Info.npbar

  id = sprintf('%-8d', beam_model.PBar.ID(i));
  pid = sprintf('%-8d', beam_model.Mat.ID(beam_model.PBar.Mat(i)));
  A = sprintf('%-8g', beam_model.PBar.A(i));
  I1 = sprintf('%-8g', beam_model.PBar.I(i,1));
  I2 = sprintf('%-8g', beam_model.PBar.I(i,2));
  J = sprintf('%-8g', beam_model.PBar.J(i));
  RHO = sprintf('%-8g', beam_model.PBar.RhoNS(i));
  fprintf(fid,'\nPBAR    %s%s%s%s%s%s%s\n        ', id, pid, A, I1, I2, J, RHO);
  for n=1:4
    c1 = sprintf('%-8g', beam_model.PBar.Str_point(n,1,i));
    c2 = sprintf('%-8g', beam_model.PBar.Str_point(n,2,i));
    fprintf(fid,'%s%s', c1, c2);
  end
%  K1 = sprintf('%-8f', beam_model.PBar.Kshear(i,1));
%  K2 = sprintf('%-8f', beam_model.PBar.Kshear(i,2));
%  I3 = sprintf('%-8f', beam_model.PBar.I(i,3));
%  fprintf(fid,'\n%s%s%s', K1, K2, I3);

end

fprintf(fid,'\n');
fprintf(fid,'$-------2-------3-------4-------5-------6-------7-------8-------9-------10\n');
fprintf(fid,'$ Bar elements\n');
fprintf(fid,'$-------2-------3-------4-------5-------6-------7-------8-------9-------10\n');

for i=1:Info.nbar

  EID = beam_model.Bar.ID(i);
  PID = beam_model.PBar.ID(beam_model.Bar.PID(i));
  G = beam_model.Node.ID(beam_model.Bar.Conn(i,[1,3]));
  X = beam_model.Bar.Orient(i,1:3);
  OFFT = 'GGG';
  
  writeCBAR(fid, EID, PID, G, X, OFFT);

end

fprintf(fid,'$-------2-------3-------4-------5-------6-------7-------8-------9-------10\n');
fprintf(fid,'$ Lumped masses                                                           \n');
fprintf(fid,'$-------2-------3-------4-------5-------6-------7-------8-------9-------10\n');
ncom = 0;
for i=1:Info.nconm
  ncom = ncom+1;

  % Check if the mass can be written in CONM2 format
  M = diag(beam_model.ConM.M(1:3,1:3, ncom));
  if max(abs(M-M(1))) < 1e-10
    [M, X, I] = massMatrix2ConmData(beam_model.ConM.M(:,:,ncom)); 
    EID = ncom;
    G = beam_model.Node.ID(beam_model.ConM.Node(ncom));
    writeCONM2(fid, EID, G, 0, M, X, I);
  else
    id = sprintf('%-8d', ncom);
    n = sprintf('%-8d', beam_model.Node.ID(beam_model.ConM.Node(ncom)));
    M11 = sprintf('%-8g', beam_model.ConM.M(1,1,ncom));
    M21 = sprintf('%-8g', beam_model.ConM.M(2,1,ncom));
    M22 = sprintf('%-8g', beam_model.ConM.M(2,2,ncom));
%
    M31 = sprintf('%-8g', beam_model.ConM.M(3,1,ncom));
    M32 = sprintf('%-8g', beam_model.ConM.M(3,2,ncom));
    M33 = sprintf('%-8g', beam_model.ConM.M(3,3,ncom));
%
    M41 = sprintf('%-8g', beam_model.ConM.M(4,1,ncom));
    M42 = sprintf('%-8g', beam_model.ConM.M(4,2,ncom));
    M43 = sprintf('%-8g', beam_model.ConM.M(4,3,ncom));
    M44 = sprintf('%-8g', beam_model.ConM.M(4,4,ncom));
%
    M51 = sprintf('%-8g', beam_model.ConM.M(5,1,ncom));
    M52 = sprintf('%-8g', beam_model.ConM.M(5,2,ncom));
    M53 = sprintf('%-8g', beam_model.ConM.M(5,3,ncom));
    M54 = sprintf('%-8g', beam_model.ConM.M(5,4,ncom));
    M55 = sprintf('%-8g', beam_model.ConM.M(5,5,ncom));
%
    M61 = sprintf('%-8g', beam_model.ConM.M(6,1,ncom));
    M62 = sprintf('%-8g', beam_model.ConM.M(6,2,ncom));
    M63 = sprintf('%-8g', beam_model.ConM.M(6,3,ncom));
    M64 = sprintf('%-8g', beam_model.ConM.M(6,4,ncom));
    M65 = sprintf('%-8g', beam_model.ConM.M(6,5,ncom));
    M66 = sprintf('%-8g', beam_model.ConM.M(6,6,ncom));
%
    fprintf(fid,'\nCONM1   %s%s        %s%s%s%s%s', id, n, M11,M21,M22,M31,M32);
    fprintf(fid,'\n        %s%s%s%s%s%s%s%s', M33,M41,M42,M43,M44,M51,M52,M53);
    fprintf(fid,'\n        %s%s%s%s%s%s%s%s', M54,M55,M61,M62,M63,M64,M65,M66);
%
  end
end

if (Info.nrbe0)
  fprintf(fid,'\n$-------2-------3-------4-------5-------6-------7-------8-------9-------10');
  fprintf(fid,'\n$ Box cross nodes');
  fprintf(fid,'\n$-------2-------3-------4-------5-------6-------7-------8-------9-------10');
  for i=1:length(beam_model.Node.Aero.Index)
    if ~isempty(beam_model.Node.Aero.Index(i).data)
      nrbe2 = nrbe2+1;
      cont = sprintf('%-8d', nrbe2);
      id = sprintf('%-8d', beam_model.Node.ID(i));
      fprintf(fid,'\nRBE2    %s%s 123456 ', cont, id);
      for k=1:length(beam_model.Node.Aero.Index(i).data)
        id = sprintf('%-8d', beam_model.Node.ID(beam_model.Node.Aero.Index(i).data(k)));
        fprintf(fid,'%s', id);
      end
    end
  end
end

fprintf(fid,'\n$\n');


if ~isempty(beam_model.Param.SUPORT)
	fprintf(fid,'$-------2-------3-------4-------5-------6-------7-------8-------9-------10\n');
	fprintf(fid,'$ Suport definition                                                       \n');
	fprintf(fid,'$-------2-------3-------4-------5-------6-------7-------8-------9-------10\n');
	writeSUPORT(fid, beam_model.Param.SUPORT(1), beam_model.Param.SUPORT(2));
	isSuport = true;
	suportID = beam_model.Param.SUPORT(1);
else
	isSuport = false;
end


if (Info.nrbe2)
  fprintf(fid,'$-------2-------3-------4-------5-------6-------7-------8-------9-------10\n');
  fprintf(fid,'$ Rigid connections                                                       \n');
  fprintf(fid,'$-------2-------3-------4-------5-------6-------7-------8-------9-------10\n');
  for i=1:Info.nrbe2
    nrbe2 = nrbe2+1;
    cont = sprintf('%-8d', nrbe2);

    masterNode = beam_model.RBE2.IDM(i);
    slaveNodes = beam_model.RBE2.IDS(i).data;
    if isSuport && ~isempty(find(slaveNodes==suportID))
      position = find(slaveNodes, suportID);
      slaveNodes(position) = masterNode;
      masterNode = suportID;
      fprintf('WARNING !!! RBE2 %d has been modified in order to obtain SUPORT as master\n', nrbe2);
      fprintf('            this could lead to inconsistency in the MPC definition    \n');
    end
       
    id = sprintf('%-8d', masterNode);
    fprintf(fid,'RBE2    %s%s%s', cont, id);
    gdl = [];
    k=0;

    for k=1:length(beam_model.RBE2.GDL(i).data)
      id = sprintf('%-d', beam_model.RBE2.GDL(i).data(k));
      fprintf(fid,'%c', id);
    end
    fprintf(fid, '%c', blanks(8-k));
    for k=1:length(slaveNodes)
      id = sprintf('%-8d', slaveNodes(k));
      fprintf(fid,'%s', id);
    end
    fprintf(fid,'\n');
  end
end



if (Info.ncaero)
	fprintf(fid,'$-------2-------3-------4-------5-------6-------7-------8-------9-------10\n');
	fprintf(fid,'$ Aerodynamic panels                                                      \n');
	fprintf(fid,'$-------2-------3-------4-------5-------6-------7-------8-------9-------10\n');
	PID = 10000;
	caeroIDscale = 10000;
	caeroIDdelta = 1000;
	CIDnew = 1000;

	fprintf(fid,'PAERO1  %d\n', PID);

	for iCaero = 1:Info.ncaero

		DIH = beam_model.Aero.geo.dihed(iCaero)*180/pi;
		SPN = beam_model.Aero.geo.b(iCaero);
		CHD = beam_model.Aero.geo.c(iCaero);
		CX = beam_model.Aero.geo.startx(iCaero);
		CY = beam_model.Aero.geo.starty(iCaero);
		CZ = beam_model.Aero.geo.startz(iCaero);
		TPR = beam_model.Aero.geo.T(iCaero);
		SWP = beam_model.Aero.geo.SW(iCaero)*180/pi;

		EID = beam_model.Aero.ID(iCaero)*caeroIDscale;
		CP = 0;
		IGID = 1;

		[P1, P2, c1, c2] = CAERO_neo2nas(DIH, SPN, CHD, CX, CY, CZ, TPR, SWP);

		NSPAN  = beam_model.Aero.geo.ny(iCaero);
		NCHORD = beam_model.Aero.geo.nx(iCaero);

		if beam_model.Aero.geo.flapped(iCaero);

			EIDsurf = EID + caeroIDdelta;

			nChordFixed = beam_model.Aero.geo.nx(iCaero);
			nChordSurf  = beam_model.Aero.geo.fnx(iCaero);

			c1_aft = beam_model.Aero.geo.fc(iCaero,1)*c1;
			c2_aft = beam_model.Aero.geo.fc(iCaero,2)*c2;

			writeCAERO_NAS(fid, EID, PID, CP, NSPAN, NCHORD, IGID, P1, c1-c1_aft, P2, c2-c2_aft);

			point1 = P1 + [c1-c1_aft;0;0];
			point2 = P2 + [c2-c2_aft;0;0];

			writeCAERO_NAS(fid, EIDsurf, PID, CP, NSPAN, nChordSurf, IGID, point1, c1_aft, point2, c2_aft);

			% Select the panels defining the control surface
			surfPanels = EIDsurf + (1:NSPAN*nChordSurf) - 1;

			writeGenericSet(fid, 'AELIST', EIDsurf, surfPanels);

			[X0, R] = getSpline1ReferenceSystem(point1, point2);
			CIDnew = CIDnew + 1;
			writeCORD2R(fid, CIDnew, 0, X0, R);

			if ~isempty(beam_model.Aero.lattice_vlm)
				index = find(beam_model.Aero.lattice_vlm.Control.Patch == iCaero);
				LABEL = beam_model.Aero.lattice_vlm.Control.Name{index};
			elseif ~isempty(beam_model.Aero.lattice_dlm)
				index = find(beam_model.Aero.lattice_dlm.Control.Patch == iCaero);
				LABEL = beam_model.Aero.lattice_dlm.Control.Name{index};
			elseif isfield(beam_model.Aero.geo, 'controlName')
				index = find(find(beam_model.Aero.geo.flapped) == iCaero);
				LABEL = beam_model.Aero.geo.controlName{index};
			else
				error('lattice_vlm and lattice_dlm not defined')
			end

			writeAESURF_NAS(fid, EIDsurf, LABEL, CIDnew, EIDsurf);
		else
			writeCAERO_NAS(fid, EID, PID, CP, NSPAN, NCHORD, IGID, P1, c1, P2, c2);
		end
		fprintf(fid,'$\n');
	end

	fprintf(fid,'$-------2-------3-------4-------5-------6-------7-------8-------9-------10\n');
	fprintf(fid,'$ Spline definition                                                       \n');
	fprintf(fid,'$-------2-------3-------4-------5-------6-------7-------8-------9-------10\n');

	splineIDdelta = caeroIDdelta;

	nSpline = length(beam_model.Aero.Interp.ID);

	for iSpline = 1:nSpline

		EID = beam_model.Aero.Interp.ID(iSpline);
		CAERO = beam_model.Aero.ID(beam_model.Aero.Interp.Patch(iSpline))*caeroIDscale;
		BOX1_loc = beam_model.Aero.Interp.Index(iSpline,1);
		BOX2_loc = beam_model.Aero.Interp.Index(iSpline,2);
		SETG = beam_model.Aero.Set.ID(beam_model.Aero.Interp.Set(iSpline));

		DZ = [];
		USAGE = [];

		iCaero = beam_model.Aero.Interp.Patch(iSpline);
		if beam_model.Aero.geo.flapped(iCaero)

			CAEROsurf = CAERO + caeroIDdelta;

			NSPAN  = beam_model.Aero.geo.ny(iCaero);
			nChordFixed = beam_model.Aero.geo.nx(iCaero);
			nChordSurf  = beam_model.Aero.geo.fnx(iCaero);

			caeroPanels = zeros(nChordFixed+nChordSurf, NSPAN);
			caeroPanels(BOX1_loc:BOX2_loc) = 1;

			frontPanels = find(caeroPanels(1:nChordFixed,:));
			surfPanels = find(caeroPanels(nChordFixed+(1:nChordSurf),:));

			BOX1 = CAERO + frontPanels(1) - 1;
			BOX2 = CAERO + frontPanels(end) - 1;
			BOX1surf = CAEROsurf + surfPanels(1) - 1;
			BOX2surf = CAEROsurf + surfPanels(end) - 1;

			flapped = true;
		else
			BOX1 = CAERO + BOX1_loc - 1;
			BOX2 = CAERO + BOX2_loc - 1;
			flapped = false;
		end

		switch beam_model.Aero.Interp.Type(iSpline)
		case 1
			DTOR = [];
			DTHX = [];
			DTHY = [];

			setNodes = beam_model.Aero.Set.Node(beam_model.Aero.Interp.Set(iSpline)).data;
			ID1 = beam_model.Aero.Interp.Param(iSpline,1);
			ID2 = beam_model.Aero.Interp.Param(iSpline,2);
			[X0, R] = getSpline1ReferenceSystem(ID1, ID2, setNodes, beam_model.Node.Coord);

			CIDnew = CIDnew + 1;
			writeCORD2R(fid, CIDnew, 0, X0, R);

			writeSPLINE2_NAS(fid, EID, CAERO, BOX1, BOX2, SETG, DZ, DTOR, CIDnew, DTHX, DTHY, USAGE);
			fprintf(fid,'$\n');

			if flapped
				writeSPLINE2_NAS(fid, EID+splineIDdelta, CAEROsurf, BOX1surf, BOX2surf, SETG, DZ, DTOR, CIDnew, DTHX, DTHY, USAGE);
				fprintf(fid,'$\n');
			end

		case 2
			weight = beam_model.Aero.Interp.Param(iSpline,2);
			switch weight
			case 2
				METH = 'TPS';
			otherwise
				METH = [];
			end

			writeSPLINE1_NAS(fid, EID, CAERO, BOX1, BOX2, SETG, DZ, METH, USAGE, [], []);
			fprintf(fid,'$\n');
			if flapped
				writeSPLINE1_NAS(fid, EID+splineIDdelta, CAEROsurf, BOX1surf, BOX2surf, SETG, DZ, METH, USAGE, [], []);
				fprintf(fid,'$\n');
			end

		otherwise
			fprintf(' !!! WARNING spline %d has a non-recognized type\n', iSpline);
		end
	end
	fprintf(fid,'$\n');

	fprintf(fid,'$-------2-------3-------4-------5-------6-------7-------8-------9-------10\n');
	fprintf(fid,'$ Interpolation set                                                       \n');
	fprintf(fid,'$-------2-------3-------4-------5-------6-------7-------8-------9-------10\n');

	nSet = length(beam_model.Aero.Set.ID);

	for iSet = 1:nSet
		nodeList = beam_model.Node.ID(beam_model.Aero.Set.Node(iSet).data);
		writeGenericSet(fid, 'SET1', beam_model.Aero.Set.ID(iSet), nodeList);
		fprintf(fid,'$\n');
	end


	fprintf(fid,'$-------2-------3-------4-------5-------6-------7-------8-------9-------10\n');
	fprintf(fid,'$ Links on control surfaces                                               \n');
	fprintf(fid,'$-------2-------3-------4-------5-------6-------7-------8-------9-------10\n');

	trimID = 1000;

	nAelink = length(beam_model.Aero.Trim.Link.ID);

	for iAelink = 1:nAelink
		LABLD = beam_model.Aero.Trim.Link.Slave{iAelink};
		LABL  = beam_model.Aero.Trim.Link.Master{iAelink};
		C = -beam_model.Aero.Trim.Link.Coeff(iAelink);

		writeAELINK(fid, trimID, LABLD, LABL, C);
	end
	fprintf(fid,'$\n');

end


fprintf(fid,'$--------------- end of file --------------------\n');
fclose(fid);
