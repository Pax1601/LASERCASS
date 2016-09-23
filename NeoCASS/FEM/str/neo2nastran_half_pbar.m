% This function convert NeoCASS smartcard file to Nastran input file
% file1: NeoCASS file
% file2: Nastran file
% type is internal variable, to not define by user
%  neo2nastran_half_pbar(file1,file2,pbar)
%
%  Author: Lorenzo Travaglini
function neo2nastran_half_pbar(file1, file2, pbar, conm, type)
beam_model = [];
%
pbar = sort(pbar);
conm = sort(conm);
%
SM = fopen(file1,'r'); 
if exist('type','var')
    NA = fopen(file2,'a+');
else
    if exist(file2, 'file')
       delete(file2);
    end 
    NA = fopen(file2,'a+');
    fprintf(NA, '$\n$ Warning: remember to include NASTRAN BARMASS=1 \n$          at the begin of NASTRAN file.\n$');
end
IDcaero = [];
IDcaero2 = [];
IDcoord = 1;
IDsurf = 1;
indpaero = [];
riga = fgetl(SM);
nrbe = 0; 
while isempty(find(riga ==-1,1))
    if length(riga)<8
        fprintf(NA,'\n');
        riga = fgetl(SM);
    else
        switch riga(1:8)
            case 'INCLUDE '
                neo2nastran_half_pbar(riga(9:end),file2,pbar,conm,1);
                riga = fgetl(SM); 
            case 'MAT1    ' 
                fprintf(NA,['\n',riga(1:16),num2str8(str2double(riga(17:24))),riga(25:32),num2str8(str2double(riga(33:40))),num2str8(str2double(riga(41:48)))]);
                riga = fgetl(SM);
                fprintf(NA,['\n',riga(1:8),num2str8(str2double(riga(9:16))),num2str8(str2double(riga(17:24))),num2str8(str2double(riga(25:32)))]);
                riga = fgetl(SM);
            case 'CORD2R  ' 
                fprintf(NA,['\n',riga]);
                riga = fgetl(SM);
                fprintf(NA,['\n',riga]);
                riga = fgetl(SM);
            case 'GRID    '
                fprintf(NA,['\n',riga(1:24),num2str8(str2double(riga(25:32))),num2str8(str2double(riga(33:40))),num2str8(str2double(riga(41:48))),'        ',riga(57:end)]);
                riga = fgetl(SM);
            case 'CBAR    ' 
%               fprintf(NA,['\nCBEAM   ',riga(9:64)]);
                fprintf(NA,['\n',riga(1:64)]);
                riga = fgetl(SM);
            case 'RBE0    '
                nrbe = nrbe +1;
                fprintf(NA,['\nRBE2    ',cnvt2_8chs(nrbe),riga(17:24),'123456  ',riga(25:end)]);
                riga = fgetl(SM);
            case 'RBE2    '
                nrbe = nrbe +1;
                fprintf(NA,['\nRBE2    ',cnvt2_8chs(nrbe), riga(17:end)]);
                riga = fgetl(SM);
            case {'PBAR    ','PBARGFF ','PBARGMW ','PBARGFU ','PBARSM3 ','PBARSM4 ','PBARSM5 '} 
                  if isempty(beam_model)
                    beam_model = load_nastran_model(file1);
                  end
                    IDPBar = str2double(riga(9:16));
                   ind = find(beam_model.PBar.ID == IDPBar);
                  indh = find(pbar == IDPBar);
                  if isempty(indh)
                     fprintf(NA,['\nPBAR    ',riga(9:16),cnvt2_8chs(beam_model.Mat.ID(beam_model.PBar.Mat(ind))),...
                                                           num2str8(beam_model.PBar.A(ind)), num2str8(beam_model.PBar.I(ind,1)),...
                                                           num2str8(beam_model.PBar.I(ind,2)), num2str8(beam_model.PBar.J(ind)),...
                                                           num2str8(beam_model.PBar.RhoNS(ind))]);
                  else
                    fprintf(NA,'\n$ Half values used');
                     fprintf(NA,['\nPBAR    ',riga(9:16),cnvt2_8chs(beam_model.Mat.ID(beam_model.PBar.Mat(ind))),...
                                                           cnvt2_8chs_real(beam_model.PBar.A(ind)/2), cnvt2_8chs_real(beam_model.PBar.I(ind,1)/2),...
                                                           cnvt2_8chs_real(beam_model.PBar.I(ind,2)/2), cnvt2_8chs_real(beam_model.PBar.J(ind)/2),...
                                                           cnvt2_8chs_real(beam_model.PBar.RhoNS(ind)/2)]);
                  end
                   fprintf(NA,['\n        ', num2str8(beam_model.PBar.Str_point(1,1,ind)), num2str8(beam_model.PBar.Str_point(1,2,ind)),...
                                                            num2str8(beam_model.PBar.Str_point(2,1,ind)), num2str8(beam_model.PBar.Str_point(2,2,ind)),... 
                                                            num2str8(beam_model.PBar.Str_point(3,1,ind)), num2str8(beam_model.PBar.Str_point(3,2,ind)),...
                                                            num2str8(beam_model.PBar.Str_point(4,1,ind)), num2str8(beam_model.PBar.Str_point(4,2,ind))]); 
                   fprintf(NA,['\n        ', num2str8(beam_model.PBar.Kshear(ind,1)), num2str8(beam_model.PBar.Kshear(ind,1)), num2str8(beam_model.PBar.I(ind,3))]);                                     
                   fgetl(SM);
                   riga = fgetl(SM);
            case 'SET1    '
                % IS needed to delete grid out of aerodynamic plane (odd grids)
                [riga, rem] = strtok(riga);
                [riga, rem] = strtok(rem);
                IDs = str2num(riga);             
                GrSet=[]; cont=1;
                while (~isempty(riga))
                  [riga, rem] = strtok(rem);            
                  if isempty(riga)
                    break;
                  end
                  GrSet(cont) = str2num(riga);
                  cont = cont+1;
                end
                riga = fgetl(SM);
                while length(riga)>8 && strcmp(riga(1:8), '        ')
                  rem = riga;
                  while (~isempty(riga))
                    [riga, rem] = strtok(rem);            
                    if isempty(riga)
                      break;
                    end
                    GrSet(cont) = str2num(riga);
                    cont = cont+1;
                  end
                  riga = fgetl(SM);
                end
                v = sprintf('%-8d', IDs);
                fprintf(NA,'\nSET1    ');
                fprintf(NA,'%s',v);
                cont = 2;
                for i=1:length(GrSet)
                  cont=cont+1;
                  v = sprintf('%-8d', GrSet(i));
                  fprintf(NA,'%s',v);
                  if (cont==9)
                    fprintf(NA,'\n        '); cont=1;
                  end
                end                 
            case 'AELINK  '
                fprintf(NA,['\n',riga(1:8),'1       ',riga(17:32),num2str8(-str2double(riga(33:end)))]);
                riga = fgetl(SM);
            case 'CONM2   '
                IDconm = str2double(riga(9:16));
                ind = find(conm == IDconm);
                if isempty(ind)
                  fprintf(NA,['\n',riga(1:32),num2str8(str2double(riga(33:40))),num2str8(str2double(riga(41:48))),num2str8(str2double(riga(49:56))),num2str8(str2double(riga(57:64)))]);
                  riga = fgetl(SM);
                  fprintf(NA,['\n',riga(1:8),num2str8(str2double(riga(9:16))),num2str8(str2double(riga(17:24))),num2str8(str2double(riga(25:32))),num2str8(str2double(riga(33:40))),num2str8(str2double(riga(41:48))),num2str8(str2double(riga(49:56)))]);
                  riga = fgetl(SM);
                else
                  fprintf(NA,'\n$ Half values used');
                  fprintf(NA,['\n',riga(1:32),num2str8(str2double(riga(33:40))/2),num2str8(str2double(riga(41:48))),num2str8(str2double(riga(49:56))),num2str8(str2double(riga(57:64)))]);
                  riga = fgetl(SM);
                  fprintf(NA,['\n',riga(1:8),cnvt2_8chs_real(str2double(riga(9:16))/2),cnvt2_8chs_real(str2double(riga(17:24))/2),cnvt2_8chs_real(str2double(riga(25:32))/2),cnvt2_8chs_real(str2double(riga(33:40))/2),cnvt2_8chs_real(str2double(riga(41:48))/2),cnvt2_8chs_real(str2double(riga(49:56))/2)]);
                  riga = fgetl(SM);
                end
            case 'CAERO1  '
                if isempty(indpaero)
                     fprintf(NA,'\nPAERO1  10000');
                     indpaero = 1;
                end
                % if CAERO1 (NeoCASS) define a control surface too, a new CAERO1 (Nastran)  will be defined
                % and also an AELIST, a COORD and an AESURF needed to
                % completely defined a surface control.
                ID = str2double(riga(9:16));
                DH = str2double(riga(17:24))*pi/180;
                NSPAN = riga(33:40);
                NCHOR = riga(41:48);
                MTYPE = str2double(riga(65:72));
                if isempty(IDcaero2)
                    IDcaero2 = [ID,ID,str2double(NSPAN)*str2double(NCHOR)-1]; 
                    IDC = ID+str2double(NSPAN)*str2double(NCHOR);
                else
                    IDcaero2 = [IDcaero2;ID,IDC,str2double(NSPAN)*str2double(NCHOR)-1];
                    IDC = IDC+str2double(NSPAN)*str2double(NCHOR);
                end
                riga = fgetl(SM);
                POS = [str2double(riga(9:16)),str2double(riga(17:24)),str2double(riga(25:32))];
                c = str2double(riga(33:40));
                b = str2double(riga(41:48));
                TAP = str2double(riga(49:56));
                SWQC = str2double(riga(57:64))*pi/180;
                POS2(1) = POS(1) + 0.25*c+tan(SWQC)*b - 0.25*c*TAP;
                if DH == 90*pi/180
                    POS2(2) = POS(2);
                    POS2(3) = POS(3) + b;
                else
                    POS2(2) = POS(2) + b;
                    POS2(3) = POS(3) + tan(DH)*b;
                end
                riga = fgetl(SM);
                if length(riga)>8 && strcmp(riga(1:8),'        ')
                    FRC1 = 1-str2double(riga(17:24));
                    FRC2 = 1-str2double(riga(25:32));
                    NCHORcs = riga(33:40);
                    label = riga(41:48);
                    fprintf(NA,['\nCAERO1  ',cnvt2_8chs(IDcaero2(end,2)),'10000   ','0       ',NSPAN,NCHOR,'                       1']);
                    fprintf(NA,['\n        ',num2str8(POS(1)),num2str8(POS(2)),num2str8(POS(3)),...
                                             num2str8(c*FRC1),num2str8(POS2(1)),num2str8(POS2(2)),num2str8(POS2(3)), num2str8(c*TAP*FRC2)]);
                    
                    POS(1) = POS(1) + c*FRC1;
                    POS2(1) = POS2(1) + c*TAP*FRC2;
                    NY = str2double(NSPAN);
                    NX = str2double(NCHORcs);
                    IDcaero = [IDcaero;ID,IDcaero2(end,2)+10000,NX*NY];
                    fprintf(NA,['\nCAERO1  ',cnvt2_8chs(IDcaero2(end,2)+10000),'10000   ','0       ',NSPAN,NCHORcs,'                       1']);
                    fprintf(NA,['\n        ',num2str8(POS(1)),num2str8(POS(2)),num2str8(POS(3)),...
                                             num2str8(c*(1-FRC1)),num2str8(POS2(1)),num2str8(POS2(2)),num2str8(POS2(3)), num2str8(c*TAP*(1-FRC2))]);
                                       
                    % Define AELIST
                    if MTYPE == 6
                        fprintf(NA,['\nAELIST  ',cnvt2_8chs(IDcaero2(end,2)+10000), cnvt2_8chs(IDcaero2(end,2)+10000),'    THRU',cnvt2_8chs(IDcaero2(end,2)+10000+NX*NY-1),...
                                                cnvt2_8chs(IDcaero2(end,2)),'    THRU',cnvt2_8chs(IDcaero2(end,2)+str2double(NCHOR)*NY-1) ]);
                    else
                        fprintf(NA,['\nAELIST  ',cnvt2_8chs(IDcaero2(end,2)+10000), cnvt2_8chs(IDcaero2(end,2)+10000),'    THRU',cnvt2_8chs(IDcaero2(end,2)+10000+NX*NY-1)]);
                    
                    end
                    % Define Reference sistem
                    IDcoord = IDcoord+1;
                    Y = POS2-POS;
                    A = POS;
%                     [dummy,vers] = max(abs(Y));
%                     if vers == 1 || vers == 2
%                         C = cross(Y,[0,0,1]);
%                         B = cross(C,Y);
%                     else % vtail
%                         C = cross(Y,[0,1,0]);
%                         B = cross(C,Y);
%                     end
                    B = cross(Y,[1,0,0])/norm(cross(Y,[1,0,0]));
                    C = cross(Y,B)/norm(cross(Y,B));
                    fprintf(NA,['\nCORD2R  ',cnvt2_8chs(IDcoord),'       0',num2str8(A(1)),num2str8(A(2)),num2str8(A(3)),...
                                num2str8(A(1)+B(1)),num2str8(A(2)+B(2)),num2str8(A(3)+B(3))]);
                    fprintf(NA,['\n        ',num2str8(A(1)+C(1)),num2str8(A(2)+C(2)),num2str8(A(3)+C(3))]);
                    % Define AESURF
                    fprintf(NA,['\nAESURF  ',cnvt2_8chs(IDsurf),label,cnvt2_8chs(IDcoord),cnvt2_8chs(IDcaero2(end,2)+10000)]);
                    IDsurf = IDsurf+1;
                    riga = fgetl(SM);
                else
                    fprintf(NA,['\nCAERO1  ',cnvt2_8chs(IDcaero2(end,2)),'10000   ','0       ',NSPAN,NCHOR,'                       1']);
                    fprintf(NA,['\n        ',num2str8(POS(1)),num2str8(POS(2)),num2str8(POS(3)),...
                                num2str8(c),num2str8(POS2(1)),num2str8(POS2(2)),num2str8(POS2(3)), num2str8(c*TAP)]);
                    
                end
            case 'SPLINE1 '
                ID = str2double(riga(9:16));
                IDc = str2double(riga(17:24));
                IDset = str2double(riga(41:48));
                ind2 = find(IDcaero2(:,1)==IDc);
                fprintf(NA,['\nSPLINE2 ',cnvt2_8chs(ID+20000),cnvt2_8chs(IDcaero2(ind2,2)),cnvt2_8chs(IDcaero2(ind2,2)),cnvt2_8chs(IDcaero2(ind2,2)+IDcaero2(ind2,3)),cnvt2_8chs(IDset),'                ',cnvt2_8chs(IDset),'\n        1.0     1.0']);
                if ~isempty(IDcaero) && ~isempty(find(IDcaero(:,1) == IDc,1))
                    ind = find(IDcaero(:,1) == IDc);
                    fprintf(NA,['\nSPLINE2 ',cnvt2_8chs(IDcaero(ind,2)+20000),cnvt2_8chs(IDcaero(ind,2)),cnvt2_8chs(IDcaero(ind,2)),cnvt2_8chs(IDcaero(ind,2)+IDcaero(ind,3)-1),cnvt2_8chs(IDset),'                ',cnvt2_8chs(IDset),'\n        1.0     1.0']);
                end
                riga = fgetl(SM);

            case 'SPLINE2 '
                ID = str2double(riga(9:16));
                IDc = str2double(riga(17:24));
%                 Ntot = str2double(riga(33:40));
                IDset = str2double(riga(41:48));
                ind2 = find(IDcaero2(:,1)==IDc);
                fprintf(NA,['\nSPLINE1 ',cnvt2_8chs(ID+20000),cnvt2_8chs(IDcaero2(ind2,2)),cnvt2_8chs(IDcaero2(ind2,2)),cnvt2_8chs(IDcaero2(ind2,2)+IDcaero2(ind2,3)),cnvt2_8chs(IDset),'        TPS']);
                if ~isempty(IDcaero) && ~isempty(find(IDcaero(:,1) == IDc,1))
                    ind = find(IDcaero(:,1) == IDc);
                    fprintf(NA,['\nSPLINE1 ',cnvt2_8chs(IDcaero(ind,2)+20000),cnvt2_8chs(IDcaero(ind,2)),cnvt2_8chs(IDcaero(ind,2)),cnvt2_8chs(IDcaero(ind,2)+IDcaero(ind,3)-1),cnvt2_8chs(IDset),'        TPS']);
                end
                riga = fgetl(SM);
            case 'TRIM    '
                fprintf(NA,'\nAESTAT  501     ANGLEA');
                fprintf(NA,'\nAESTAT  502     SIDES');
                fprintf(NA,'\nAESTAT  503     PITCH');
                fprintf(NA,'\nAESTAT  504     ROLL');
                fprintf(NA,'\nAESTAT  505     YAW');
                fprintf(NA,'\nAESTAT  506     URDD1');
                fprintf(NA,'\nAESTAT  507     URDD2');
                fprintf(NA,'\nAESTAT  508     URDD3');
                fprintf(NA,'\nAESTAT  509     URDD4');
                fprintf(NA,'\nAESTAT  510     URDD5');
                fprintf(NA,'\nAESTAT  511     URDD6');
                fprintf(NA,'\nAESTAT  512     HEAD');
                fprintf(NA,'\nAESTAT  513     CLIMB');
                fprintf(NA,'\nAESTAT  514     BANK');
                fprintf(NA,'\nAESTAT  515     THRUST');
                fprintf(NA,['\n',riga]);
                riga = fgetl(SM);
            case 'AEROS   '
                 fprintf(NA,['\n',riga(1:8),'100     ','100     ',num2str8(str2double(riga(25:32))),num2str8(str2double(riga(33:40))),num2str8(str2double(riga(41:48))),riga(49:end)]);
                 riga = fgetl(SM);
            case 'SUPORT  '
%                 if exist(beam_model,'var')
%                 else
%                 end
                 fprintf(NA,['\n',riga(1:8),riga(17:end)]);
                 riga = fgetl(SM);
%             case 'CELAS'
%                 fprintf(NA,['\n',[riga(1:5),'1  '],riga(9:16),num2str8(str2double(riga(17:24))),riga(25:end)]);
            otherwise
                fprintf(NA,['\n',riga]);
                riga = fgetl(SM);
        end
    end
end
fclose(NA);
fclose(SM);
end
