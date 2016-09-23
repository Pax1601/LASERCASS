%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (C) 2008 - 2011 
% 
% Sergio Ricci (sergio.ricci@polimi.it)
%
% Politecnico di Milano, Dipartimento di Ingegneria Aerospaziale
% Via La Masa 34, 20156 Milano - ITALY
% 
% This file is part of NeoCASS Software (www.neocass.org)
%
% NeoCASS is free software; you can redistribute it and/or
% modify it under the terms of the GNU General Public
% License as published by the Free Software Foundation;
% either version 2, or (at your option) any later version.
%
% NeoCASS is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied
% warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
% PURPOSE.  See the GNU General Public License for more
% details.
%
% You should have received a copy of the GNU General Public
% License along with NeoCASS; see the file GNU GENERAL 
% PUBLIC LICENSE.TXT.  If not, write to the Free Software 
% Foundation, 59 Temple Place -Suite 330, Boston, MA
% 02111-1307, USA.
%

function [data]=readdatcom(state)

global MACH ALPHA NMACH NALPHA 
% This script was written by Dr. Mehdi Ghoreyshi

data.alt=state.altmin;
data.nmach=NMACH;
data.mach=MACH;
data.nalpha=NALPHA;
data.alpha=ALPHA;


k=1;
count=1.0;
num=0;
i=1;
exist=0;
j=1;
number=1;
fid = fopen('for006.dat', 'rt');
while feof(fid) == 0
    tline = fgetl(fid);
    matches = findstr(tline, 'WING-BODY-VERTICAL');
    num = length(matches);
    

    if num>0
        i=1;
        exist=1;
        count=1;
        k=1;
        number=number+1;
    end;

    if exist==1
        count=count+1;
    end;

    

        switch number
            case 2
               if (count>=13)
                
                results= strread(tline,'%s');
                k=k+1;
                if (k<NALPHA+2)
                    data.cl(i,j,1)=str2num(results{3});
                    data.cd(i,j,1)=str2num(results{2});
                    data.cm(i,j,1)=str2num(results{4});
                    data.cla(i,j,1)=str2num(results{8});
                    data.cma(i,j,1)=str2num(results{9});

                    if i==1
                        data.cyb(i,j,1)=str2num(results{10});
                        data.cnb(i,j,1)=str2num(results{11});
                        data.clb(i,j,1)=str2num(results{12});

                    else
                        data.cyb(i,j,1)=data.cyb(1,j,1);
                        data.cnb(i,j,1)=data.cnb(1,j,1);
                        data.clb(i,j,1)=str2num(results{10});

                    end;
                    i=i+1;
                end;
               end;

            case 3
                if count>=14
                results= strread(tline,'%s');
                k=k+1;
                if (k<NALPHA+2)

                    if i==1
                        data.clq(i,j,1)=str2num(results{2});
                        data.cmq(i,j,1)=str2num(results{3});
                        data.clad(i,j,1)=str2num(results{4});
                        data.cmad(i,j,1)=str2num(results{5});
                        data.clp(i,j,1)=str2num(results{6});
                        data.cyp(i,j,1)=str2num(results{7});
                        data.cnp(i,j,1)=str2num(results{8});
                        data.cnr(i,j,1)=str2num(results{9});
                        data.clr(i,j,1)=str2num(results{10});
                    else
                        data.clq(i,j,1)=data.clq(1,j,1);
                        data.cmq(i,j,1)=data.cmq(1,j,1);
                        data.clad(i,j,1)=str2num(results{2});
                        data.cmad(i,j,1)=str2num(results{3});
                        data.clp(i,j,1)=str2num(results{4});
                        data.cyp(i,j,1)=str2num(results{5});
                        data.cnp(i,j,1)=str2num(results{6});
                        data.cnr(i,j,1)=str2num(results{7});
                        data.clr(i,j,1)=str2num(results{8});
                    end
                    i=i+1;
                end;
                if k==NALPHA+2
                    number=1;
                    j=j+1;
                end;
                end;
        end;
   
end
fclose('all');

