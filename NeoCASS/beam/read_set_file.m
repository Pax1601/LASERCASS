%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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

%
%**************************************************************************
%*****
%  SimSAC Project
%
%  NeoCASS
%  Next generation Conceptual Aero Structural Sizing
%
%                      Sergio Ricci            <ricci@aero.polimi.it>
%                      Luca Cavagna            <cavagna@aero.polimi.it>
%                      Luca Riccobene          <riccobene@aero.polimi.it>
%                      Alessandro De Gaspari   <degaspari@aero.polimi.it>
%
%  Department of Aerospace Engineering - Politecnico di Milano (DIAPM)
%  Warning: This code is released only to be used by SimSAC partners.
%  Any usage without an explicit authorization may be persecuted.
%
%**************************************************************************
%
% Input: 
% cline: line to process
% card: card which called the function
%
function data = read_set_file(cline, card)
%
data = [];
indTHRU = strfind(cline,'THRU');
if isempty(indTHRU)
  data = str2num(cline);
else
  indEXCEPT = strfind(cline,'EXCEPT');
  if isempty(indEXCEPT)
    data = str2num(cline(1:indTHRU(1)-1));
    for iTH = 1 : length(indTHRU)-1
      dataTHRU = str2num(cline(indTHRU(iTH)+4:indTHRU(iTH+1)-1));
      data = [data,data(end)+1:dataTHRU(1),dataTHRU(2:end)];
    end
    dataTHRU = str2num(cline(indTHRU(end)+4:end));
    data = unique([data,data(end)+1:dataTHRU(1),dataTHRU(2:end)]);
  else
    if length(indEXCEPT) >  length(indTHRU)
      error(['Too many EXCEPT found in ', card,'.']);
    else
      data = str2num(cline(1:indTHRU(1)-1));
      for iTH = 1 : length(indTHRU)-1
        if isempty(find(indEXCEPT>indTHRU(iTH) & indEXCEPT<indTHRU(iTH+1),1))
          dataTHRU = str2num(cline(indTHRU(iTH)+4:indTHRU(iTH+1)-1));
          data = [data,data(end)+1:dataTHRU(1),dataTHRU(2:end)];
        else
          indEXCEPTloc = indEXCEPT(indEXCEPT>indTHRU(iTH) & indEXCEPT<indTHRU(iTH+1));
          dataEXCEPT = str2num(cline(indEXCEPTloc+6:indTHRU(iTH+1)-1));
          dataTHRU = str2double(cline(indTHRU(iTH)+4:indEXCEPTloc-1));
          data = [data,setdiff(data(end)+1:dataTHRU(1),dataEXCEPT(dataEXCEPT<=dataTHRU(1))),dataEXCEPT(dataEXCEPT>dataTHRU(1))];
        end
      end
      if isempty(find(indEXCEPT>indTHRU(end),1))
        dataTHRU = str2num(cline(indTHRU(end)+4:end));
        data = unique([data,data(end)+1:dataTHRU(1),dataTHRU(2:end)]);
      else
        indEXCEPTloc = indEXCEPT(indEXCEPT>indTHRU(end) );
        dataEXCEPT = str2num(cline(indEXCEPTloc+6:end));
        dataTHRU = str2double(cline(indTHRU(end)+4:indEXCEPTloc-1));
        data = unique([data,setdiff(data(end)+1:dataTHRU(1),dataEXCEPT(dataEXCEPT<=dataTHRU(1))),dataEXCEPT(dataEXCEPT>dataTHRU(1))]);
      end
    end
  end
end
if (isempty(data))
  error(['Failed in processing card ', card,'.']);
end
end