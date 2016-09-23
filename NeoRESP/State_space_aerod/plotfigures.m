function plotfigures(p,Ha,Happrox,labeltitle,labelleg,nfig)
% =========================================================================
%                                                               plotfigures
% =========================================================================
%
% Description: auxiliary function for figures plotting 
%
% -------------------------------------------------------------------------
%
%
%   Copyright (C) 2012 Paolo Mantegazza   <mantegazza@aero.polimi.it>
%   Copyright (C) 2012 Matteo Ripepi      <ripepi@aero.polimi.it>
%  
%   This program is free software; you can redistribute it and/or
%   modify it under the terms of the GNU General Public License as
%   published by the Free Software Foundation; either version 3 of the
%   License, or (at your option) any later version.
%  
%   This program is distributed in the hope that it will be useful,
%   but WITHOUT ANY WARRANTY; without even the implied warranty of
%   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%   GNU General Public License for more details.
%  
%   You should have received a copy of the GNU General Public License
%   along with this program; if not, write to the Free Software
%   Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
%  
%   Ref: Ripepi M., Mantegazza P., 'An improved matrix fraction approximation of
%        aerodynamic transfer matrices', AIAA journal, submitted for publication.
%
%
% =========================================================================

% ask for plotting figures
flagplot = input('>> Do you want to plot the results? (y/n): ','s');
flagplot = any( strcmp(flagplot,{'y','Y','yes','YES'}) );

while flagplot
    
    if (nargin == 5)
        nfig = 1;
    end
    
    % dimensions
    nrows = size(Ha,1);
    ncols = size(Ha,2);
    
    % ask for matrix components to plot
    ask = 1;
    while ask
        row = input('>> Input a vector of the matrix rows to plot: ');
        if (max(row) > nrows)
            fprintf(1, ' - Warning: required row index exceeds matrix size.\n\n');
        else
            ask = 0;
        end
    end
    
    ask = 1;
    while ask
        col = input('>> Input a vector of the matrix column to plot: ');
        if (max(col) > ncols)
            fprintf(1, ' - Warning: required column index exceeds matrix size.\n\n');
        else
            ask = 0;
        end
    end
    
    nrowplot = length(row);
    ncolplot = length(col);
    
    % Plot results
    % ---------------------------------------------------------------------
    % real part
    H1 = figure(nfig); clf
    set(H1, 'Visible','on', 'Name', strcat(labeltitle, ' - real part'), 'NumberTitle','off', 'MenuBar','figure');
    hold on, box on
    n = 1;
    for i = 1:nrowplot
        for j = 1:ncolplot
            subplot(nrowplot,ncolplot,n),
            data1 = squeeze(Ha(row(i),col(j),:));
            data2 = squeeze(Happrox(row(i),col(j),:));
            plot( abs(p), real(data1), abs(p), real(data2), ...
                '--rs','MarkerSize',5 , 'MarkerFaceColor','r' , 'LineWidth', 1 );
            xlabel('k','fontsize',12)
            labelaxis = [ 'Re H_{', num2str(row(i)), num2str(col(j)),'}' ];
            ylabel(labelaxis,'fontsize',12)
            n = n+1;
        end
    end
    hleg = legend('GAF data',labelleg,'Location','Best');
    set(hleg,'fontsize',11,'EdgeColor', [1 1 1])
    
    % imaginary part
    H2 = figure(nfig+1); clf
    set(H2, 'Visible','on', 'Name', strcat(labeltitle, ' - imag part'), 'NumberTitle','off', 'MenuBar','figure');
    hold on, box on
    n = 1;
    for i = 1:nrowplot
        for j = 1:ncolplot
            subplot(nrowplot,ncolplot,n),
            data1 = squeeze(Ha(row(i),col(j),:));
            data2 = squeeze(Happrox(row(i),col(j),:));
            plot( abs(p), imag(data1), abs(p), imag(data2), ...
                '--rs','MarkerSize',5 , 'MarkerFaceColor','r' , 'LineWidth', 1 );
            xlabel('k','fontsize',12)
            labelaxis = [ 'Im H_{', num2str(row(i)), num2str(col(j)),'}' ];
            ylabel(labelaxis,'fontsize',12)
            n = n+1;
        end
    end
    hleg = legend('GAF data',labelleg,'Location','Best');
    set(hleg,'fontsize',11,'EdgeColor', [1 1 1])
    
    % complex domain
    H3 = figure(nfig+2); clf
    set(H3, 'Visible','on', 'Name', strcat(labeltitle, ' - complex domain'), 'NumberTitle','off', 'MenuBar','figure');
    hold on, box on
    n = 1;
    for i = 1:nrowplot
        for j = 1:ncolplot
            subplot(nrowplot,ncolplot,n),
            data1 = squeeze(Ha(row(i),col(j),:));
            data2 = squeeze(Happrox(row(i),col(j),:));
            plot( real(data1), imag(data1), real(data2), imag(data2), ...
                '--rs','MarkerSize',5 , 'MarkerFaceColor','r' , 'LineWidth', 1 );
            labelaxis1 = [ 'Re H_{', num2str(row(i)), num2str(col(j)),'}' ];
            labelaxis2 = [ 'Im H_{', num2str(row(i)), num2str(col(j)),'}' ];
            xlabel(labelaxis1,'fontsize',12)
            ylabel(labelaxis2,'fontsize',12)
            n = n+1;
        end
    end
    hleg = legend('GAF data',labelleg,'Location','Best');
    set(hleg,'fontsize',11,'EdgeColor', [1 1 1])
    
    % save plot
    % -------------------------------------------------------------------------
    flag_saveplot = input('\n>> Do you want to save the figure as an images? (y/n): ','s');
    switch flag_saveplot
        case{'y','yes','Y','YES','1'}
            
            plotfilename = input('>> Enter output file name with the related extension\n    [eps (default), ps, jpeg, tiff, png]: ','s');
            
            % extract extension
            i_ext = strfind(plotfilename,'.');
            if isempty(i_ext), ext = 'epsc2';
            else ext = plotfilename(i_ext(end) + 1 : end);
            end
            
            print( H1, strcat('-d', ext ), '-r600', strcat(plotfilename,'_Re_',date) )
            print( H2, strcat('-d', ext ), '-r600', strcat(plotfilename,'_Im_',date) )
            print( H3, strcat('-d', ext ), '-r600', strcat(plotfilename,'_Cx_',date) )
    end
    
    flagplot = input('\n>> Do you want others plots? (y/n): ','s');
    flagplot = any( strcmp(flagplot,{'y','Y','yes','YES'}) );
    
end


