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

function Report_Stab_derNa(file_name,fileNA, varargin)

% Rstab_der,Dstab_der

% this function write a LaTeX file reporting results in therms of stabylity
% derivatives on file_name
[CC,M,Ix,label]= Derivate_stabilita(fileNA);

if nargin == 3 % only rigid Stability derivatives
    SD = varargin{1};
    fid = fopen([file_name,'.tex'],'w');
    fprintf(fid,'%s','%Report written by NeoCASS');
    fprintf(fid,'\n%s','\documentclass[openany]{book}%[twocolumn]');
    fprintf(fid,'\n%s','\usepackage[ansinew]{inputenc}');
    fprintf(fid,'\n%s','\usepackage{amsmath}%');
    fprintf(fid,'\n%s','\usepackage{amsfonts}%');
    fprintf(fid,'\n%s','\usepackage{fontenc}');
    fprintf(fid,'\n%s','\usepackage{amssymb}%');
    fprintf(fid,'\n%s','\usepackage{graphicx}');
    fprintf(fid,'\n%s','\usepackage{color}');
    fprintf(fid,'\n%s','\usepackage{epsfig}');
    fprintf(fid,'\n%s','\usepackage{graphics}%[dvips]');
    fprintf(fid,'\n%s','\usepackage{subfigure}%');
    fprintf(fid,'\n%s','\usepackage{fancyhdr}');
    fprintf(fid,'\n%s','\usepackage[Glenn]{fncychap}');
    fprintf(fid,'\n%s','\usepackage{eso-pic}');
    fprintf(fid,'\n%s','\usepackage{makeidx}');
    fprintf(fid,'\n%s','\usepackage[english,refpage,intoc]{nomencl}');
    fprintf(fid,'\n%s','\usepackage{pifont}');
    fprintf(fid,'\n%s','\usepackage{float}');
    fprintf(fid,'\n%s','\usepackage{wrapfig}');
%     fprintf(fid,'\n%s','\usepackage[vflt]{floatflt}');
    fprintf(fid,'\n%s','\usepackage{fancybox}');
    fprintf(fid,'\n%s','\usepackage{booktabs}');
    fprintf(fid,'\n%s','\usepackage{lipsum}');
    fprintf(fid,'\n%s','%\usepackage{classicthesis-ldpkg}');
    fprintf(fid,'\n%s','\usepackage[dvipdfmx]{hyperref}');
    fprintf(fid,'\n%s','\hypersetup{%');
    fprintf(fid,'\n%s','colorlinks=true, linktocpage=true, pdfstartpage=1, pdfstartview=FitV,%');
    fprintf(fid,'\n%s','breaklinks=true, pdfpagemode=UseNone, pageanchor=true, pdfpagemode=UseOutlines,%');
    fprintf(fid,'\n%s','plainpages=false, bookmarksnumbered, bookmarksopen=true, bookmarksopenlevel=1,%');
    fprintf(fid,'\n%s','hypertexnames=true, pdfhighlight=/O,%hyperfootnotes=true,%nesting=true,%frenchlinks,%');
    fprintf(fid,'\n%s','urlcolor=webbrown, linkcolor=blue, citecolor=webgreen %pagecolor=RoyalBlue,%');
    fprintf(fid,'\n%s','% uncomment the following line if you want to have black links (e.g., for printing)');
    fprintf(fid,'\n%s','%urlcolor=Black, linkcolor=Black, citecolor=Black, %pagecolor=Black,%');
    fprintf(fid,'\n%s','%pdftitle={\myTitle},%');
    fprintf(fid,'\n%s','%pdfauthor={\textcopyright\ \myName, \myUni, \myFaculty},%');
    fprintf(fid,'\n%s','%pdfsubject={},%');
    fprintf(fid,'\n%s','%pdfkeywords={},%');
    fprintf(fid,'\n%s','%pdfcreator={dvipdfmx},%');
    fprintf(fid,'\n%s','%pdfproducer={LaTeX with hyperref and classicthesis}%');
    fprintf(fid,'\n%s','}');
    fprintf(fid,'\n%s','%----------------------------------------------------------');
    fprintf(fid,'\n%s','\newtheorem{theorem}{Theorem}');
    fprintf(fid,'\n%s','\newtheorem{acknowledgement}[theorem]{Acknowledgement}');
    fprintf(fid,'\n%s','\newtheorem{algorithm}[theorem]{Algorithm}');
    fprintf(fid,'\n%s','\newtheorem{axiom}[theorem]{Axiom}');
    fprintf(fid,'\n%s','\newtheorem{case}[theorem]{Case}');
    fprintf(fid,'\n%s','\newtheorem{claim}[theorem]{Claim}');
    fprintf(fid,'\n%s','\newtheorem{conclusion}[theorem]{Conclusion}');
    fprintf(fid,'\n%s','\newtheorem{condition}[theorem]{Condition}');
    fprintf(fid,'\n%s','\newtheorem{conjecture}[theorem]{Conjecture}');
    fprintf(fid,'\n%s','\newtheorem{corollary}[theorem]{Corollary}');
    fprintf(fid,'\n%s','\newtheorem{criterion}[theorem]{Criterion}');
    fprintf(fid,'\n%s','\newtheorem{definition}[theorem]{Definition}');
    fprintf(fid,'\n%s','\newtheorem{example}[theorem]{Example}');
    fprintf(fid,'\n%s','\newtheorem{exercise}[theorem]{Exercise}');
    fprintf(fid,'\n%s','\newtheorem{lemma}[theorem]{Lemma}');
    fprintf(fid,'\n%s','\newtheorem{notation}[theorem]{Notation}');
    fprintf(fid,'\n%s','\newtheorem{problem}[theorem]{Problem}');
    fprintf(fid,'\n%s','\newtheorem{proposition}[theorem]{Proposition}');
    fprintf(fid,'\n%s','\newtheorem{remark}[theorem]{Remark}');
    fprintf(fid,'\n%s','\newtheorem{solution}[theorem]{Solution}');
    fprintf(fid,'\n%s','\newtheorem{summary}[theorem]{Summary}');
    fprintf(fid,'\n%s','\newenvironment{proof}[1][Proof]{\textbf{#1.} }{\ \rule{0.9em}{0.9em}}');
    fprintf(fid,'\n%s','\setlength{\topmargin}{-0.3in}');
    fprintf(fid,'\n%s','\setlength{\topskip}{0.2in}    % between header and text');
    fprintf(fid,'\n%s','\setlength{\textheight}{9.5in} % height of main text');
    fprintf(fid,'\n%s','\setlength{\textwidth}{6in}    % width of text');
    fprintf(fid,'\n%s','\setlength{\oddsidemargin}{0in} % odd page left margin');
    fprintf(fid,'\n%s','\setlength{\evensidemargin}{0.2in}');
    fprintf(fid,'\n%s','\pagestyle{fancy}');
    fprintf(fid,'\n%s','\newcommand{\fncyfront}{%');
    fprintf(fid,'\n%s','\fancyhead[RO]{\footnotesize\rightmark}');
    fprintf(fid,'\n%s','\fancyfoot[RO]{\footnotesize\textcolor[gray]{0.5}{\textbf{SimSAC}}}');
    fprintf(fid,'\n%s','\fancyhead[LE]{\footnotesize{\leftmark }}');
    fprintf(fid,'\n%s','\fancyfoot[LE]{\footnotesize\textcolor[gray]{0.5}{\textbf{NeoCASS}}}');
    fprintf(fid,'\n%s','\fancyhead[LO,RE]{{\textcolor[gray]{0.35}{\footnotesize \textbf{Stability Derivatives}}}}');
    fprintf(fid,'\n%s','\fancyfoot[C]{\thepage}');
    fprintf(fid,'\n%s','\renewcommand{\headrulewidth}{1.3 pt}');
    fprintf(fid,'\n%s','\renewcommand{\footrulewidth}{1.3 pt}}');
    fprintf(fid,'\n%s','\newcommand {\fncymain }{%');
    fprintf(fid,'\n%s','\fancyhead [RO ]{{\scriptsize \MakeUppercase');
    fprintf(fid,'\n%s','    \rightmark }}');
    fprintf(fid,'\n%s','\fancyfoot [RO ]{\thepage }');
    fprintf(fid,'\n%s','\fancyhead [LE ]{{\scriptsize \MakeUppercase');
    fprintf(fid,'\n%s','    \leftmark }}');
    fprintf(fid,'\n%s','\fancyfoot [LE ]{\thepage }');
    fprintf(fid,'\n%s','\fancyfoot [C]{}');
    fprintf(fid,'\n%s','\renewcommand {\headrulewidth }{0.3 pt }} ');
    fprintf(fid,'\n%s','\newcommand\AlCentroPagina[1]{ %');
    fprintf(fid,'\n%s','    \AddToShipoutPicture *{\AtPageCenter {%');
    fprintf(fid,'\n%s','    \makebox (0,0){\includegraphics %');
    fprintf(fid,'\n%s','[width =0.5\paperwidth]{#1}}}}}');
    fprintf(fid,'\n%s','\makeatletter');
    fprintf(fid,'\n%s','\ChNameVar{\huge\rm} % sets the style for name');
    fprintf(fid,'\n%s','\ChNumVar{\Huge\rm\centering} % sets the style for digit');
    fprintf(fid,'\n%s','\ChTitleVar{\Large\rm\centering} % sets the style for title');
    fprintf(fid,'\n%s','\ChRuleWidth{2pt} % Set RW=4pt');
    fprintf(fid,'\n%s','\ChNameUpperCase % Make name uppercase');
    fprintf(fid,'\n%s','\renewcommand{\DOCH}{%');
    fprintf(fid,'\n%s','\setlength{\fboxrule}{\RW} % Let fbox lines be controlled by');
    fprintf(fid,'\n%s','% \ChRuleWidth');
    fprintf(fid,'\n%s','\centering\fbox{ \textcolor[gray]{0.35}{\CNV\FmN{\@chapapp}}\space \textcolor[gray]{0.35}{\CNoV\thechapter}}\par\nobreak');
    fprintf(fid,'\n%s','\vskip 10\p@}');
    fprintf(fid,'\n%s','\renewcommand{\DOTI}[1]{%');
    fprintf(fid,'\n%s','\CTV\FmTi{#1}\par\nobreak');
    fprintf(fid,'\n%s','\vskip 50\p@}');
    fprintf(fid,'\n%s','\renewcommand{\DOTIS}[1]{%');
    fprintf(fid,'\n%s','\CTV\FmTi{#1}\par\nobreak');
    fprintf(fid,'\n%s','\vskip 20\p@}');
    fprintf(fid,'\n%s','\makeatother');
    fprintf(fid,'\n%s','\def\cleardoublepage{\clearpage\if@twoside');
    fprintf(fid,'\n%s','\ifodd\c@page');
    fprintf(fid,'\n%s','\else\hbox{}\thispagestyle{empty}\newpage');
    fprintf(fid,'\n%s','\if@twocolumn\hbox{}\newpage\fi\fi\fi}     ');
    fprintf(fid,'\n%s','\makeindex');
    fprintf(fid,'\n%s','\makenomenclature');
    fprintf(fid,'\n%s','\floatstyle{ruled}');
    fprintf(fid,'\n%s','\newfloat{appendix}{ht}{pro}[chapter]');
    fprintf(fid,'\n%s','\floatname{appendix}{Appendix}');
    fprintf(fid,'\n%s','\floatstyle{ruled}');
    fprintf(fid,'\n%s','\newfloat{example}{htbp}{pro}[chapter]');
    fprintf(fid,'\n%s','\floatname{example}{Example}');
    fprintf(fid,'\n%s','\begin{document}');
    fprintf(fid,'\n%s','\chapter{Stability Derivatives}');
    fprintf(fid,'\n%s','\begin{table}[htbp]');
    fprintf(fid,'\n%s','\centering');
    fprintf(fid,'\n%s','\begin{tabular}{|l|c|c|}');
    fprintf(fid,'\n%s','\hline\hline');
    fprintf(fid,'\n%s','\multicolumn{3}{c}{\textbf{Rigid}}\\\hline\hline');
    fprintf(fid,'\n%s',' & NeoCASS & Nastran\\\hline');
    fprintf(fid,'\n%s',['$C_{L_{\alpha}}$ & ',num2str(SD.Alpha.dcl_dalpha),'& ',num2str(CC(3,label.anglea,1)),'\\\hline']);
    fprintf(fid,'\n%s',['$C_{{\cal M}_{\alpha}}$ & ',num2str(SD.Alpha.dcmm_dalpha),'& ',num2str(CC(5,label.anglea,1)),'\\\hline']);
    fprintf(fid,'\n%s','\end{tabular}');
    % \hspace{10mm}
    % \begin{tabular}{|l|c|}
    % \hline\hline
    % \multicolumn{3}{c}{\textbf{H tail}}\\\hline\hline
    % $c_{root}$ [ft] & 5\\\hline
    % $c_{tip}$ [ft] & 2.35\\\hline
    % $b$ [ft] & 14.7\\\hline
    % $\Gamma_{le}$ [deg] & 29.1\\\hline
    % $x_{le}$ [ft] &  37.1\\\hline
    % $z_{le}$ [ft] & 7.88\\\hline
    % \end{tabular}
    % \hspace{10mm}
    % \begin{tabular}{|l|c|}
    % \hline\hline
    % \multicolumn{3}{c}{\textbf{V tail}}\\\hline\hline
    % $c_{root}$ [ft] & 9.05\\\hline
    % $c_{tip}$ [ft] & 4.19\\\hline
    % $b$ [ft] & 5.7\\\hline
    % $\Gamma_{le}$ [deg] & 46.115\\\hline
    % $x_{le}$ [ft] &  32\\\hline
    % $z_{le}$ [ft] & 1.18\\\hline
    % \end{tabular}
    fprintf(fid,'\n%s','\caption{$\alpha$ derivatives}');
    fprintf(fid,'\n%s','\label{tab:Alpha}');
    fprintf(fid,'\n%s','\end{table}');
    
    fprintf(fid,'\n%s','\begin{table}[htbp]');
    fprintf(fid,'\n%s','\centering');
    fprintf(fid,'\n%s','\begin{tabular}{|l|c|c|}');
    fprintf(fid,'\n%s','\hline\hline');
    fprintf(fid,'\n%s','\multicolumn{3}{c}{\textbf{Rigid}}\\\hline\hline');
    fprintf(fid,'\n%s',' & NeoCASS & Nastran\\\hline');
    fprintf(fid,'\n%s',['$C_{S_{\beta}}$ & ',num2str(SD.Beta.dcs_dbeta),'& ',num2str(CC(2,label.sides,1)),'\\\hline']);
    fprintf(fid,'\n%s',['$C_{L_{\beta}}$ & ',num2str(SD.Beta.dcl_dbeta),'& ',num2str(CC(3,label.sides,1)),'\\\hline']);
    fprintf(fid,'\n%s',['$C_{{\cal L}_{\beta}}$ & ',num2str(SD.Beta.dcml_dbeta),'& ',num2str(CC(4,label.sides,1)),'\\\hline']);
    fprintf(fid,'\n%s',['$C_{{\cal M}_{\beta}}$ & ',num2str(SD.Beta.dcmm_dbeta),'& ',num2str(CC(5,label.sides,1)),'\\\hline']);
    fprintf(fid,'\n%s',['$C_{{\cal N}_{\beta}}$ & ',num2str(SD.Beta.dcmn_dbeta),'& ',num2str(CC(6,label.sides,1)),'\\\hline']);
    fprintf(fid,'\n%s','\end{tabular}');
    % \hspace{10mm}
    % \begin{tabular}{|l|c|}
    % \hline\hline
    % \multicolumn{3}{c}{\textbf{H tail}}\\\hline\hline
    % $c_{root}$ [ft] & 5\\\hline
    % $c_{tip}$ [ft] & 2.35\\\hline
    % $b$ [ft] & 14.7\\\hline
    % $\Gamma_{le}$ [deg] & 29.1\\\hline
    % $x_{le}$ [ft] &  37.1\\\hline
    % $z_{le}$ [ft] & 7.88\\\hline
    % \end{tabular}
    % \hspace{10mm}
    % \begin{tabular}{|l|c|}
    % \hline\hline
    % \multicolumn{3}{c}{\textbf{V tail}}\\\hline\hline
    % $c_{root}$ [ft] & 9.05\\\hline
    % $c_{tip}$ [ft] & 4.19\\\hline
    % $b$ [ft] & 5.7\\\hline
    % $\Gamma_{le}$ [deg] & 46.115\\\hline
    % $x_{le}$ [ft] &  32\\\hline
    % $z_{le}$ [ft] & 1.18\\\hline
    % \end{tabular}
    fprintf(fid,'\n%s','\caption{$\beta$ derivatives}');
    fprintf(fid,'\n%s','\label{tab:beta}');
    fprintf(fid,'\n%s','\end{table}');
    
    fprintf(fid,'\n%s','\begin{table}[htbp]');
    fprintf(fid,'\n%s','\centering');
    fprintf(fid,'\n%s','\begin{tabular}{|l|c|c|}');
    fprintf(fid,'\n%s','\hline\hline');
    fprintf(fid,'\n%s','\multicolumn{3}{c}{\textbf{Rigid}}\\\hline\hline');
    fprintf(fid,'\n%s',' & NeoCASS & Nastran\\\hline');
    fprintf(fid,'\n%s',['$C_{S_{p}}$ & ',num2str(SD.P_rate.dcs_dP),'& ',num2str(CC(2,label.roll,1)),'\\\hline']);
    fprintf(fid,'\n%s',['$C_{{\cal L}_{p}}$ & ',num2str(SD.P_rate.dcml_dP),'& ',num2str(CC(4,label.roll,1)),'\\\hline']);
    fprintf(fid,'\n%s',['$C_{{\cal N}_{p}}$ & ',num2str(SD.P_rate.dcmn_dP),'& ',num2str(CC(6,label.roll,1)),'\\\hline']);
    fprintf(fid,'\n%s','\end{tabular}');
    % \hspace{10mm}
    % \begin{tabular}{|l|c|}
    % \hline\hline
    % \multicolumn{3}{c}{\textbf{H tail}}\\\hline\hline
    % $c_{root}$ [ft] & 5\\\hline
    % $c_{tip}$ [ft] & 2.35\\\hline
    % $b$ [ft] & 14.7\\\hline
    % $\Gamma_{le}$ [deg] & 29.1\\\hline
    % $x_{le}$ [ft] &  37.1\\\hline
    % $z_{le}$ [ft] & 7.88\\\hline
    % \end{tabular}
    % \hspace{10mm}
    % \begin{tabular}{|l|c|}
    % \hline\hline
    % \multicolumn{3}{c}{\textbf{V tail}}\\\hline\hline
    % $c_{root}$ [ft] & 9.05\\\hline
    % $c_{tip}$ [ft] & 4.19\\\hline
    % $b$ [ft] & 5.7\\\hline
    % $\Gamma_{le}$ [deg] & 46.115\\\hline
    % $x_{le}$ [ft] &  32\\\hline
    % $z_{le}$ [ft] & 1.18\\\hline
    % \end{tabular}
    fprintf(fid,'\n%s','\caption{$p$ derivatives}');
    fprintf(fid,'\n%s','\label{tab:P}');
    fprintf(fid,'\n%s','\end{table}');
    
    
    fprintf(fid,'\n%s','\begin{table}[htbp]');
    fprintf(fid,'\n%s','\centering');
    fprintf(fid,'\n%s','\begin{tabular}{|l|c|c|}');
    fprintf(fid,'\n%s','\hline\hline');
    fprintf(fid,'\n%s','\multicolumn{3}{c}{\textbf{Rigid}}\\\hline\hline');
    fprintf(fid,'\n%s',' & NeoCASS & Nastran\\\hline');
    fprintf(fid,'\n%s',['$C_{L_{q}}$ & ',num2str(SD.Q_rate.dcl_dQ),'& ',num2str(CC(3,label.pitch,1)),'\\\hline']);
    fprintf(fid,'\n%s',['$C_{{\cal M}_{q}}$ & ',num2str(SD.Q_rate.dcmm_dQ),'& ',num2str(CC(5,label.pitch,1)),'\\\hline']);
    fprintf(fid,'\n%s','\end{tabular}');
    % \hspace{10mm}
    % \begin{tabular}{|l|c|}
    % \hline\hline
    % \multicolumn{3}{c}{\textbf{H tail}}\\\hline\hline
    % $c_{root}$ [ft] & 5\\\hline
    % $c_{tip}$ [ft] & 2.35\\\hline
    % $b$ [ft] & 14.7\\\hline
    % $\Gamma_{le}$ [deg] & 29.1\\\hline
    % $x_{le}$ [ft] &  37.1\\\hline
    % $z_{le}$ [ft] & 7.88\\\hline
    % \end{tabular}
    % \hspace{10mm}
    % \begin{tabular}{|l|c|}
    % \hline\hline
    % \multicolumn{3}{c}{\textbf{V tail}}\\\hline\hline
    % $c_{root}$ [ft] & 9.05\\\hline
    % $c_{tip}$ [ft] & 4.19\\\hline
    % $b$ [ft] & 5.7\\\hline
    % $\Gamma_{le}$ [deg] & 46.115\\\hline
    % $x_{le}$ [ft] &  32\\\hline
    % $z_{le}$ [ft] & 1.18\\\hline
    % \end{tabular}
    fprintf(fid,'\n%s','\caption{$q$ derivatives}');
    fprintf(fid,'\n%s','\label{tab:q}');
    fprintf(fid,'\n%s','\end{table}');
    
    
    fprintf(fid,'\n%s','\begin{table}[htbp]');
    fprintf(fid,'\n%s','\centering');
    fprintf(fid,'\n%s','\begin{tabular}{|l|c|c|}');
    fprintf(fid,'\n%s','\hline\hline');
    fprintf(fid,'\n%s','\multicolumn{3}{c}{\textbf{Rigid}}\\\hline\hline');
    fprintf(fid,'\n%s',' & NeoCASS & Nastran\\\hline');
    fprintf(fid,'\n%s',['$C_{S_{r}}$ & ',num2str(SD.R_rate.dcs_dR),'& ',num2str(CC(2,label.yaw,1)),'\\\hline']);
    fprintf(fid,'\n%s',['$C_{L_{r}}$ & ',num2str(SD.R_rate.dcl_dR),'& ',num2str(CC(3,label.yaw,1)),'\\\hline']);
    fprintf(fid,'\n%s',['$C_{{\cal L}_{r}}$ & ',num2str(SD.R_rate.dcml_dR),'& ',num2str(CC(4,label.yaw,1)),'\\\hline']);
    fprintf(fid,'\n%s',['$C_{{\cal M}_{r}}$ & ',num2str(SD.R_rate.dcmm_dR),'& ',num2str(CC(5,label.yaw,1)),'\\\hline']);
    fprintf(fid,'\n%s',['$C_{{\cal N}_{r}}$ & ',num2str(SD.R_rate.dcmn_dR),'& ',num2str(CC(6,label.yaw,1)),'\\\hline']);
    fprintf(fid,'\n%s','\end{tabular}');
    % \hspace{10mm}
    % \begin{tabular}{|l|c|}
    % \hline\hline
    % \multicolumn{3}{c}{\textbf{H tail}}\\\hline\hline
    % $c_{root}$ [ft] & 5\\\hline
    % $c_{tip}$ [ft] & 2.35\\\hline
    % $b$ [ft] & 14.7\\\hline
    % $\Gamma_{le}$ [deg] & 29.1\\\hline
    % $x_{le}$ [ft] &  37.1\\\hline
    % $z_{le}$ [ft] & 7.88\\\hline
    % \end{tabular}
    % \hspace{10mm}
    % \begin{tabular}{|l|c|}
    % \hline\hline
    % \multicolumn{3}{c}{\textbf{V tail}}\\\hline\hline
    % $c_{root}$ [ft] & 9.05\\\hline
    % $c_{tip}$ [ft] & 4.19\\\hline
    % $b$ [ft] & 5.7\\\hline
    % $\Gamma_{le}$ [deg] & 46.115\\\hline
    % $x_{le}$ [ft] &  32\\\hline
    % $z_{le}$ [ft] & 1.18\\\hline
    % \end{tabular}
    fprintf(fid,'\n%s','\caption{$r$ derivatives}');
    fprintf(fid,'\n%s','\label{tab:r}');
    fprintf(fid,'\n%s','\end{table}');
    
    fprintf(fid,'\n%s','\chapter{Control Derivatives}');
    
    %      beam_model.Res.Aero.RStab_Der.Control
    %     dcs_dDelta: [1x6 double]
    %      dcl_dDelta: [-0.2790 -0.5134 2.0403e-005 0.1521 -8.4092e-004 0.0381]
    %     dcml_dDelta: [4.7722e-011 1.0337e-010 0.0532 -2.2715e-011 0.0180 3.4925e-012]
    %     dcmm_dDelta: [0.0144 0.0366 -9.9454e-006 -0.1195 6.0542e-004 0.0633]
    %     dcmn_dDelta: [1x6 double]
    %            Name: {'flap1l'  'flap2l'  'aileronl'  'elev2r'  'rudder2'  'elevC2r'}
    
    for i = 1:length(SD.Control.dcl_dDelta)
        switch SD.Control.Name{i}
            case 'flap1l'
                name = 'f1';
                incc = label.flap1;
            case 'flap2l'
                name = 'f2';
                incc = label.flap2;
            case 'aileronl'
                name = 'a';
                incc = label.aileron;
            case 'elev1r'
                name = 'e';
                incc = label.elev;
            case 'rudder1'
                name = 'r';
                incc = label.rudder;
            case 'elevC1r'
                name = 'c';
                incc = label.elevC;
            otherwise
                name = SD.Control.Name{i};
        end
        
        fprintf(fid,'\n%s','\begin{table}[htbp]');
        fprintf(fid,'\n%s','\centering');
        fprintf(fid,'\n%s','\begin{tabular}{|l|c|c|}');
        fprintf(fid,'\n%s','\hline\hline');
        fprintf(fid,'\n%s','\multicolumn{3}{c}{\textbf{Rigid}}\\\hline\hline');
        fprintf(fid,'\n%s',' & NeoCASS & Nastran\\\hline');
        fprintf(fid,'\n%s',['$C_{S_{\delta_{',name,'}}}$ & ',num2str(SD.Control.dcs_dDelta(i)),'& ',num2str(CC(2,incc,1)),'\\\hline']);
        fprintf(fid,'\n%s',['$C_{L_{\delta_{',name,'}}}$ & ',num2str(SD.Control.dcl_dDelta(i)),'& ',num2str(CC(3,incc,1)),'\\\hline']);
        fprintf(fid,'\n%s',['$C_{{\cal L}_{\delta_{',name,'}}}$ & ',num2str(SD.Control.dcml_dDelta(i)),'& ',num2str(CC(4,incc,1)),'\\\hline']);
        fprintf(fid,'\n%s',['$C_{{\cal M}_{\delta_{',name,'}}}$ & ',num2str(SD.Control.dcmm_dDelta(i)),'& ',num2str(CC(5,incc,1)),'\\\hline']);
        fprintf(fid,'\n%s',['$C_{{\cal N}_{\delta_{',name,'}}}$ & ',num2str(SD.Control.dcmn_dDelta(i)),'& ',num2str(CC(6,incc,1)),'\\\hline']);
        fprintf(fid,'\n%s','\end{tabular}');
        
        fprintf(fid,'\n%s',['\caption{$\delta_{',name,'}$ derivatives}']);
        fprintf(fid,'\n%s',['\label{tab:',SD.Control.Name{i},'}']);
        fprintf(fid,'\n%s','\end{table}');
    end
    
    
    
%     fprintf(fid,'\n%s','\begin{table}[htbp]');
%     fprintf(fid,'\n%s','\centering');
%     fprintf(fid,'\n%s','\begin{tabular}{|l|c|}');
%     fprintf(fid,'\n%s','\hline\hline');
%     fprintf(fid,'\n%s','\multicolumn{3}{c}{\textbf{Rigid}}\\\hline\hline');
%     fprintf(fid,'\n%s',['$C_{S_{\delta_{f2}}}$ & ',num2str(SD.Control.dcs_dDelta(2)),'\\\hline']);
%     fprintf(fid,'\n%s',['$C_{L_{\delta_{f2}}}$ & ',num2str(SD.Control.dcl_dDelta(2)),'\\\hline']);
%     fprintf(fid,'\n%s',['$C_{{\cal L}_{\delta_{f2}}}$ & ',num2str(SD.Control.dcml_dDelta(2)),'\\\hline']);
%     fprintf(fid,'\n%s',['$C_{{\cal M}_{\delta_{f2}}}$ & ',num2str(SD.Control.dcmm_dDelta(2)),'\\\hline']);
%     fprintf(fid,'\n%s',['$C_{{\cal N}_{\delta_{f2}}}$ & ',num2str(SD.Control.dcmn_dDelta(2)),'\\\hline']);
%     fprintf(fid,'\n%s','\end{tabular}');
%     % \hspace{10mm}
%     % \begin{tabular}{|l|c|}
%     % \hline\hline
%     % \multicolumn{3}{c}{\textbf{H tail}}\\\hline\hline
%     % $c_{root}$ [ft] & 5\\\hline
%     % $c_{tip}$ [ft] & 2.35\\\hline
%     % $b$ [ft] & 14.7\\\hline
%     % $\Gamma_{le}$ [deg] & 29.1\\\hline
%     % $x_{le}$ [ft] &  37.1\\\hline
%     % $z_{le}$ [ft] & 7.88\\\hline
%     % \end{tabular}
%     % \hspace{10mm}
%     % \begin{tabular}{|l|c|}
%     % \hline\hline
%     % \multicolumn{3}{c}{\textbf{V tail}}\\\hline\hline
%     % $c_{root}$ [ft] & 9.05\\\hline
%     % $c_{tip}$ [ft] & 4.19\\\hline
%     % $b$ [ft] & 5.7\\\hline
%     % $\Gamma_{le}$ [deg] & 46.115\\\hline
%     % $x_{le}$ [ft] &  32\\\hline
%     % $z_{le}$ [ft] & 1.18\\\hline
%     % \end{tabular}
%     fprintf(fid,'\n%s','\caption{$\delta_{f2}$ derivatives}');
%     fprintf(fid,'\n%s','\label{tab:d2}');
%     fprintf(fid,'\n%s','\end{table}');
%     
%     fprintf(fid,'\n%s','\begin{table}[htbp]');
%     fprintf(fid,'\n%s','\centering');
%     fprintf(fid,'\n%s','\begin{tabular}{|l|c|}');
%     fprintf(fid,'\n%s','\hline\hline');
%     fprintf(fid,'\n%s','\multicolumn{3}{c}{\textbf{Rigid}}\\\hline\hline');
%     fprintf(fid,'\n%s',['$C_{S_{\delta_{a}}}$ & ',num2str(SD.Control.dcs_dDelta(3)),'\\\hline']);
%     fprintf(fid,'\n%s',['$C_{L_{\delta_{a}}}$ & ',num2str(SD.Control.dcl_dDelta(3)),'\\\hline']);
%     fprintf(fid,'\n%s',['$C_{{\cal L}_{\delta_{a}}}$ & ',num2str(SD.Control.dcml_dDelta(3)),'\\\hline']);
%     fprintf(fid,'\n%s',['$C_{{\cal M}_{\delta_{a}}}$ & ',num2str(SD.Control.dcmm_dDelta(3)),'\\\hline']);
%     fprintf(fid,'\n%s',['$C_{{\cal N}_{\delta_{a}}}$ & ',num2str(SD.Control.dcmn_dDelta(3)),'\\\hline']);
%     fprintf(fid,'\n%s','\end{tabular}');
%     % \hspace{10mm}
%     % \begin{tabular}{|l|c|}
%     % \hline\hline
%     % \multicolumn{3}{c}{\textbf{H tail}}\\\hline\hline
%     % $c_{root}$ [ft] & 5\\\hline
%     % $c_{tip}$ [ft] & 2.35\\\hline
%     % $b$ [ft] & 14.7\\\hline
%     % $\Gamma_{le}$ [deg] & 29.1\\\hline
%     % $x_{le}$ [ft] &  37.1\\\hline
%     % $z_{le}$ [ft] & 7.88\\\hline
%     % \end{tabular}
%     % \hspace{10mm}
%     % \begin{tabular}{|l|c|}
%     % \hline\hline
%     % \multicolumn{3}{c}{\textbf{V tail}}\\\hline\hline
%     % $c_{root}$ [ft] & 9.05\\\hline
%     % $c_{tip}$ [ft] & 4.19\\\hline
%     % $b$ [ft] & 5.7\\\hline
%     % $\Gamma_{le}$ [deg] & 46.115\\\hline
%     % $x_{le}$ [ft] &  32\\\hline
%     % $z_{le}$ [ft] & 1.18\\\hline
%     % \end{tabular}
%     fprintf(fid,'\n%s','\caption{$\delta_{a}$ derivatives}');
%     fprintf(fid,'\n%s','\label{tab:da}');
%     fprintf(fid,'\n%s','\end{table}');
%     
%     
%     fprintf(fid,'\n%s','\begin{table}[htbp]');
%     fprintf(fid,'\n%s','\centering');
%     fprintf(fid,'\n%s','\begin{tabular}{|l|c|}');
%     fprintf(fid,'\n%s','\hline\hline');
%     fprintf(fid,'\n%s','\multicolumn{3}{c}{\textbf{Rigid}}\\\hline\hline');
%     fprintf(fid,'\n%s',['$C_{S_{\delta_{r}}}$ & ',num2str(SD.Control.dcs_dDelta(4)),'\\\hline']);
%     fprintf(fid,'\n%s',['$C_{L_{\delta_{r}}}$ & ',num2str(SD.Control.dcl_dDelta(4)),'\\\hline']);
%     fprintf(fid,'\n%s',['$C_{{\cal L}_{\delta_{r}}}$ & ',num2str(SD.Control.dcml_dDelta(4)),'\\\hline']);
%     fprintf(fid,'\n%s',['$C_{{\cal M}_{\delta_{r}}}$ & ',num2str(SD.Control.dcmm_dDelta(4)),'\\\hline']);
%     fprintf(fid,'\n%s',['$C_{{\cal N}_{\delta_{r}}}$ & ',num2str(SD.Control.dcmn_dDelta(4)),'\\\hline']);
%     fprintf(fid,'\n%s','\end{tabular}');
%     % \hspace{10mm}
%     % \begin{tabular}{|l|c|}
%     % \hline\hline
%     % \multicolumn{3}{c}{\textbf{H tail}}\\\hline\hline
%     % $c_{root}$ [ft] & 5\\\hline
%     % $c_{tip}$ [ft] & 2.35\\\hline
%     % $b$ [ft] & 14.7\\\hline
%     % $\Gamma_{le}$ [deg] & 29.1\\\hline
%     % $x_{le}$ [ft] &  37.1\\\hline
%     % $z_{le}$ [ft] & 7.88\\\hline
%     % \end{tabular}
%     % \hspace{10mm}
%     % \begin{tabular}{|l|c|}
%     % \hline\hline
%     % \multicolumn{3}{c}{\textbf{V tail}}\\\hline\hline
%     % $c_{root}$ [ft] & 9.05\\\hline
%     % $c_{tip}$ [ft] & 4.19\\\hline
%     % $b$ [ft] & 5.7\\\hline
%     % $\Gamma_{le}$ [deg] & 46.115\\\hline
%     % $x_{le}$ [ft] &  32\\\hline
%     % $z_{le}$ [ft] & 1.18\\\hline
%     % \end{tabular}
%     fprintf(fid,'\n%s','\caption{$\delta_{r}$ derivatives}');
%     fprintf(fid,'\n%s','\label{tab:dr}');
%     fprintf(fid,'\n%s','\end{table}');
%     
%     fprintf(fid,'\n%s','\begin{table}[htbp]');
%     fprintf(fid,'\n%s','\centering');
%     fprintf(fid,'\n%s','\begin{tabular}{|l|c|}');
%     fprintf(fid,'\n%s','\hline\hline');
%     fprintf(fid,'\n%s','\multicolumn{3}{c}{\textbf{Rigid}}\\\hline\hline');
%     fprintf(fid,'\n%s',['$C_{S_{\delta_{e}}}$ & ',num2str(SD.Control.dcs_dDelta(5)),'\\\hline']);
%     fprintf(fid,'\n%s',['$C_{L_{\delta_{e}}}$ & ',num2str(SD.Control.dcl_dDelta(5)),'\\\hline']);
%     fprintf(fid,'\n%s',['$C_{{\cal L}_{\delta_{e}}}$ & ',num2str(SD.Control.dcml_dDelta(5)),'\\\hline']);
%     fprintf(fid,'\n%s',['$C_{{\cal M}_{\delta_{e}}}$ & ',num2str(SD.Control.dcmm_dDelta(5)),'\\\hline']);
%     fprintf(fid,'\n%s',['$C_{{\cal N}_{\delta_{e}}}$ & ',num2str(SD.Control.dcmn_dDelta(5)),'\\\hline']);
%     fprintf(fid,'\n%s','\end{tabular}');
%     % \hspace{10mm}
%     % \begin{tabular}{|l|c|}
%     % \hline\hline
%     % \multicolumn{3}{c}{\textbf{H tail}}\\\hline\hline
%     % $c_{root}$ [ft] & 5\\\hline
%     % $c_{tip}$ [ft] & 2.35\\\hline
%     % $b$ [ft] & 14.7\\\hline
%     % $\Gamma_{le}$ [deg] & 29.1\\\hline
%     % $x_{le}$ [ft] &  37.1\\\hline
%     % $z_{le}$ [ft] & 7.88\\\hline
%     % \end{tabular}
%     % \hspace{10mm}
%     % \begin{tabular}{|l|c|}
%     % \hline\hline
%     % \multicolumn{3}{c}{\textbf{V tail}}\\\hline\hline
%     % $c_{root}$ [ft] & 9.05\\\hline
%     % $c_{tip}$ [ft] & 4.19\\\hline
%     % $b$ [ft] & 5.7\\\hline
%     % $\Gamma_{le}$ [deg] & 46.115\\\hline
%     % $x_{le}$ [ft] &  32\\\hline
%     % $z_{le}$ [ft] & 1.18\\\hline
%     % \end{tabular}
%     fprintf(fid,'\n%s','\caption{$\delta_{e}$ derivatives}');
%     fprintf(fid,'\n%s','\label{tab:de}');
%     fprintf(fid,'\n%s','\end{table}');
    
    
    fprintf(fid,'\n%s','\end{document}');
    fclose(fid);
    system(['texi2dvi ',file_name,'.tex']);
    system(['dvipdfmx ',file_name,'.dvi']);
    
else           % also elastic Stability derivatives
    RSD = varargin{1};
    DSD = varargin{2};
    
    fid = fopen([file_name,'.tex'],'w');
    
    fprintf(fid,'%s','%Report written by NeoCASS');
    fprintf(fid,'\n%s','\documentclass[openany]{book}%[twocolumn]');
    fprintf(fid,'\n%s','\usepackage[ansinew]{inputenc}');
    fprintf(fid,'\n%s','\usepackage{amsmath}%');
    fprintf(fid,'\n%s','\usepackage{amsfonts}%');
    fprintf(fid,'\n%s','\usepackage{fontenc}');
    fprintf(fid,'\n%s','\usepackage{amssymb}%');
    fprintf(fid,'\n%s','\usepackage{graphicx}');
    fprintf(fid,'\n%s','\usepackage{color}');
    fprintf(fid,'\n%s','\usepackage{epsfig}');
    fprintf(fid,'\n%s','\usepackage{graphics}%[dvips]');
    fprintf(fid,'\n%s','\usepackage{subfigure}%');
    fprintf(fid,'\n%s','\usepackage{fancyhdr}');
    fprintf(fid,'\n%s','\usepackage[Glenn]{fncychap}');
    fprintf(fid,'\n%s','\usepackage{eso-pic}');
    fprintf(fid,'\n%s','\usepackage{makeidx}');
    fprintf(fid,'\n%s','\usepackage[english,refpage,intoc]{nomencl}');
    fprintf(fid,'\n%s','\usepackage{pifont}');
    fprintf(fid,'\n%s','\usepackage{float}');
    fprintf(fid,'\n%s','\usepackage{wrapfig}');
%     fprintf(fid,'\n%s','\usepackage[vflt]{floatflt}');
    fprintf(fid,'\n%s','\usepackage{fancybox}');
    fprintf(fid,'\n%s','\usepackage{booktabs}');
    fprintf(fid,'\n%s','\usepackage{lipsum}');
    fprintf(fid,'\n%s','%\usepackage{classicthesis-ldpkg}');
    fprintf(fid,'\n%s','\usepackage[dvipdfmx]{hyperref}');
    fprintf(fid,'\n%s','\hypersetup{%');
    fprintf(fid,'\n%s','colorlinks=true, linktocpage=true, pdfstartpage=1, pdfstartview=FitV,%');
    fprintf(fid,'\n%s','breaklinks=true, pdfpagemode=UseNone, pageanchor=true, pdfpagemode=UseOutlines,%');
    fprintf(fid,'\n%s','plainpages=false, bookmarksnumbered, bookmarksopen=true, bookmarksopenlevel=1,%');
    fprintf(fid,'\n%s','hypertexnames=true, pdfhighlight=/O,%hyperfootnotes=true,%nesting=true,%frenchlinks,%');
    fprintf(fid,'\n%s','urlcolor=webbrown, linkcolor=blue, citecolor=webgreen %pagecolor=RoyalBlue,%');
    fprintf(fid,'\n%s','% uncomment the following line if you want to have black links (e.g., for printing)');
    fprintf(fid,'\n%s','%urlcolor=Black, linkcolor=Black, citecolor=Black, %pagecolor=Black,%');
    fprintf(fid,'\n%s','%pdftitle={\myTitle},%');
    fprintf(fid,'\n%s','%pdfauthor={\textcopyright\ \myName, \myUni, \myFaculty},%');
    fprintf(fid,'\n%s','%pdfsubject={},%');
    fprintf(fid,'\n%s','%pdfkeywords={},%');
    fprintf(fid,'\n%s','%pdfcreator={dvipdfmx},%');
    fprintf(fid,'\n%s','%pdfproducer={LaTeX with hyperref and classicthesis}%');
    fprintf(fid,'\n%s','}');
    fprintf(fid,'\n%s','%----------------------------------------------------------');
    fprintf(fid,'\n%s','\newtheorem{theorem}{Theorem}');
    fprintf(fid,'\n%s','\newtheorem{acknowledgement}[theorem]{Acknowledgement}');
    fprintf(fid,'\n%s','\newtheorem{algorithm}[theorem]{Algorithm}');
    fprintf(fid,'\n%s','\newtheorem{axiom}[theorem]{Axiom}');
    fprintf(fid,'\n%s','\newtheorem{case}[theorem]{Case}');
    fprintf(fid,'\n%s','\newtheorem{claim}[theorem]{Claim}');
    fprintf(fid,'\n%s','\newtheorem{conclusion}[theorem]{Conclusion}');
    fprintf(fid,'\n%s','\newtheorem{condition}[theorem]{Condition}');
    fprintf(fid,'\n%s','\newtheorem{conjecture}[theorem]{Conjecture}');
    fprintf(fid,'\n%s','\newtheorem{corollary}[theorem]{Corollary}');
    fprintf(fid,'\n%s','\newtheorem{criterion}[theorem]{Criterion}');
    fprintf(fid,'\n%s','\newtheorem{definition}[theorem]{Definition}');
    fprintf(fid,'\n%s','\newtheorem{example}[theorem]{Example}');
    fprintf(fid,'\n%s','\newtheorem{exercise}[theorem]{Exercise}');
    fprintf(fid,'\n%s','\newtheorem{lemma}[theorem]{Lemma}');
    fprintf(fid,'\n%s','\newtheorem{notation}[theorem]{Notation}');
    fprintf(fid,'\n%s','\newtheorem{problem}[theorem]{Problem}');
    fprintf(fid,'\n%s','\newtheorem{proposition}[theorem]{Proposition}');
    fprintf(fid,'\n%s','\newtheorem{remark}[theorem]{Remark}');
    fprintf(fid,'\n%s','\newtheorem{solution}[theorem]{Solution}');
    fprintf(fid,'\n%s','\newtheorem{summary}[theorem]{Summary}');
    fprintf(fid,'\n%s','\newenvironment{proof}[1][Proof]{\textbf{#1.} }{\ \rule{0.9em}{0.9em}}');
    fprintf(fid,'\n%s','\setlength{\topmargin}{-0.3in}');
    fprintf(fid,'\n%s','\setlength{\topskip}{0.2in}    % between header and text');
    fprintf(fid,'\n%s','\setlength{\textheight}{9.5in} % height of main text');
    fprintf(fid,'\n%s','\setlength{\textwidth}{6in}    % width of text');
    fprintf(fid,'\n%s','\setlength{\oddsidemargin}{0in} % odd page left margin');
    fprintf(fid,'\n%s','\setlength{\evensidemargin}{0.2in}');
    fprintf(fid,'\n%s','\pagestyle{fancy}');
    fprintf(fid,'\n%s','\newcommand{\fncyfront}{%');
    fprintf(fid,'\n%s','\fancyhead[RO]{\footnotesize\rightmark}');
    fprintf(fid,'\n%s','\fancyfoot[RO]{\footnotesize\textcolor[gray]{0.5}{\textbf{SimSAC}}}');
    fprintf(fid,'\n%s','\fancyhead[LE]{\footnotesize{\leftmark }}');
    fprintf(fid,'\n%s','\fancyfoot[LE]{\footnotesize\textcolor[gray]{0.5}{\textbf{NeoCASS}}}');
    fprintf(fid,'\n%s','\fancyhead[LO,RE]{{\textcolor[gray]{0.35}{\footnotesize \textbf{Stability Derivatives}}}}');
    fprintf(fid,'\n%s','\fancyfoot[C]{\thepage}');
    fprintf(fid,'\n%s','\renewcommand{\headrulewidth}{1.3 pt}');
    fprintf(fid,'\n%s','\renewcommand{\footrulewidth}{1.3 pt}}');
    fprintf(fid,'\n%s','\newcommand {\fncymain }{%');
    fprintf(fid,'\n%s','\fancyhead [RO ]{{\scriptsize \MakeUppercase');
    fprintf(fid,'\n%s','    \rightmark }}');
    fprintf(fid,'\n%s','\fancyfoot [RO ]{\thepage }');
    fprintf(fid,'\n%s','\fancyhead [LE ]{{\scriptsize \MakeUppercase');
    fprintf(fid,'\n%s','    \leftmark }}');
    fprintf(fid,'\n%s','\fancyfoot [LE ]{\thepage }');
    fprintf(fid,'\n%s','\fancyfoot [C]{}');
    fprintf(fid,'\n%s','\renewcommand {\headrulewidth }{0.3 pt }} ');
    fprintf(fid,'\n%s','\newcommand\AlCentroPagina[1]{ %');
    fprintf(fid,'\n%s','    \AddToShipoutPicture *{\AtPageCenter {%');
    fprintf(fid,'\n%s','    \makebox (0,0){\includegraphics %');
    fprintf(fid,'\n%s','[width =0.5\paperwidth]{#1}}}}}');
    fprintf(fid,'\n%s','\makeatletter');
    fprintf(fid,'\n%s','\ChNameVar{\huge\rm} % sets the style for name');
    fprintf(fid,'\n%s','\ChNumVar{\Huge\rm\centering} % sets the style for digit');
    fprintf(fid,'\n%s','\ChTitleVar{\Large\rm\centering} % sets the style for title');
    fprintf(fid,'\n%s','\ChRuleWidth{2pt} % Set RW=4pt');
    fprintf(fid,'\n%s','\ChNameUpperCase % Make name uppercase');
    fprintf(fid,'\n%s','\renewcommand{\DOCH}{%');
    fprintf(fid,'\n%s','\setlength{\fboxrule}{\RW} % Let fbox lines be controlled by');
    fprintf(fid,'\n%s','% \ChRuleWidth');
    fprintf(fid,'\n%s','\centering\fbox{ \textcolor[gray]{0.35}{\CNV\FmN{\@chapapp}}\space \textcolor[gray]{0.35}{\CNoV\thechapter}}\par\nobreak');
    fprintf(fid,'\n%s','\vskip 10\p@}');
    fprintf(fid,'\n%s','\renewcommand{\DOTI}[1]{%');
    fprintf(fid,'\n%s','\CTV\FmTi{#1}\par\nobreak');
    fprintf(fid,'\n%s','\vskip 50\p@}');
    fprintf(fid,'\n%s','\renewcommand{\DOTIS}[1]{%');
    fprintf(fid,'\n%s','\CTV\FmTi{#1}\par\nobreak');
    fprintf(fid,'\n%s','\vskip 20\p@}');
    fprintf(fid,'\n%s','\makeatother');
    fprintf(fid,'\n%s','\def\cleardoublepage{\clearpage\if@twoside');
    fprintf(fid,'\n%s','\ifodd\c@page');
    fprintf(fid,'\n%s','\else\hbox{}\thispagestyle{empty}\newpage');
    fprintf(fid,'\n%s','\if@twocolumn\hbox{}\newpage\fi\fi\fi}     ');
    fprintf(fid,'\n%s','\makeindex');
    fprintf(fid,'\n%s','\makenomenclature');
    fprintf(fid,'\n%s','\floatstyle{ruled}');
    fprintf(fid,'\n%s','\newfloat{appendix}{ht}{pro}[chapter]');
    fprintf(fid,'\n%s','\floatname{appendix}{Appendix}');
    fprintf(fid,'\n%s','\floatstyle{ruled}');
    fprintf(fid,'\n%s','\newfloat{example}{htbp}{pro}[chapter]');
    fprintf(fid,'\n%s','\floatname{example}{Example}');
    fprintf(fid,'\n%s','\begin{document}');
    fprintf(fid,'\n%s','\chapter{Stability Derivatives}');
    fprintf(fid,'\n%s','\begin{table}[htbp]');
    fprintf(fid,'\n%s','\centering');
    fprintf(fid,'\n%s','\begin{tabular}{|l|c|c|}');
    fprintf(fid,'\n%s','\hline\hline');
    fprintf(fid,'\n%s','\multicolumn{3}{c}{\textbf{Rigid}}\\\hline\hline');
    fprintf(fid,'\n%s',' & NeoCASS & Nastran\\\hline');
    fprintf(fid,'\n%s',['$C_{L_{\alpha}}$ & ',num2str(RSD.Alpha.dcl_dalpha),'&',num2str(CC(3,label.anglea,1),'%4g'),'\\\hline']);
    fprintf(fid,'\n%s',['$C_{{\cal M}_{\alpha}}$ & ',num2str(RSD.Alpha.dcmm_dalpha),'&',num2str(CC(5,label.anglea,1),'%4g'),'\\\hline']);
    fprintf(fid,'\n%s','\end{tabular}');
    
    fprintf(fid,'\n%s','\hspace{10mm}');
    
    fprintf(fid,'\n%s','\begin{tabular}{|l|c|c|}');
    fprintf(fid,'\n%s','\hline\hline');
    fprintf(fid,'\n%s','\multicolumn{3}{c}{\textbf{Deformable}}\\\hline\hline');
    fprintf(fid,'\n%s',' & NeoCASS & Nastran\\\hline');
    fprintf(fid,'\n%s',['$C_{L_{\alpha}}$ & ',num2str(DSD.Alpha.dcl_dalpha),'&',num2str(CC(3,label.anglea,2),'%4g'),'\\\hline']);
    fprintf(fid,'\n%s',['$C_{{\cal M}_{\alpha}}$ & ',num2str(DSD.Alpha.dcmm_dalpha),'&',num2str(CC(5,label.anglea,2),'%4g'),'\\\hline']);
    fprintf(fid,'\n%s','\end{tabular}');
    
    fprintf(fid,'\n%s','\hspace{10mm}');
    
    fprintf(fid,'\n%s','\begin{tabular}{|l|c|c|}');
    fprintf(fid,'\n%s','\hline\hline');
    fprintf(fid,'\n%s','\multicolumn{3}{c}{\textbf{D$/$R}}\\\hline\hline');
    fprintf(fid,'\n%s',' & NeoCASS & Nastran\\\hline');
    fprintf(fid,'\n%s',['$C_{L_{\alpha}}$ & ',num2str(DSD.Alpha.dcl_dalpha/RSD.Alpha.dcl_dalpha),'&',num2str(CC(3,label.anglea,2)/CC(3,label.anglea,1),'%4g'),'\\\hline']);
    fprintf(fid,'\n%s',['$C_{{\cal M}_{\alpha}}$ & ',num2str(DSD.Alpha.dcmm_dalpha/RSD.Alpha.dcmm_dalpha),'&',num2str(CC(5,label.anglea,2)/CC(5,label.anglea,1),'%4g'),'\\\hline']);
    fprintf(fid,'\n%s','\end{tabular}');
    
    fprintf(fid,'\n%s','\caption{$\alpha$ derivatives}');
    fprintf(fid,'\n%s','\label{tab:Alpha}');
    fprintf(fid,'\n%s','\end{table}');
    
    fprintf(fid,'\n%s','\begin{table}[htbp]');
    fprintf(fid,'\n%s','\centering');
    fprintf(fid,'\n%s','\begin{tabular}{|l|c|c|}');
    fprintf(fid,'\n%s','\hline\hline');
    fprintf(fid,'\n%s','\multicolumn{3}{c}{\textbf{Rigid}}\\\hline\hline');
    fprintf(fid,'\n%s',' & NeoCASS & Nastran\\\hline');
    fprintf(fid,'\n%s',['$C_{S_{\beta}}$ & ',num2str(RSD.Beta.dcs_dbeta),'&',num2str(CC(2,label.sides,1),'%4g'),'\\\hline']);
    fprintf(fid,'\n%s',['$C_{L_{\beta}}$ & ',num2str(RSD.Beta.dcl_dbeta),'&',num2str(CC(3,label.sides,1),'%4g'),'\\\hline']);
    fprintf(fid,'\n%s',['$C_{{\cal L}_{\beta}}$ & ',num2str(RSD.Beta.dcml_dbeta),'&',num2str(CC(4,label.sides,1),'%4g'),'\\\hline']);
    fprintf(fid,'\n%s',['$C_{{\cal M}_{\beta}}$ & ',num2str(RSD.Beta.dcmm_dbeta),'&',num2str(CC(5,label.sides,1),'%4g'),'\\\hline']);
    fprintf(fid,'\n%s',['$C_{{\cal N}_{\beta}}$ & ',num2str(RSD.Beta.dcmn_dbeta),'&',num2str(CC(6,label.sides,1),'%4g'),'\\\hline']);
    fprintf(fid,'\n%s','\end{tabular}');
    
    fprintf(fid,'\n%s','\hspace{10mm}');
    
    fprintf(fid,'\n%s','\begin{tabular}{|l|c|c|}');
    fprintf(fid,'\n%s','\hline\hline');
    fprintf(fid,'\n%s','\multicolumn{3}{c}{\textbf{Deformable}}\\\hline\hline');
    fprintf(fid,'\n%s',' & NeoCASS & Nastran\\\hline');
    fprintf(fid,'\n%s',['$C_{S_{\beta}}$ & ',num2str(DSD.Beta.dcs_dbeta),'&',num2str(CC(2,label.sides,2),'%4g'),'\\\hline']);
    fprintf(fid,'\n%s',['$C_{L_{\beta}}$ & ',num2str(DSD.Beta.dcl_dbeta),'&',num2str(CC(3,label.sides,2),'%4g'),'\\\hline']);
    fprintf(fid,'\n%s',['$C_{{\cal L}_{\beta}}$ & ',num2str(DSD.Beta.dcml_dbeta),'&',num2str(CC(4,label.sides,2),'%4g'),'\\\hline']);
    fprintf(fid,'\n%s',['$C_{{\cal M}_{\beta}}$ & ',num2str(DSD.Beta.dcmm_dbeta),'&',num2str(CC(5,label.sides,2),'%4g'),'\\\hline']);
    fprintf(fid,'\n%s',['$C_{{\cal N}_{\beta}}$ & ',num2str(DSD.Beta.dcmn_dbeta),'&',num2str(CC(6,label.sides,2),'%4g'),'\\\hline']);
    fprintf(fid,'\n%s','\end{tabular}');
    
    
    fprintf(fid,'\n%s','\hspace{10mm}');
    
    fprintf(fid,'\n%s','\begin{tabular}{|l|c|c|}');
    fprintf(fid,'\n%s','\hline\hline');
    fprintf(fid,'\n%s','\multicolumn{3}{c}{\textbf{D/R}}\\\hline\hline');
    fprintf(fid,'\n%s',' & NeoCASS & Nastran\\\hline');
    if abs(DSD.Beta.dcs_dbeta/RSD.Beta.dcs_dbeta)>2 || abs(DSD.Beta.dcs_dbeta/RSD.Beta.dcs_dbeta)<1e-3
        fprintf(fid,'\n%s','$C_{S_{\beta}}$ & //&// \\\hline');
    else
        fprintf(fid,'\n%s',['$C_{S_{\beta}}$ & ',num2str(DSD.Beta.dcs_dbeta/RSD.Beta.dcs_dbeta),'&',num2str(CC(2,label.sides,2)/CC(2,label.sides,1),'%4g'),'\\\hline']);
    end
    if abs(DSD.Beta.dcl_dbeta/RSD.Beta.dcl_dbeta)>2 || abs(DSD.Beta.dcl_dbeta/RSD.Beta.dcl_dbeta) <1e-3
        fprintf(fid,'\n%s','$C_{L_{\beta}}$ & //& // \\\hline');
    else
        fprintf(fid,'\n%s',['$C_{L_{\beta}}$ & ',num2str(DSD.Beta.dcl_dbeta/RSD.Beta.dcl_dbeta),'&',num2str(CC(3,label.sides,2)/CC(3,label.sides,1),'%4g'),'\\\hline']);
    end
    if abs(DSD.Beta.dcml_dbeta/RSD.Beta.dcml_dbeta)>2 || abs(DSD.Beta.dcml_dbeta/RSD.Beta.dcml_dbeta)<1e-3
        fprintf(fid,'\n%s','$C_{{\cal L}_{\beta}}$ & //& // \\\hline');
    else
        fprintf(fid,'\n%s',['$C_{{\cal L}_{\beta}}$ & ',num2str(DSD.Beta.dcml_dbeta/RSD.Beta.dcml_dbeta),'&',num2str(CC(4,label.sides,2)/CC(4,label.sides,1),'%4g'),'\\\hline']);
    end
    if abs(DSD.Beta.dcmm_dbeta/RSD.Beta.dcmm_dbeta)>2 || abs(DSD.Beta.dcmm_dbeta/RSD.Beta.dcmm_dbeta)<1e-3
        fprintf(fid,'\n%s','$C_{{\cal M}_{\beta}}$ & //& //\\\hline');
    else
        fprintf(fid,'\n%s',['$C_{{\cal M}_{\beta}}$ & ',num2str(DSD.Beta.dcmm_dbeta/RSD.Beta.dcmm_dbeta),'&',num2str(CC(5,label.sides,2)/CC(5,label.sides,1),'%4g'),'\\\hline']);
    end
    if abs(DSD.Beta.dcmn_dbeta/RSD.Beta.dcmn_dbeta)>2 || abs(DSD.Beta.dcmn_dbeta/RSD.Beta.dcmn_dbeta)<1e-3
        fprintf(fid,'\n%s','$C_{{\cal N}_{\beta}}$ & //& // \\\hline');
    else
        fprintf(fid,'\n%s',['$C_{{\cal N}_{\beta}}$ & ',num2str(DSD.Beta.dcmn_dbeta/RSD.Beta.dcmn_dbeta),'&',num2str(CC(6,label.sides,2)/CC(6,label.sides,1),'%4g'),'\\\hline']);
    end
    fprintf(fid,'\n%s','\end{tabular}');
    
    
    fprintf(fid,'\n%s','\caption{$\beta$ derivatives}');
    fprintf(fid,'\n%s','\label{tab:beta}');
    fprintf(fid,'\n%s','\end{table}');
    
    fprintf(fid,'\n%s','\begin{table}[htbp]');
    fprintf(fid,'\n%s','\centering');
    fprintf(fid,'\n%s','\begin{tabular}{|l|c|c|}');
    fprintf(fid,'\n%s','\hline\hline');
    fprintf(fid,'\n%s','\multicolumn{3}{c}{\textbf{Rigid}}\\\hline\hline');
    fprintf(fid,'\n%s',' & NeoCASS & Nastran\\\hline');
    fprintf(fid,'\n%s',['$C_{S_{p}}$ & ',num2str(RSD.P_rate.dcs_dP),'&',num2str(CC(2,label.roll,1),'%4g'),'\\\hline']);
    fprintf(fid,'\n%s',['$C_{{\cal L}_{p}}$ & ',num2str(RSD.P_rate.dcml_dP),'&',num2str(CC(4,label.roll,1),'%4g'),'\\\hline']);
    fprintf(fid,'\n%s',['$C_{{\cal N}_{p}}$ & ',num2str(RSD.P_rate.dcmn_dP),'&',num2str(CC(6,label.roll,1),'%4g'),'\\\hline']);
    fprintf(fid,'\n%s','\end{tabular}');
    
    fprintf(fid,'\n%s','\hspace{10mm}');
    
    fprintf(fid,'\n%s','\begin{tabular}{|l|c|c|}');
    fprintf(fid,'\n%s','\hline\hline');
    fprintf(fid,'\n%s','\multicolumn{3}{c}{\textbf{Rigid}}\\\hline\hline');
    fprintf(fid,'\n%s',' & NeoCASS & Nastran\\\hline');
    fprintf(fid,'\n%s',['$C_{S_{p}}$ & ',num2str(DSD.P_rate.dcs_dP),'&',num2str(CC(2,label.roll,2),'%4g'),'\\\hline']);
    fprintf(fid,'\n%s',['$C_{{\cal L}_{p}}$ & ',num2str(DSD.P_rate.dcml_dP),'&',num2str(CC(4,label.roll,2),'%4g'),'\\\hline']);
    fprintf(fid,'\n%s',['$C_{{\cal N}_{p}}$ & ',num2str(DSD.P_rate.dcmn_dP),'&',num2str(CC(5,label.roll,2),'%4g'),'\\\hline']);
    fprintf(fid,'\n%s','\end{tabular}');
    
    fprintf(fid,'\n%s','\hspace{10mm}');
    
    fprintf(fid,'\n%s','\begin{tabular}{|l|c|c|}');
    fprintf(fid,'\n%s','\hline\hline');
    fprintf(fid,'\n%s','\multicolumn{3}{c}{\textbf{D/R}}\\\hline\hline');
    fprintf(fid,'\n%s',' & NeoCASS & Nastran\\\hline');
    fprintf(fid,'\n%s',['$C_{S_{p}}$ & ',num2str(DSD.P_rate.dcs_dP/RSD.P_rate.dcs_dP),'&',num2str(CC(2,label.roll,2)/CC(2,label.roll,1),'%4g'),'\\\hline']);
    fprintf(fid,'\n%s',['$C_{{\cal L}_{p}}$ & ',num2str(DSD.P_rate.dcml_dP/RSD.P_rate.dcml_dP),'&',num2str(CC(4,label.roll,2)/CC(4,label.roll,1),'%4g'),'\\\hline']);
    fprintf(fid,'\n%s',['$C_{{\cal N}_{p}}$ & ',num2str(DSD.P_rate.dcmn_dP/RSD.P_rate.dcmn_dP),'&',num2str(CC(6,label.roll,2)/CC(6,label.roll,1),'%4g'),'\\\hline']);
    fprintf(fid,'\n%s','\end{tabular}');
    fprintf(fid,'\n%s','\caption{$p$ derivatives}');
    fprintf(fid,'\n%s','\label{tab:P}');
    fprintf(fid,'\n%s','\end{table}');
    
    
    fprintf(fid,'\n%s','\begin{table}[htbp]');
    fprintf(fid,'\n%s','\centering');
    fprintf(fid,'\n%s','\begin{tabular}{|l|c|c|}');
    fprintf(fid,'\n%s','\hline\hline');
    fprintf(fid,'\n%s','\multicolumn{3}{c}{\textbf{Rigid}}\\\hline\hline');
    fprintf(fid,'\n%s',' & NeoCASS & Nastran\\\hline');
    fprintf(fid,'\n%s',['$C_{L_{q}}$ & ',num2str(RSD.Q_rate.dcl_dQ),'&',num2str(CC(3,label.pitch,1),'%4g'),'\\\hline']);
    fprintf(fid,'\n%s',['$C_{{\cal M}_{q}}$ & ',num2str(RSD.Q_rate.dcmm_dQ),'&',num2str(CC(5,label.pitch,1),'%4g'),'\\\hline']);
    fprintf(fid,'\n%s','\end{tabular}');
    
    fprintf(fid,'\n%s','\hspace{10mm}');
   
    fprintf(fid,'\n%s','\begin{tabular}{|l|c|c|}');
    fprintf(fid,'\n%s','\hline\hline');
    fprintf(fid,'\n%s','\multicolumn{3}{c}{\textbf{Deformable}}\\\hline\hline');
    fprintf(fid,'\n%s',' & NeoCASS & Nastran\\\hline');
    fprintf(fid,'\n%s',['$C_{L_{q}}$ & ',num2str(DSD.Q_rate.dcl_dQ),'&',num2str(CC(3,label.pitch,2),'%4g'),'\\\hline']);
    fprintf(fid,'\n%s',['$C_{{\cal M}_{q}}$ & ',num2str(DSD.Q_rate.dcmm_dQ),'&',num2str(CC(5,label.pitch,2),'%4g'),'\\\hline']);
    fprintf(fid,'\n%s','\end{tabular}');
    
    fprintf(fid,'\n%s','\hspace{10mm}');
   
    fprintf(fid,'\n%s','\begin{tabular}{|l|c|c|}');
    fprintf(fid,'\n%s','\hline\hline');
    fprintf(fid,'\n%s','\multicolumn{3}{c}{\textbf{D/R}}\\\hline\hline');
    fprintf(fid,'\n%s',['$C_{L_{q}}$ & ',num2str(DSD.Q_rate.dcl_dQ/RSD.Q_rate.dcl_dQ),'&',num2str(CC(3,label.pitch,2)/CC(3,label.pitch,1),'%4g'),'\\\hline']);
    fprintf(fid,'\n%s',['$C_{{\cal M}_{q}}$ & ',num2str(DSD.Q_rate.dcmm_dQ/RSD.Q_rate.dcmm_dQ),'&',num2str(CC(5,label.pitch,2)/CC(5,label.pitch,1),'%4g'),'\\\hline']);
    fprintf(fid,'\n%s','\end{tabular}');
    
    fprintf(fid,'\n%s','\caption{$q$ derivatives}');
    fprintf(fid,'\n%s','\label{tab:q}');
    fprintf(fid,'\n%s','\end{table}');
    
    
    fprintf(fid,'\n%s','\begin{table}[htbp]');
    fprintf(fid,'\n%s','\centering');
    fprintf(fid,'\n%s','\begin{tabular}{|l|c|c|}');
    fprintf(fid,'\n%s','\hline\hline');
    fprintf(fid,'\n%s','\multicolumn{3}{c}{\textbf{Rigid}}\\\hline\hline');
    fprintf(fid,'\n%s',' & NeoCASS & Nastran\\\hline');
    fprintf(fid,'\n%s',['$C_{S_{r}}$ & ',num2str(RSD.R_rate.dcs_dR),'&',num2str(CC(2,label.yaw,1),'%4g'),'\\\hline']);
    fprintf(fid,'\n%s',['$C_{L_{r}}$ & ',num2str(RSD.R_rate.dcl_dR),'&',num2str(CC(3,label.yaw,1),'%4g'),'\\\hline']);
    fprintf(fid,'\n%s',['$C_{{\cal L}_{r}}$ & ',num2str(RSD.R_rate.dcml_dR),'&',num2str(CC(4,label.yaw,1),'%4g'),'\\\hline']);
    fprintf(fid,'\n%s',['$C_{{\cal M}_{r}}$ & ',num2str(RSD.R_rate.dcmm_dR),'&',num2str(CC(5,label.yaw,1),'%4g'),'\\\hline']);
    fprintf(fid,'\n%s',['$C_{{\cal N}_{r}}$ & ',num2str(RSD.R_rate.dcmn_dR),'&',num2str(CC(6,label.yaw,1),'%4g'),'\\\hline']);
    fprintf(fid,'\n%s','\end{tabular}');
    
    fprintf(fid,'\n%s','\hspace{10mm}');
    fprintf(fid,'\n%s','\begin{tabular}{|l|c|c}');
    fprintf(fid,'\n%s','\hline\hline');
    fprintf(fid,'\n%s','\multicolumn{3}{c}{\textbf{Deformable}}\\\hline\hline');
    fprintf(fid,'\n%s',' & NeoCASS & Nastran\\\hline');
    fprintf(fid,'\n%s',['$C_{S_{r}}$ & ',num2str(DSD.R_rate.dcs_dR),'&',num2str(CC(2,label.yaw,2),'%4g'),'\\\hline']);
    fprintf(fid,'\n%s',['$C_{L_{r}}$ & ',num2str(DSD.R_rate.dcl_dR),'&',num2str(CC(3,label.yaw,2),'%4g'),'\\\hline']);
    fprintf(fid,'\n%s',['$C_{{\cal L}_{r}}$ & ',num2str(DSD.R_rate.dcml_dR),'&',num2str(CC(4,label.yaw,2),'%4g'),'\\\hline']);
    fprintf(fid,'\n%s',['$C_{{\cal M}_{r}}$ & ',num2str(DSD.R_rate.dcmm_dR),'&',num2str(CC(5,label.yaw,2),'%4g'),'\\\hline']);
    fprintf(fid,'\n%s',['$C_{{\cal N}_{r}}$ & ',num2str(DSD.R_rate.dcmn_dR),'&',num2str(CC(6,label.yaw,2),'%4g'),'\\\hline']);
    fprintf(fid,'\n%s','\end{tabular}');
    fprintf(fid,'\n%s','\hspace{10mm}');
    fprintf(fid,'\n%s','\begin{tabular}{|l|c|c|}');
    fprintf(fid,'\n%s','\hline\hline');
    fprintf(fid,'\n%s','\multicolumn{3}{c}{\textbf{D/R}}\\\hline\hline');
    
    if abs(DSD.R_rate.dcs_dR/RSD.R_rate.dcs_dR)<2 && DSD.R_rate.dcs_dR/RSD.R_rate.dcs_dR> 1e-3
        fprintf(fid,'\n%s',['$C_{S_{r}}$ & ',num2str(DSD.R_rate.dcs_dR/RSD.R_rate.dcs_dR),'&',num2str(CC(2,label.yaw,2)/CC(2,label.yaw,1),'%4g'),'\\\hline']);
    else
        fprintf(fid,'\n%s','$C_{S_{r}}$ & // & //\\\hline');
    end
    if abs(DSD.R_rate.dcl_dR/RSD.R_rate.dcl_dR)<2 && abs(DSD.R_rate.dcl_dR/RSD.R_rate.dcl_dR)>1e-3
        fprintf(fid,'\n%s',['$C_{L_{r}}$ & ',num2str(DSD.R_rate.dcl_dR/RSD.R_rate.dcl_dR),'&',num2str(CC(3,label.yaw,2)/CC(3,label.yaw,1),'%4g'),'\\\hline']);
    else
        fprintf(fid,'\n%s','$C_{L_{r}}$ & // & //\\\hline');
    end
    if abs(DSD.R_rate.dcml_dR/RSD.R_rate.dcml_dR)<2 && abs(DSD.R_rate.dcml_dR/RSD.R_rate.dcml_dR)>1e-3
        fprintf(fid,'\n%s',['$C_{{\cal L}_{r}}$ & ',num2str(DSD.R_rate.dcml_dR/RSD.R_rate.dcml_dR),'&',num2str(CC(4,label.yaw,2)/CC(4,label.yaw,1),'%4g'),'\\\hline']);
    else
        fprintf(fid,'\n%s','$C_{{\cal L}_{r}}$ & //& // \\\hline');
    end
    if abs(DSD.R_rate.dcmm_dR/RSD.R_rate.dcmm_dR)<2 && abs(DSD.R_rate.dcmm_dR/RSD.R_rate.dcmm_dR)>1e-3
        fprintf(fid,'\n%s',['$C_{{\cal M}_{r}}$ & ',num2str(DSD.R_rate.dcmm_dR/RSD.R_rate.dcmm_dR),'&',num2str(CC(5,label.yaw,2)/CC(5,label.yaw,1),'%4g'),'\\\hline']);
    else
        fprintf(fid,'\n%s','$C_{{\cal M}_{r}}$ & //& // \\\hline');
    end
    if abs(DSD.R_rate.dcmn_dR/RSD.R_rate.dcmn_dR)<2 && abs(DSD.R_rate.dcmn_dR/RSD.R_rate.dcmn_dR)>1e-3
        fprintf(fid,'\n%s',['$C_{{\cal N}_{r}}$ & ',num2str(DSD.R_rate.dcmn_dR/RSD.R_rate.dcmn_dR),'&',num2str(CC(6,label.yaw,2)/CC(6,label.yaw,1),'%4g'),'\\\hline']);
    else
        fprintf(fid,'\n%s','$C_{{\cal N}_{r}}$ & //& // \\\hline');
    end
    fprintf(fid,'\n%s','\end{tabular}');
    
    fprintf(fid,'\n%s','\caption{$r$ derivatives}');
    fprintf(fid,'\n%s','\label{tab:r}');
    fprintf(fid,'\n%s','\end{table}');
    
    fprintf(fid,'\n%s','\chapter{Control Derivatives}');
    
    for i = 1:length(RSD.Control.dcl_dDelta)
        switch RSD.Control.Name{i}
            case 'flap1r'
                name = 'f1';
                incc = label.flap1;
            case 'flap2r'
                name = 'f2';
                incc = label.flap2;
            case 'aileronr'
                name = 'a';
                incc = label.aileron;
            case 'elev1r'
                name = 'e';
                incc = label.elev;
            case 'rudder1'
                name = 'dr';
                incc = label.rudder;
            case 'elevC1r'
                name = 'c';
                incc = label.elevC;
            otherwise
                name = RSD.Control.Name{i};
        end
        
        
        fprintf(fid,'\n%s','\begin{table}[htbp]');
        fprintf(fid,'\n%s','\centering');
        fprintf(fid,'\n%s','\begin{tabular}{|l|c|c|}');
        fprintf(fid,'\n%s','\hline\hline'); 
        fprintf(fid,'\n%s','\multicolumn{3}{c}{\textbf{Rigid}}\\\hline\hline');
        fprintf(fid,'\n%s',' & NeoCASS & Nastran\\\hline'); 
        fprintf(fid,'\n%s',['$C_{S_{\delta_{',name,'}}}$ & ',num2str(RSD.Control.dcs_dDelta(i)),'&',num2str(CC(2,incc,1),'%4g'),'\\\hline']);
        fprintf(fid,'\n%s',['$C_{L_{\delta_{',name,'}}}$ & ',num2str(RSD.Control.dcl_dDelta(i)),'&',num2str(CC(3,incc,1),'%4g'),'\\\hline']);
        fprintf(fid,'\n%s',['$C_{{\cal L}_{\delta_{',name,'}}}$ & ',num2str(RSD.Control.dcml_dDelta(i)),'&',num2str(CC(4,incc,1),'%4g'),'\\\hline']);
        fprintf(fid,'\n%s',['$C_{{\cal M}_{\delta_{',name,'}}}$ & ',num2str(RSD.Control.dcmm_dDelta(i)),'&',num2str(CC(5,incc,1),'%4g'),'\\\hline']);
        fprintf(fid,'\n%s',['$C_{{\cal N}_{\delta_{',name,'}}}$ & ',num2str(RSD.Control.dcmn_dDelta(i)),'&',num2str(CC(6,incc,1),'%4g'),'\\\hline']);
        fprintf(fid,'\n%s','\end{tabular}');
        
        fprintf(fid,'\n%s','\hspace{10mm}');
        
        fprintf(fid,'\n%s','\begin{tabular}{|l|c|c|}');
        fprintf(fid,'\n%s','\hline\hline');
        fprintf(fid,'\n%s','\multicolumn{3}{c}{\textbf{Deformable}}\\\hline\hline');
        fprintf(fid,'\n%s',' & NeoCASS & Nastran\\\hline');
        fprintf(fid,'\n%s',['$C_{S_{\delta_{',name,'}}}$ & ',num2str(DSD.Control.dcs_dDelta(i)),'&',num2str(CC(2,incc,2),'%4g'),'\\\hline']);
        fprintf(fid,'\n%s',['$C_{L_{\delta_{',name,'}}}$ & ',num2str(DSD.Control.dcl_dDelta(i)),'&',num2str(CC(3,incc,2),'%4g'),'\\\hline']);
        fprintf(fid,'\n%s',['$C_{{\cal L}_{\delta_{',name,'}}}$ & ',num2str(DSD.Control.dcml_dDelta(i)),'&',num2str(CC(4,incc,2),'%4g'),'\\\hline']);
        fprintf(fid,'\n%s',['$C_{{\cal M}_{\delta_{',name,'}}}$ & ',num2str(DSD.Control.dcmm_dDelta(i)),'&',num2str(CC(5,incc,2),'%4g'),'\\\hline']);
        fprintf(fid,'\n%s',['$C_{{\cal N}_{\delta_{',name,'}}}$ & ',num2str(DSD.Control.dcmn_dDelta(i)),'&',num2str(CC(6,incc,2),'%4g'),'\\\hline']);
        fprintf(fid,'\n%s','\end{tabular}');
        fprintf(fid,'\n%s','\hspace{10mm}');
        
        fprintf(fid,'\n%s','\begin{tabular}{|l|c|c|}');
        fprintf(fid,'\n%s','\hline\hline');
        fprintf(fid,'\n%s','\multicolumn{3}{c}{\textbf{D/R}}\\\hline\hline');
        fprintf(fid,'\n%s',' & NeoCASS & Nastran\\\hline');
        if abs(DSD.Control.dcs_dDelta(i)/RSD.Control.dcs_dDelta(i))<2 && abs(DSD.Control.dcs_dDelta(i)/RSD.Control.dcs_dDelta(i))>1e-3
            fprintf(fid,'\n%s',['$C_{S_{\delta_{',name,'}}}$ & ',num2str(DSD.Control.dcs_dDelta(i)/RSD.Control.dcs_dDelta(i)),'&',num2str(CC(2,incc,2)/CC(2,incc,1),'%4g'),'\\\hline']);
        else
            fprintf(fid,'\n%s',['$C_{S_{\delta_{',name,'}}}$ & //& //\\\hline']);
        end
        if abs(DSD.Control.dcl_dDelta(i)/RSD.Control.dcl_dDelta(i))<2 && abs(DSD.Control.dcl_dDelta(i)/RSD.Control.dcl_dDelta(i))>1e-3
            fprintf(fid,'\n%s',['$C_{L_{\delta_{',name,'}}}$ & ',num2str(DSD.Control.dcl_dDelta(i)/RSD.Control.dcl_dDelta(i)),'&',num2str(CC(3,incc,2)/CC(3,incc,1),'%4g'),'\\\hline']);
        else
            fprintf(fid,'\n%s',['$C_{L_{\delta_{',name,'}}}$ & //& // \\\hline']);
        end
        if abs(DSD.Control.dcml_dDelta(i)/RSD.Control.dcml_dDelta(i))<2 && abs(DSD.Control.dcml_dDelta(i)/RSD.Control.dcml_dDelta(i))>1e-3
            fprintf(fid,'\n%s',['$C_{{\cal L}_{\delta_{',name,'}}}$ & ',num2str(DSD.Control.dcml_dDelta(i)/RSD.Control.dcml_dDelta(i)),'&',num2str(CC(4,incc,2)/CC(4,incc,1),'%4g'),'\\\hline']);
        else
            fprintf(fid,'\n%s',['$C_{{\cal L}_{\delta_{',name,'}}}$ & //& // \\\hline']);
        end
        if abs(DSD.Control.dcmm_dDelta(i)/RSD.Control.dcmm_dDelta(i))<2 && abs(DSD.Control.dcmm_dDelta(i)/RSD.Control.dcmm_dDelta(i))>1e-3
            fprintf(fid,'\n%s',['$C_{{\cal M}_{\delta_{',name,'}}}$ & ',num2str(DSD.Control.dcmm_dDelta(i)/RSD.Control.dcmm_dDelta(i)),'&',num2str(CC(5,incc,2)/CC(5,incc,1),'%4g'),'\\\hline']);
        else
            fprintf(fid,'\n%s',['$C_{{\cal M}_{\delta_{',name,'}}}$ & //& //\\\hline']);
        end
        if abs(DSD.Control.dcmn_dDelta(i)/RSD.Control.dcmn_dDelta(i))<2 && abs(DSD.Control.dcmn_dDelta(i)/RSD.Control.dcmn_dDelta(i))>1e-3
            fprintf(fid,'\n%s',['$C_{{\cal N}_{\delta_{',name,'}}}$ & ',num2str(DSD.Control.dcmn_dDelta(i)/RSD.Control.dcmn_dDelta(i)),'&',num2str(CC(6,incc,2)/CC(6,incc,1),'%4g'),'\\\hline']);
        else
            fprintf(fid,'\n%s',['$C_{{\cal N}_{\delta_{',name,'}}}$ & //& // \\\hline']);
        end
        fprintf(fid,'\n%s','\end{tabular}');
        
        fprintf(fid,'\n%s',['\caption{$\delta_{',name,'}$ derivatives}']);
        fprintf(fid,'\n%s',['\label{tab:',name,'}']);
        fprintf(fid,'\n%s','\end{table}');
    end
    
%     fprintf(fid,'\n%s','\begin{table}[htbp]');
%     fprintf(fid,'\n%s','\centering');
%     fprintf(fid,'\n%s','\begin{tabular}{|l|c|}');
%     fprintf(fid,'\n%s','\hline\hline');
%     fprintf(fid,'\n%s','\multicolumn{3}{c}{\textbf{Rigid}}\\\hline\hline');
%     fprintf(fid,'\n%s',['$C_{S_{\delta_{f2}}}$ & ',num2str(RSD.Control.dcs_dDelta(2)),'\\\hline']);
%     fprintf(fid,'\n%s',['$C_{L_{\delta_{f2}}}$ & ',num2str(RSD.Control.dcl_dDelta(2)),'\\\hline']);
%     fprintf(fid,'\n%s',['$C_{{\cal L}_{\delta_{f2}}}$ & ',num2str(RSD.Control.dcml_dDelta(2)),'\\\hline']);
%     fprintf(fid,'\n%s',['$C_{{\cal M}_{\delta_{f2}}}$ & ',num2str(RSD.Control.dcmm_dDelta(2)),'\\\hline']);
%     fprintf(fid,'\n%s',['$C_{{\cal N}_{\delta_{f2}}}$ & ',num2str(RSD.Control.dcmn_dDelta(2)),'\\\hline']);
%     fprintf(fid,'\n%s','\end{tabular}');
%     fprintf(fid,'\n%s','\hspace{10mm}');
%     
%     fprintf(fid,'\n%s','\begin{tabular}{|l|c|}');
%     fprintf(fid,'\n%s','\hline\hline');
%     fprintf(fid,'\n%s','\multicolumn{3}{c}{\textbf{Deformable}}\\\hline\hline');
%     fprintf(fid,'\n%s',['$C_{S_{\delta_{f2}}}$ & ',num2str(DSD.Control.dcs_dDelta(2)),'\\\hline']);
%     fprintf(fid,'\n%s',['$C_{L_{\delta_{f2}}}$ & ',num2str(DSD.Control.dcl_dDelta(2)),'\\\hline']);
%     fprintf(fid,'\n%s',['$C_{{\cal L}_{\delta_{f2}}}$ & ',num2str(DSD.Control.dcml_dDelta(2)),'\\\hline']);
%     fprintf(fid,'\n%s',['$C_{{\cal M}_{\delta_{f2}}}$ & ',num2str(DSD.Control.dcmm_dDelta(2)),'\\\hline']);
%     fprintf(fid,'\n%s',['$C_{{\cal N}_{\delta_{f2}}}$ & ',num2str(DSD.Control.dcmn_dDelta(2)),'\\\hline']);
%     fprintf(fid,'\n%s','\end{tabular}');
%     fprintf(fid,'\n%s','\hspace{10mm}');
%     
%     fprintf(fid,'\n%s','\begin{tabular}{|l|c|}');
%     fprintf(fid,'\n%s','\hline\hline');
%     fprintf(fid,'\n%s','\multicolumn{3}{c}{\textbf{D/R}}\\\hline\hline');
%     if abs(DSD.Control.dcs_dDelta(2)/RSD.Control.dcs_dDelta(2))<2 && abs(DSD.Control.dcs_dDelta(2)/RSD.Control.dcs_dDelta(2))>1e-3
%         fprintf(fid,'\n%s',['$C_{S_{\delta_{f2}}}$ & ',num2str(DSD.Control.dcs_dDelta(2)/RSD.Control.dcs_dDelta(2)),'\\\hline']);
%     else
%         fprintf(fid,'\n%s','$C_{S_{\delta_{f2}}}$ & //\\\hline');
%     end
%     if abs(DSD.Control.dcl_dDelta(2)/RSD.Control.dcl_dDelta(2))<2 && abs(DSD.Control.dcl_dDelta(2)/RSD.Control.dcl_dDelta(2))>1e-3
%         fprintf(fid,'\n%s',['$C_{L_{\delta_{f2}}}$ & ',num2str(DSD.Control.dcl_dDelta(2)/RSD.Control.dcl_dDelta(2)),'\\\hline']);
%     else
%         fprintf(fid,'\n%s','$C_{L_{\delta_{f2}}}$ & // \\\hline');
%     end
%     if abs(DSD.Control.dcml_dDelta(2)/RSD.Control.dcml_dDelta(2))<2 && abs(DSD.Control.dcml_dDelta(2)/RSD.Control.dcml_dDelta(2))>1e-3
%         fprintf(fid,'\n%s',['$C_{{\cal L}_{\delta_{f2}}}$ & ',num2str(DSD.Control.dcml_dDelta(2)/RSD.Control.dcml_dDelta(2)),'\\\hline']);
%     else
%         fprintf(fid,'\n%s','$C_{{\cal L}_{\delta_{f2}}}$ & // \\\hline');
%     end
%     if abs(DSD.Control.dcmm_dDelta(2)/RSD.Control.dcmm_dDelta(2))<2 && abs(DSD.Control.dcmm_dDelta(2)/RSD.Control.dcmm_dDelta(2))>1e-3
%         fprintf(fid,'\n%s',['$C_{{\cal M}_{\delta_{f2}}}$ & ',num2str(DSD.Control.dcmm_dDelta(2)/RSD.Control.dcmm_dDelta(2)),'\\\hline']);
%     else
%         fprintf(fid,'\n%s','$C_{{\cal M}_{\delta_{f2}}}$ & //\\\hline');
%     end
%     if abs(DSD.Control.dcmn_dDelta(2)/RSD.Control.dcmn_dDelta(2))<2 && abs(DSD.Control.dcmn_dDelta(2)/RSD.Control.dcmn_dDelta(2))>1e-3
%         fprintf(fid,'\n%s',['$C_{{\cal N}_{\delta_{f2}}}$ & ',num2str(DSD.Control.dcmn_dDelta(2)/RSD.Control.dcmn_dDelta(2)),'\\\hline']);
%     else
%         fprintf(fid,'\n%s','$C_{{\cal N}_{\delta_{f2}}}$ & // \\\hline');
%     end
%     fprintf(fid,'\n%s','\end{tabular}');
%     fprintf(fid,'\n%s','\caption{$\delta_{f2}$ derivatives}');
%     fprintf(fid,'\n%s','\label{tab:d2}');
%     fprintf(fid,'\n%s','\end{table}');
%     
%     fprintf(fid,'\n%s','\begin{table}[htbp]');
%     fprintf(fid,'\n%s','\centering');
%     fprintf(fid,'\n%s','\begin{tabular}{|l|c|}');
%     fprintf(fid,'\n%s','\hline\hline');
%     fprintf(fid,'\n%s','\multicolumn{3}{c}{\textbf{Rigid}}\\\hline\hline');
%     fprintf(fid,'\n%s',['$C_{S_{\delta_{a}}}$ & ',num2str(RSD.Control.dcs_dDelta(3)),'\\\hline']);
%     fprintf(fid,'\n%s',['$C_{L_{\delta_{a}}}$ & ',num2str(RSD.Control.dcl_dDelta(3)),'\\\hline']);
%     fprintf(fid,'\n%s',['$C_{{\cal L}_{\delta_{a}}}$ & ',num2str(RSD.Control.dcml_dDelta(3)),'\\\hline']);
%     fprintf(fid,'\n%s',['$C_{{\cal M}_{\delta_{a}}}$ & ',num2str(RSD.Control.dcmm_dDelta(3)),'\\\hline']);
%     fprintf(fid,'\n%s',['$C_{{\cal N}_{\delta_{a}}}$ & ',num2str(RSD.Control.dcmn_dDelta(3)),'\\\hline']);
%     fprintf(fid,'\n%s','\end{tabular}');
%     fprintf(fid,'\n%s','\hspace{10mm}');
%     
%     fprintf(fid,'\n%s','\begin{tabular}{|l|c|}');
%     fprintf(fid,'\n%s','\hline\hline');
%     fprintf(fid,'\n%s','\multicolumn{3}{c}{\textbf{Deformable}}\\\hline\hline');
%     fprintf(fid,'\n%s',['$C_{S_{\delta_{a}}}$ & ',num2str(DSD.Control.dcs_dDelta(3)),'\\\hline']);
%     fprintf(fid,'\n%s',['$C_{L_{\delta_{a}}}$ & ',num2str(DSD.Control.dcl_dDelta(3)),'\\\hline']);
%     fprintf(fid,'\n%s',['$C_{{\cal L}_{\delta_{a}}}$ & ',num2str(DSD.Control.dcml_dDelta(3)),'\\\hline']);
%     fprintf(fid,'\n%s',['$C_{{\cal M}_{\delta_{a}}}$ & ',num2str(DSD.Control.dcmm_dDelta(3)),'\\\hline']);
%     fprintf(fid,'\n%s',['$C_{{\cal N}_{\delta_{a}}}$ & ',num2str(DSD.Control.dcmn_dDelta(3)),'\\\hline']);
%     fprintf(fid,'\n%s','\end{tabular}');
%     fprintf(fid,'\n%s','\hspace{10mm}');
%     
%     fprintf(fid,'\n%s','\begin{tabular}{|l|c|}');
%     fprintf(fid,'\n%s','\hline\hline');
%     fprintf(fid,'\n%s','\multicolumn{3}{c}{\textbf{D/R}}\\\hline\hline');
%     if abs(DSD.Control.dcs_dDelta(3)/RSD.Control.dcs_dDelta(3))<2 && abs(DSD.Control.dcs_dDelta(3)/RSD.Control.dcs_dDelta(3))>1e-3
%         fprintf(fid,'\n%s',['$C_{S_{\delta_{a}}}$ & ',num2str(DSD.Control.dcs_dDelta(3)/RSD.Control.dcs_dDelta(3)),'\\\hline']);
%     else
%         fprintf(fid,'\n%s','$C_{S_{\delta_{a}}}$ & //\\\hline');
%     end
%     if abs(DSD.Control.dcl_dDelta(3)/RSD.Control.dcl_dDelta(3))<2 && abs(DSD.Control.dcl_dDelta(3)/RSD.Control.dcl_dDelta(3))>1e-3
%         fprintf(fid,'\n%s',['$C_{L_{\delta_{a}}}$ & ',num2str(DSD.Control.dcl_dDelta(3)/RSD.Control.dcl_dDelta(3)),'\\\hline']);
%     else
%         fprintf(fid,'\n%s','$C_{L_{\delta_{a}}}$ & // \\\hline');
%     end
%     if abs(DSD.Control.dcml_dDelta(3)/RSD.Control.dcml_dDelta(3))<2 && abs(DSD.Control.dcml_dDelta(3)/RSD.Control.dcml_dDelta(3))>1e-3
%         fprintf(fid,'\n%s',['$C_{{\cal L}_{\delta_{a}}}$ & ',num2str(DSD.Control.dcml_dDelta(3)/RSD.Control.dcml_dDelta(3)),'\\\hline']);
%     else
%         fprintf(fid,'\n%s','$C_{{\cal L}_{\delta_{a}}}$ & // \\\hline');
%     end
%     if abs(DSD.Control.dcmm_dDelta(3)/RSD.Control.dcmm_dDelta(3))<2 && abs(DSD.Control.dcmm_dDelta(3)/RSD.Control.dcmm_dDelta(3))>1e-3
%         fprintf(fid,'\n%s',['$C_{{\cal M}_{\delta_{a}}}$ & ',num2str(DSD.Control.dcmm_dDelta(3)/RSD.Control.dcmm_dDelta(3)),'\\\hline']);
%     else
%         fprintf(fid,'\n%s','$C_{{\cal M}_{\delta_{a}}}$ & //\\\hline');
%     end
%     if abs(DSD.Control.dcmn_dDelta(3)/RSD.Control.dcmn_dDelta(3))<2 && abs(DSD.Control.dcmn_dDelta(3)/RSD.Control.dcmn_dDelta(3))>1e-3
%         fprintf(fid,'\n%s',['$C_{{\cal N}_{\delta_{a}}}$ & ',num2str(DSD.Control.dcmn_dDelta(3)/RSD.Control.dcmn_dDelta(3)),'\\\hline']);
%     else
%         fprintf(fid,'\n%s','$C_{{\cal N}_{\delta_{a}}}$ & // \\\hline');
%     end
%     fprintf(fid,'\n%s','\end{tabular}');
%     fprintf(fid,'\n%s','\caption{$\delta_{a}$ derivatives}');
%     fprintf(fid,'\n%s','\label{tab:da}');
%     fprintf(fid,'\n%s','\end{table}');
%     
%     
%     fprintf(fid,'\n%s','\begin{table}[htbp]');
%     fprintf(fid,'\n%s','\centering');
%     fprintf(fid,'\n%s','\begin{tabular}{|l|c|}');
%     fprintf(fid,'\n%s','\hline\hline');
%     fprintf(fid,'\n%s','\multicolumn{3}{c}{\textbf{Rigid}}\\\hline\hline');
%     fprintf(fid,'\n%s',['$C_{S_{\delta_{r}}}$ & ',num2str(RSD.Control.dcs_dDelta(4)),'\\\hline']);
%     fprintf(fid,'\n%s',['$C_{L_{\delta_{r}}}$ & ',num2str(RSD.Control.dcl_dDelta(4)),'\\\hline']);
%     fprintf(fid,'\n%s',['$C_{{\cal L}_{\delta_{r}}}$ & ',num2str(RSD.Control.dcml_dDelta(4)),'\\\hline']);
%     fprintf(fid,'\n%s',['$C_{{\cal M}_{\delta_{r}}}$ & ',num2str(RSD.Control.dcmm_dDelta(4)),'\\\hline']);
%     fprintf(fid,'\n%s',['$C_{{\cal N}_{\delta_{r}}}$ & ',num2str(RSD.Control.dcmn_dDelta(4)),'\\\hline']);
%     fprintf(fid,'\n%s','\end{tabular}');
%     fprintf(fid,'\n%s','\hspace{10mm}');
%     
%     fprintf(fid,'\n%s','\begin{tabular}{|l|c|}');
%     fprintf(fid,'\n%s','\hline\hline');
%     fprintf(fid,'\n%s','\multicolumn{3}{c}{\textbf{Deformable}}\\\hline\hline');
%     fprintf(fid,'\n%s',['$C_{S_{\delta_{r}}}$ & ',num2str(DSD.Control.dcs_dDelta(4)),'\\\hline']);
%     fprintf(fid,'\n%s',['$C_{L_{\delta_{r}}}$ & ',num2str(DSD.Control.dcl_dDelta(4)),'\\\hline']);
%     fprintf(fid,'\n%s',['$C_{{\cal L}_{\delta_{r}}}$ & ',num2str(DSD.Control.dcml_dDelta(4)),'\\\hline']);
%     fprintf(fid,'\n%s',['$C_{{\cal M}_{\delta_{r}}}$ & ',num2str(DSD.Control.dcmm_dDelta(4)),'\\\hline']);
%     fprintf(fid,'\n%s',['$C_{{\cal N}_{\delta_{r}}}$ & ',num2str(DSD.Control.dcmn_dDelta(4)),'\\\hline']);
%     fprintf(fid,'\n%s','\end{tabular}');
%   fprintf(fid,'\n%s','\hspace{10mm}');
%     
%     fprintf(fid,'\n%s','\begin{tabular}{|l|c|}');
%     fprintf(fid,'\n%s','\hline\hline');
%     fprintf(fid,'\n%s','\multicolumn{3}{c}{\textbf{D/R}}\\\hline\hline');
%     if abs(DSD.Control.dcs_dDelta(4)/RSD.Control.dcs_dDelta(4))<2 && abs(DSD.Control.dcs_dDelta(4)/RSD.Control.dcs_dDelta(4))>1e-3
%         fprintf(fid,'\n%s',['$C_{S_{\delta_{r}}}$ & ',num2str(DSD.Control.dcs_dDelta(4)/RSD.Control.dcs_dDelta(4)),'\\\hline']);
%     else
%         fprintf(fid,'\n%s','$C_{S_{\delta_{r}}}$ & //\\\hline');
%     end
%     if abs(DSD.Control.dcl_dDelta(4)/RSD.Control.dcl_dDelta(4))<2 && abs(DSD.Control.dcl_dDelta(4)/RSD.Control.dcl_dDelta(4))>1e-3
%         fprintf(fid,'\n%s',['$C_{L_{\delta_{r}}}$ & ',num2str(DSD.Control.dcl_dDelta(4)/RSD.Control.dcl_dDelta(4)),'\\\hline']);
%     else
%         fprintf(fid,'\n%s','$C_{L_{\delta_{r}}}$ & // \\\hline');
%     end
%     if abs(DSD.Control.dcml_dDelta(4)/RSD.Control.dcml_dDelta(4))<2 && abs(DSD.Control.dcml_dDelta(4)/RSD.Control.dcml_dDelta(4))>1e-3
%         fprintf(fid,'\n%s',['$C_{{\cal L}_{\delta_{r}}}$ & ',num2str(DSD.Control.dcml_dDelta(4)/RSD.Control.dcml_dDelta(4)),'\\\hline']);
%     else
%         fprintf(fid,'\n%s','$C_{{\cal L}_{\delta_{r}}}$ & // \\\hline');
%     end
%     if abs(DSD.Control.dcmm_dDelta(4)/RSD.Control.dcmm_dDelta(4))<2 && abs(DSD.Control.dcmm_dDelta(4)/RSD.Control.dcmm_dDelta(4))>1e-3
%         fprintf(fid,'\n%s',['$C_{{\cal M}_{\delta_{r}}}$ & ',num2str(DSD.Control.dcmm_dDelta(4)/RSD.Control.dcmm_dDelta(4)),'\\\hline']);
%     else
%         fprintf(fid,'\n%s','$C_{{\cal M}_{\delta_{r}}}$ & //\\\hline');
%     end
%     if abs(DSD.Control.dcmn_dDelta(4)/RSD.Control.dcmn_dDelta(4))<2 && abs(DSD.Control.dcmn_dDelta(4)/RSD.Control.dcmn_dDelta(4))>1e-3
%         fprintf(fid,'\n%s',['$C_{{\cal N}_{\delta_{r}}}$ & ',num2str(DSD.Control.dcmn_dDelta(4)/RSD.Control.dcmn_dDelta(4)),'\\\hline']);
%     else
%         fprintf(fid,'\n%s','$C_{{\cal N}_{\delta_{r}}}$ & // \\\hline');
%     end
%     fprintf(fid,'\n%s','\end{tabular}');
%     fprintf(fid,'\n%s','\caption{$\delta_{r}$ derivatives}');
%     fprintf(fid,'\n%s','\label{tab:dr}');
%     fprintf(fid,'\n%s','\end{table}');
%     
%     
%     fprintf(fid,'\n%s','\begin{table}[htbp]');
%     fprintf(fid,'\n%s','\centering');
%     fprintf(fid,'\n%s','\begin{tabular}{|l|c|}');
%     fprintf(fid,'\n%s','\hline\hline');
%     fprintf(fid,'\n%s','\multicolumn{3}{c}{\textbf{Rigid}}\\\hline\hline');
%     fprintf(fid,'\n%s',['$C_{S_{\delta_{e}}}$ & ',num2str(RSD.Control.dcs_dDelta(5)),'\\\hline']);
%     fprintf(fid,'\n%s',['$C_{L_{\delta_{e}}}$ & ',num2str(RSD.Control.dcl_dDelta(5)),'\\\hline']);
%     fprintf(fid,'\n%s',['$C_{{\cal L}_{\delta_{e}}}$ & ',num2str(RSD.Control.dcml_dDelta(5)),'\\\hline']);
%     fprintf(fid,'\n%s',['$C_{{\cal M}_{\delta_{e}}}$ & ',num2str(RSD.Control.dcmm_dDelta(5)),'\\\hline']);
%     fprintf(fid,'\n%s',['$C_{{\cal N}_{\delta_{e}}}$ & ',num2str(RSD.Control.dcmn_dDelta(5)),'\\\hline']);
%     fprintf(fid,'\n%s','\end{tabular}');
%     fprintf(fid,'\n%s','\hspace{10mm}');
%     
%     fprintf(fid,'\n%s','\begin{tabular}{|l|c|}');
%     fprintf(fid,'\n%s','\hline\hline');
%     fprintf(fid,'\n%s','\multicolumn{3}{c}{\textbf{Deformable}}\\\hline\hline');
%     fprintf(fid,'\n%s',['$C_{S_{\delta_{e}}}$ & ',num2str(DSD.Control.dcs_dDelta(5)),'\\\hline']);
%     fprintf(fid,'\n%s',['$C_{L_{\delta_{e}}}$ & ',num2str(DSD.Control.dcl_dDelta(5)),'\\\hline']);
%     fprintf(fid,'\n%s',['$C_{{\cal L}_{\delta_{e}}}$ & ',num2str(DSD.Control.dcml_dDelta(5)),'\\\hline']);
%     fprintf(fid,'\n%s',['$C_{{\cal M}_{\delta_{e}}}$ & ',num2str(DSD.Control.dcmm_dDelta(5)),'\\\hline']);
%     fprintf(fid,'\n%s',['$C_{{\cal N}_{\delta_{e}}}$ & ',num2str(DSD.Control.dcmn_dDelta(5)),'\\\hline']);
%     fprintf(fid,'\n%s','\end{tabular}');
%   fprintf(fid,'\n%s','\hspace{10mm}');
%     
%     fprintf(fid,'\n%s','\begin{tabular}{|l|c|}');
%     fprintf(fid,'\n%s','\hline\hline');
%     fprintf(fid,'\n%s','\multicolumn{3}{c}{\textbf{D/R}}\\\hline\hline');
%     if abs(DSD.Control.dcs_dDelta(5)/RSD.Control.dcs_dDelta(5))<2 && abs(DSD.Control.dcs_dDelta(5)/RSD.Control.dcs_dDelta(5))>1e-3
%         fprintf(fid,'\n%s',['$C_{S_{\delta_{e}}}$ & ',num2str(DSD.Control.dcs_dDelta(5)/RSD.Control.dcs_dDelta(5)),'\\\hline']);
%     else
%         fprintf(fid,'\n%s','$C_{S_{\delta_{e}}}$ & // \\\hline');
%     end
%     if abs(DSD.Control.dcl_dDelta(5)/RSD.Control.dcl_dDelta(5))<2 && abs(DSD.Control.dcl_dDelta(5)/RSD.Control.dcl_dDelta(5))>1e-3
%         fprintf(fid,'\n%s',['$C_{L_{\delta_{e}}}$ & ',num2str(DSD.Control.dcl_dDelta(5)/RSD.Control.dcl_dDelta(5)),'\\\hline']);
%     else
%         fprintf(fid,'\n%s','$C_{L_{\delta_{e}}}$ & // \\\hline');
%     end
%     if abs(DSD.Control.dcml_dDelta(5)/RSD.Control.dcml_dDelta(5))<2 && abs(DSD.Control.dcml_dDelta(5)/RSD.Control.dcml_dDelta(5))>1e-3
%         fprintf(fid,'\n%s',['$C_{{\cal L}_{\delta_{e}}}$ & ',num2str(DSD.Control.dcml_dDelta(5)/RSD.Control.dcml_dDelta(5)),'\\\hline']);
%     else
%         fprintf(fid,'\n%s','$C_{{\cal L}_{\delta_{e}}}$ & // \\\hline');
%     end
%     if abs(DSD.Control.dcmm_dDelta(5)/RSD.Control.dcmm_dDelta(5))<2 && abs(DSD.Control.dcmm_dDelta(5)/RSD.Control.dcmm_dDelta(5))>1e-3
%         fprintf(fid,'\n%s',['$C_{{\cal M}_{\delta_{e}}}$ & ',num2str(DSD.Control.dcmm_dDelta(5)/RSD.Control.dcmm_dDelta(5)),'\\\hline']);
%     else
%         fprintf(fid,'\n%s','$C_{{\cal M}_{\delta_{e}}}$ & //\\\hline');
%     end
%     if abs(DSD.Control.dcmn_dDelta(5)/RSD.Control.dcmn_dDelta(5))<2 && abs(DSD.Control.dcmn_dDelta(5)/RSD.Control.dcmn_dDelta(5))>1e-3
%         fprintf(fid,'\n%s',['$C_{{\cal N}_{\delta_{e}}}$ & ',num2str(DSD.Control.dcmn_dDelta(5)/RSD.Control.dcmn_dDelta(5)),'\\\hline']);
%     else
%         fprintf(fid,'\n%s','$C_{{\cal N}_{\delta_{e}}}$ & // \\\hline');
%     end
%     fprintf(fid,'\n%s','\end{tabular}');
%     fprintf(fid,'\n%s','\caption{$\delta_{e}$ derivatives}');
%     fprintf(fid,'\n%s','\label{tab:de}');
%     fprintf(fid,'\n%s','\end{table}');
    
    
    fprintf(fid,'\n%s','\end{document}');
    fclose(fid);
    system(['texi2dvi ',file_name,'.tex']);
    system(['dvipdfmx ',file_name,'.dvi']);
end