%***********************************************************************
%  MASSA Modern Aeroservoelastic State-Space Analysis
%  Copyright (c) 2000-2004 
%                     Giuseppe Quaranta <quaranta@aero.polimi.it>
%                     Paolo Mantegazza  <mantegazza@aero.polimi.it>
%  Dipartimento di Ingegneria Aerospaziale - Politecnico di Milano
%***********************************************************************
%
%   MS_PADE
%   Pade approximation of a matricial transfer function
%
%   Hmg(j omega) =( I  + Sum E_i (j omega)^i )^-1 * ( N_0 + N_1 j omega +
%       - N_2 omega^2 + Sum N_(i+3) (j omega)^(i+3)) 
%   The Order is the equivalent to the number of D matrices
%   Data are known at the different frequency included in the Klist
%   A state space model is the result of the approximation 
%   process
%   The flag is used to inidicate which of the two forms is required
%   flag = 0
%   Hmg(j omega) =( I  + Sum E_i (j omega)^i )^-1 * ( N_0 + N_1 j omega +
%       - N_2 omega^2 + Sum N_(i+3) (j omega)^(i+3)) 
%   flag = 1
%   Hmg(j omega) =( I (j omega)^n  + Sum E_i (j omega)^i )^-1 * ( N_0 + N_1 j omega +
%       - N_2 omega^2 + Sum N_(i+3) (j omega)^(i+3)) 

    function [D0,D1,D2,A,B,C] = ms_pade(Hmg,Order,Klist,KW,flag)
    
    dim  = size(Hmg);
    nk = length(Klist);
    
    if (size(Klist,1) == 1)
        Klist = Klist';
    end
    if (size(KW,1) == 1)
        KW = KW';
    end

        
    A = zeros(dim(1)*Order,dim(1)*Order);
    D0 = zeros(dim(1),dim(2));
    D1 = zeros(dim(1),dim(2));
    D2 = zeros(dim(1),dim(2));
    B  = zeros(dim(1)*Order,dim(2));
    C  = zeros(dim(1),dim(1)*Order);
    
    % calcola i coefficienti uno per uno
    %M = sparse(dim(1)*dim(2)*2*(nk-1),dim(1)*dim(2)*(Order+2)+dim(1)*dim(1)*(Order));
    X = zeros(dim(1)*dim(2)*(Order+2)+dim(1)*dim(1)*(Order),1);
    F = zeros(dim(1)*dim(2)*2*(nk-1),1);
    % nelle matrici ci saranno prima i coefficienti delle matrici 
    % N_i e poi quelli delle matrici E_i in ordine per riga
    % per ogni coefficiente si scrivono tante equazioni quante sono 
    % le freuqnze ridotte, una per la perte reale ed una per la parte
    % immaginaria
    
    % N_0 viene imposto uguale ad H(0)
    % matrici E
    N = zeros(dim(1),dim(2),Order+3);
    E = zeros(dim(1),dim(1),Order+1);
    
    if (flag == 1)
        E(:,:,Order+1) = eye(dim(1)); 
    else
        E(:,:,1) = eye(dim(1));
    end
    Ho = reshape(Hmg(:,:,1),dim(1),dim(2));
    Mx_r = zeros(2*(nk-1)*((2+Order)*dim(1)*dim(2)+dim(1)*dim(2)*(Order*dim(1))),1);
    Mx_c = zeros(2*(nk-1)*((2+Order)*dim(1)*dim(2)+dim(1)*dim(2)*(Order*dim(1))),1);
    Mx   = zeros(2*(nk-1)*((2+Order)*dim(1)*dim(2)+dim(1)*dim(2)*(Order*dim(1))),1);
    %Mx =sparse([],[],[],dim(1)*dim(2),dim(1)*dim(2)*(Order+2)+dim(1)*dim(1)*(Order),...
    %    (2+Order)*dim(1)*dim(2)+dim(1)*dim(2)*(Order*dim(1))); 
    %sparse(dim(1)*dim(2),dim(1)*dim(2)*(Order+2)+dim(1)*dim(1)*(Order)); 
        % attenzione il sistema viene creato in modo che per le matrici N
        % le variabili siano nell'ordine per colonna N11 N21 ... Nn1 N12
        % ... mentre per le matrici E i coefficienti sono in ordine per
        % riga E11 E12 E13 ... E21 ...
        % questo per comodita di scrittura della matrice dei coefficienti M
    if (flag == 1)
        q = 1;
        for k=2:nk
            Hl = reshape(Hmg(:,:,k),dim(1),dim(2));
            Hh = KW(k) * transpose(Ho - Hl);
            Hlt=transpose(Hl);
            for m=1:dim(2)
                for n=1:dim(1)
                    for h = 1:2+Order
                        Mx_r(q) = (2*k-4)*dim(1)*dim(2)+(m-1)*dim(1)+n;
                        Mx_c(q) = (h-1)*dim(1)*dim(2)+(m-1)*dim(1)+n;
                        t_val = (KW(k) * (complex(0,Klist(k)))^h);
                        Mx (q) = real(t_val);
                        q = q + 1;
                        Mx_r(q) = (k-2)*dim(1)*dim(2)*2+dim(1)*dim(2)+(m-1)*dim(1)+n;
                        Mx_c(q) = (h-1)*dim(1)*dim(2)+(m-1)*dim(1)+n;
                        Mx (q) = imag(t_val);
                        q = q + 1;
                    end
                    Mx_r(q:q+dim(1)-1) = ((2*k-4)*dim(1)*dim(2)+(m-1)*dim(1)+n)*ones(dim(1),1);
                    Mx_c(q:q+dim(1)-1) = ((Order+2)*dim(1)*dim(2)+(n-1)*dim(1)+1:...
                                          (Order+2)*dim(1)*dim(2)+(n-1)*dim(1)+dim(1))';
                    Mx  (q:q+dim(1)-1) = real(Hh(m,:));
                    q = q +dim(1);
                    Mx_r(q:q+dim(1)-1) = ((k-2)*dim(1)*dim(2)*2+dim(1)*dim(2)+(m-1)*dim(1)+n)*ones(dim(1),1);
                    Mx_c(q:q+dim(1)-1) = ((Order+2)*dim(1)*dim(2)+(n-1)*dim(1)+1:...
                                          (Order+2)*dim(1)*dim(2)+(n-1)*dim(1)+dim(1))';
                    Mx  (q:q+dim(1)-1) = imag(Hh(m,:));
                    q = q +dim(1);
                    for h = 2:Order
                        t_val = -KW(k) * complex(0,Klist(k))^(h-1) * transpose(Hl(:,m));
                        Mx_r(q:q+dim(1)-1) = ((2*k-4)*dim(1)*dim(2)+(m-1)*dim(1)+n)*ones(dim(1),1);
                        Mx_c(q:q+dim(1)-1) = ((Order+2)*dim(1)*dim(2)+(h-1)*dim(1)^2+(n-1)*dim(1)+1:...
                                        (Order+2)*dim(1)*dim(2)+(h-1)*dim(1)^2+(n-1)*dim(1)+dim(1))';
                        Mx  (q:q+dim(1)-1) = real(t_val);
                        q = q +dim(1);
                        Mx_r(q:q+dim(1)-1) = ((k-2)*dim(1)*dim(2)*2+dim(1)*dim(2)+(m-1)*dim(1)+n)*ones(dim(1),1);
                        Mx_c(q:q+dim(1)-1) = ((Order+2)*dim(1)*dim(2)+(h-1)*dim(1)^2+(n-1)*dim(1)+1:...
                                        (Order+2)*dim(1)*dim(2)+(h-1)*dim(1)^2+(n-1)*dim(1)+dim(1))';
                        Mx  (q:q+dim(1)-1) = imag(t_val);
                        q = q +dim(1);                        
                    end
                    Fx((m-1)*dim(1)+n,1) = KW(k) * complex(0,Klist(k))^(Order) * Hlt(m,n);
                end
            end
            F((k-2)*dim(1)*dim(2)*2+1:(k-2)*dim(1)*dim(2)*2+dim(1)*dim(2),:) = real(Fx);
            F((k-2)*dim(1)*dim(2)*2+dim(1)*dim(2)+1:(k-2)*dim(1)*dim(2)*2+2*dim(1)*dim(2),:) = imag(Fx);
        end
        M = sparse(Mx_r, Mx_c, Mx, dim(1)*dim(2)*2*(nk-1),dim(1)*dim(2)*(Order+2)+dim(1)^2*(Order));
        clear Mx_c Mx_r Mx;
        X = M\F;
        for h = 1:2+Order
            N(:,:,h+1) = reshape(X((h-1)*dim(1)*dim(2)+1:h*dim(1)*dim(2),1),dim(1),dim(2));
        end
        for h = 1:Order
            E(:,:,h) =  reshape(X((Order+2)*dim(1)*dim(2)+(h-1)*dim(1)^2+1:...
             (Order+2)*dim(1)*dim(2)+h*dim(1)^2,1),dim(1),dim(1))';
        end
        N(:,:,1) = E(:,:,1)*real(Ho);
    else
        q = 1;
        for k=2:nk
            Hl = reshape(Hmg(:,:,k),dim(1),dim(2));
            Hh = KW(k) * transpose(Hl - Ho);
            for m=1:dim(2)
                for n=1:dim(1)
                    for h = 1:2+Order
                        Mx_r(q) = (2*k-4)*dim(1)*dim(2)+(m-1)*dim(1)+n;
                        Mx_c(q) = (h-1)*dim(1)*dim(2)+(m-1)*dim(1)+n;
                        t_val = (KW(k) * (complex(0,Klist(k)))^h);
                        Mx (q) = real(t_val);
                        q = q + 1;
                        Mx_r(q) = (k-2)*dim(1)*dim(2)*2+dim(1)*dim(2)+(m-1)*dim(1)+n;
                        Mx_c(q) = (h-1)*dim(1)*dim(2)+(m-1)*dim(1)+n;
                        Mx (q) = imag(t_val);
                        q = q + 1;
                    end
                    for h = 1:Order
                        t_val = - KW(k) * (complex(0,Klist(k)))^h * transpose(Hl(:,m));
                        Mx_r(q:q+dim(1)-1) = ((2*k-4)*dim(1)*dim(2)+(m-1)*dim(1)+n)*ones(dim(1),1);
                        Mx_c(q:q+dim(1)-1) = ((Order+2)*dim(1)*dim(2)+(h-1)*dim(1)^2+(n-1)*dim(1)+1:...
                                        (Order+2)*dim(1)*dim(2)+(h-1)*dim(1)^2+(n-1)*dim(1)+dim(1))';
                        Mx  (q:q+dim(1)-1) = real(t_val);
                        q = q +dim(1);
                        Mx_r(q:q+dim(1)-1) = ((k-2)*dim(1)*dim(2)*2+dim(1)*dim(2)+(m-1)*dim(1)+n)*ones(dim(1),1);
                        Mx_c(q:q+dim(1)-1) = ((Order+2)*dim(1)*dim(2)+(h-1)*dim(1)^2+(n-1)*dim(1)+1:...
                                        (Order+2)*dim(1)*dim(2)+(h-1)*dim(1)^2+(n-1)*dim(1)+dim(1))';
                        Mx  (q:q+dim(1)-1) = imag(t_val);
                        q = q +dim(1);
                    end                            
                    Fx((m-1)*dim(1)+n,1) = Hh(m,n); 
                end
            end
            F((k-2)*dim(1)*dim(2)*2+1:(k-2)*dim(1)*dim(2)*2+dim(1)*dim(2),:) = real(Fx);
            F((k-2)*dim(1)*dim(2)*2+dim(1)*dim(2)+1:(k-2)*dim(1)*dim(2)*2+2*dim(1)*dim(2),:) = imag(Fx);
        end
        M = sparse(Mx_r, Mx_c, Mx, dim(1)*dim(2)*2*(nk-1),dim(1)*dim(2)*(Order+2)+dim(1)^2*(Order));
        %figure(1);
        %spy(M)
        %pause
        clear Mx_c Mx_r Mx;
        X = M\F;
        N(:,:,1) = real(Ho);
        for h = 1:2+Order
            N(:,:,h+1) = (reshape(X((h-1)*dim(1)*dim(2)+1:h*dim(1)*dim(2),1),dim(1),dim(2)));
        end
        for h = 1:Order
            E(:,:,h+1) =  (reshape(X((Order+2)*dim(1)*dim(2)+(h-1)*dim(1)^2+1:...
                    (Order+2)*dim(1)*dim(2)+h*dim(1)^2,1),dim(1),dim(1)))';
        end
    end    
    % dati D ed N devo trovare ABCD
    D2 = reshape(E(:,:,Order+1),dim(1),dim(1))\reshape(N(:,:,Order+3),dim(1),dim(2));
    for i=1:Order
        N(:,:,Order+3-i) = N(:,:,Order+3-i) - E(:,:,Order+1-i)*D2;
    end
    D1 = reshape(E(:,:,Order+1),dim(1),dim(1))\reshape(N(:,:,Order+2),dim(1),dim(2));
    for i=1:Order
        N(:,:,Order+2-i) = N(:,:,Order+2-i) - E(:,:,Order+1-i)*D1;
    end
    D0 = reshape(E(:,:,Order+1),dim(1),dim(1))\reshape(N(:,:,Order+1),dim(1),dim(2));
    for i=1:Order
        N(:,:,Order+1-i) = N(:,:,Order+1-i) - E(:,:,Order+1-i)*D0;
    end
    
    Efi = inv(reshape(E(:,:,Order+1), dim(1),dim(1)));
    for i = 1:Order
        B(dim(1)*(i-1)+1:i*dim(1),:) =Efi*reshape(N(:,:,Order+1-i),dim(1),dim(2));
    end
    for i = 1:Order-1
        A((i-1)*dim(1)+1:i*dim(1),1:dim(1)) = - Efi*reshape(E(:,:,Order+1-i), dim(1),dim(1));
        A((i-1)*dim(1)+1:i*dim(1),i*dim(1)+1:(i+1)*dim(1))=eye(dim(1));
    end
    A((Order-1)*dim(1)+1:Order*dim(1),1:dim(1)) = - Efi*reshape(E(:,:,1), dim(1),dim(1));
    C(:,1:dim(1)) = eye(dim(1));
    disp('Padé model creation completed');
    