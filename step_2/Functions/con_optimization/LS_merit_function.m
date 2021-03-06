function [xkp1,fxkp1,n_iter,tk] = LS_merit_function( fun,fxk,DT1k,xk,pk,tkmax,beta,c,nitermax )
% LS_merit_function Computes the value of decision variables xkp1 at iteration k+1
% by carrying out a back-tracking line search approach, trying to achieve a
% sufficient decrease of the merit function T1(xk+tk*pk) where pk is the
% search direction.
%
%   INPUTS:
%           fun         =   merit function
%           fxk         =   merit function evaluated at xk
%           DT1k        =   directional derivative of the merit function
%           xk          =   value of the decision variables at the current
%                           iterate
%           pk          =   search direction
%           tkmax       =   maximum step size
%           beta        =   ratio tk_ip1/tk_i during the back-tracking line
%                           search (beta in (0,1))
%           c           =   ratio between acheived decrease and decrease
%                           predicted by the first-order Taylor expansion,
%                           sufficient to exit the back-tracking line
%                           search algorithm
%           nitermax    =   maximum number of iterations
%
%   OUTPUTS:
%           xkp1        =   obtained value of xk+tk*pk
%           fxkp1       =   cost function evaluated at xkp1
%           niter       =   number of iterations employed to satisfy Armijo
%                           condition
%           tk          =   step length

n_iter          =   0;
tk              =   tkmax;
xkp1            =   xk+tk*pk;
fxkp1           =   fun(xkp1);

while (isnan( fxkp1 ) || ( fxkp1 > fxk+tk*c*DT1k && n_iter < nitermax ))
    
    tk          =   beta*tk;
    xkp1        =   xk+tk*pk;
    fxkp1       =   fun(xkp1);
    n_iter      =   n_iter+1;
    
end %End of the while

end %End of the function

