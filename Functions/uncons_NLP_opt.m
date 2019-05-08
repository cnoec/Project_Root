function [x_opt, fx_opt, n_iter, exit_flag] = uncons_NLP_opt(fnc, x0, options)
% Implmentation of NLP unconstrained problem

%%  common initialization

delta_x = 1;
delta_f = 1;
n = length(x0);
n_iter=0;

%% BFGS initialization

x_k=x0;
gamma = options.BFGS_gamma;
[fx,grad] = gradient_num_comp(fnc, x_k, options.gradmethod, options.graddx);
H_k=eye(n)*1e-4;
p_k = -H_k\grad;

%% algorithm
% [~, ~, ~] = linesearch(@(x)fnc(x), grad, x_k, p_k, options.ls_tkmax, options.ls_beta, options.ls_c, options.ls_nitermax);

while(abs(grad'*p_k) > options.tolgrad && delta_x > options.tolx && delta_f > options.tolfun && n_iter < options.ls_nitermax)
    
    % step 4: linesearch
    
    n_iter=1+n_iter;
    [x_kp1, fx_kp1, ~] = linesearch(fnc, grad, x_k, p_k, options.ls_tkmax, options.ls_beta, options.ls_c, options.ls_nitermax);           
    
    % step 5: Hessian approximation: 
    
    [~,grad_kp1] =  gradient_num_comp(fnc, x_kp1, options.gradmethod, options.graddx);
    y = (grad_kp1 - grad);
    s = (x_kp1 - x_k);
    
    %           Powell trick to enforce positivity
    
    if y.'*s <= gamma*s.'*H_k*s
        lambda = (gamma*s.'*H_k*s - s.'*y)/(s.'*H_k*s-s.'*y);
        y = y + lambda*(H_k*s-y);
    end
    
    H_k = H_k - H_k*s*(s.')*H_k/((s.')*H_k*s) + (y*(y.'))/((s.')*y);
        
    % step 6: tolerance update
    delta_x = norm(x_kp1 - x_k,2)/max(eps, norm(x_k,2));
    delta_f = norm(fx_kp1 - fx)/max(eps, norm(fx,2));
    
    % step 7: updating
    x_k = x_kp1;
    fx = fx_kp1;
    
    grad = grad_kp1;
    p_k = -H_k\grad;
    
    
end

%% results
        x_opt = x_k;
        fx_opt=fnc(x_opt);



