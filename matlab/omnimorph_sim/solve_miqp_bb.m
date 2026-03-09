function [u_best, y_best, cost_best, flag] = solve_miqp_bb(w,A,Q,u_underbar,u_bar,u_init,y_init)
% Branch & Bound MIQP solver (safe, active-set, warm-started)
% Solves:
%   min (1/2) u' * Q * u
%   s.t. A*u = w
%        u_underbar*y - u_bar*(1-y) <= u <= u_bar*y - u_underbar*(1-y)
%        y in {0,1}^N
%
% Inputs:
%   w, A, Q, u_underbar, u_bar
%   u_init (N x 1) initial guess for u (optional, pass [] to default)
%   y_init (N x 1) initial guess for y (optional, pass [] to default)
%
% Outputs:
%   u_best, y_best : best integer-feasible solution found (empty if none)
%   cost_best      : cost of u_best (Inf if none)
%   flag           : 1 if integer-feasible solution found, 0 otherwise
%
% Notes:
% - Uses quadprog (active-set). Make sure Optimization Toolbox is available.

% dimensions
N = size(A,2);

% defaults for initial guesses
if nargin < 6 || isempty(u_init)
    u_init = zeros(N,1);
end
if nargin < 7 || isempty(y_init)
    y_init = 0.5*ones(N,1); % fractional warm start
end

% clip initial sizes
u_init = u_init(:);
y_init = y_init(:);
if length(u_init)~=N || length(y_init)~=N
    error('u_init and y_init must be length N.');
end

% outputs / best-so-far
cost_best = Inf;
u_best = [];
y_best = [];
flag = 0;

% initial bounds on y (relaxed)
y_lb0 = zeros(N,1);
y_ub0 = ones(N,1);

% quadprog options (active-set)
options = optimoptions('quadprog', ...
                       'Algorithm','active-set', ...
                       'Display','off');

% start recursion with depth = 0
maxDepth = N + 5; % safe guard
branch_and_bound(y_lb0, y_ub0, u_init, y_init, 0);

% set flag
if ~isempty(u_best)
    flag = 1;
else
    disp('unfeasible');
    cost_best = Inf; u_best = []; y_best = []; flag = 0;
end

% ---------------- nested functions ----------------
    function branch_and_bound(y_lb, y_ub, u_start, y_start, depth)
        % Guard recursion depth
        if depth > maxDepth
            warning('max depth reached');
            return;
        end

        % If all y are fixed by bounds: Check if it reached a leaf node
        if all(abs(y_lb - y_ub) < 1e-12)
            y_fixed = round(y_lb); % integral
            % bounds for u given y_fixed
            lb_u = u_underbar*y_fixed - u_bar*(1 - y_fixed);
            ub_u = u_bar*y_fixed - u_underbar*(1 - y_fixed);

            % clip initial u
            u0 = max(lb_u, min(ub_u, u_start));

            % solve QP in u only
            [u_node, fval_u, exitflag_u] = quadprog(Q, zeros(N,1), ...
                [],[], A, w, lb_u, ub_u, u0, options);

            if exitflag_u ~= 1
                return;
            end

            % fval_u is objective value = 0.5*u'*Q*u
            if fval_u + 1e-12 < cost_best
                cost_best = fval_u;
                u_best = u_node;
                y_best = y_fixed;
            end
            return;
        end

        % Solve relaxed QP in (u,y)
        [u_relax, y_relax, cost_relax, exitflag_relax] = solve_relaxed(y_lb, y_ub, u_start, y_start);
        if exitflag_relax ~= 1
            return; % infeasible node
        end

        % prune if bound not better than best
        if cost_relax + 1e-12 >= cost_best 
            return;
        end

        % if all y integral within tolerance, becomes a leaf node -> update best or prune
        if all(abs(y_relax - round(y_relax)) < 1e-6)
            y_int = round(y_relax);
            if cost_relax + 1e-12 < cost_best 
                cost_best = cost_relax;
                u_best = u_relax;
                y_best = y_int;
            end
            return; %prune
        end

        % pick index to branch: among unfixed variables choose most fractional
        unfixed = find(y_lb + 1e-12 < y_ub); % indices that are not fixed by bounds
        if isempty(unfixed)
            return;
        end
        fractional = abs(y_relax(unfixed) - round(y_relax(unfixed)));
        [~, imax] = max(fractional);
        idx = unfixed(imax);

        % Extra guard: ensure idx isn't already fixed by numerical noise
        if y_lb(idx) == y_ub(idx)
            return;
        end

        % Branch y_idx = 0
        y_lb0 = y_lb; y_ub0 = y_ub;
        y_lb0(idx) = 0; y_ub0(idx) = 0;
        % warm start for children: use relaxed solution but clipped to new bounds
        z_u0 = u_relax;
        z_y0 = y_relax;
        z_y0(idx) = 0;
        % clip
        z_y0 = max(y_lb0, min(y_ub0, z_y0));
        [~,~,cost0,exit0] = solve_relaxed(y_lb0, y_ub0, z_u0, z_y0);
        
        % Branch y_idx = 1
        y_lb1 = y_lb; y_ub1 = y_ub;
        y_lb1(idx) = 1; y_ub1(idx) = 1;
        z_u1 = u_relax;
        z_y1 = y_relax;
        z_y1(idx) = 1;
        z_y1 = max(y_lb1, min(y_ub1, z_y1));
        [~,~,cost1,exit1] = solve_relaxed(y_lb1, y_ub1, z_u1, z_y1);

       % Order children by bound (smaller cost first)
        children = {};
        if exit0==1
            children{end+1} = {y_lb0,y_ub0,z_u0,z_y0,cost0}; 
        end
        if exit1==1
            children{end+1} = {y_lb1,y_ub1,z_u1,z_y1,cost1}; 
        end
        if isempty(children)
            return; % both infeasible, prune
        end
        % sort children by lower bound
        [~,order] = sort(cellfun(@(x)x{5},children));
        children = children(order);
        % recurse on children
        for k=1:numel(children)
            r = children{k};
            branch_and_bound(r{1},r{2},r{3},r{4},depth+1);
        end
    end

    function [u_opt, y_opt, cost_opt, exitflag] = solve_relaxed(y_lb, y_ub, u_start, y_start)
        % Solve the continuous relaxation in variables z = [u; y]
        % Build H, f, Aineq, bineq, Aeq, beq for z

        % Block Hessian: cost only on u
        H = blkdiag(Q, zeros(N));
        f = zeros(2*N,1);

        % equality: A*u = w
        Aeq = [A, zeros(size(A,1),N)];
        beq = w;

        % inequalities from bounds:
        % 1) -u + (u_underbar + u_bar)*y <= u_bar
        A1 = [-eye(N), (u_underbar + u_bar)*eye(N)];
        b1 = u_bar * ones(N,1);

        % 2)  u + (u_underbar - u_bar)*y <= -u_underbar
        A2 = [ eye(N), (u_underbar - u_bar)*eye(N)];
        b2 = -u_underbar * ones(N,1);

        Aineq = [A1; A2];
        bineq = [b1; b2];

        % bounds
        lb = [-inf(N,1); y_lb];
        ub = [ inf(N,1); y_ub];

        % z0 warm start (clip to bounds)
        z0 = [u_start; y_start];
        z0 = max(lb, min(ub, z0));

        % Solve with quadprog; pass z0 as x0 (quadprog signature uses x0 before options)
        [z,fval,exitflag] = quadprog(H,f,Aineq,bineq,Aeq,beq,lb,ub,z0,options);
        
        if exitflag==1
            u_opt = z(1:N);
            y_opt = z(N+1:end);
            cost_opt = fval;
        else
            % quadprog did not return a valid solution
            warning('quadprog failed or infeasible at this node. exitflag=%d',exitflag);
            u_opt = zeros(N,1);      % fixed-size for codegen
            y_opt = zeros(N,1);
            cost_opt = Inf;
            exitflag = -1;
        end
    end
end