function [u_best, y_best, cost_best, flag] = solve_miqp_bb_stack_2(w,A,Q,u_underbar,u_bar,u_init,y_init,y_fixed,objective,Aeq_on,verbose)
% Branch & Bound MIQP solver (safe, active-set, warm-started)
% Solves:
%   min (1/2) u' * Q * u
%   s.t. A*u = w (if Aeq_on==1)
%        u_underbar*y - u_bar*(1-y) <= u <= u_bar*y - u_underbar*(1-y)
%        y in {0,1}^N
%
% Inputs:
%   w, A, Q, u_underbar, u_bar
%   u_init (N x 1) initial guess for u (optional, pass [] to default)
%   y_init (N x 1) initial guess for y (optional, pass [] to default)
%   y_fixed (N x 1) optional vector: NaN entries mean free, 0/1 means fixed
%   objective (struct): fields 'Q' (NxN), 'f' (Nx1) for general cost (optional)
%   Aeq_on : 1 include A*u=w, 0 ignore equality (optional, default 1)
%   verbose : 1 per stampare debug, 0 per silenzioso (optional, default 0)
%
% Outputs:
%   u_best, y_best : best integer-feasible solution found (empty if none)
%   cost_best      : cost of u_best (Inf if none)
%   flag           : 1 if integer-feasible solution found, 0 otherwise
% Default per verbose
if nargin < 11 || isempty(verbose)
    verbose = 0;
end

% dimensions
N = size(A,2);

% defaults for initial guesses
if nargin < 6 || isempty(u_init)
    u_init = zeros(N,1);
end
if nargin < 7 || isempty(y_init)
    y_init = 0.5*ones(N,1); % fractional warm start
end
% default y_fixed: all free
if nargin < 8 || isempty(y_fixed)
    y_fixed = NaN(N,1);
end
% default objective
if nargin < 9 || isempty(objective)
    objective.Q = Q;
    objective.f = zeros(N,1);
end
% default Aeq indicator
if nargin < 10 || isempty(Aeq_on)
    Aeq_on = 1;
end

% clip initial sizes
u_init = u_init(:);
y_init = y_init(:);
y_fixed = y_fixed(:);
if length(u_init)~=N || length(y_init)~=N || length(y_fixed)~=N
    error('u_init, y_init and y_fixed must be length N.');
end

% outputs / best-so-far
cost_best = Inf;
u_best = [];
y_best = [];
flag = 0;

% initial bounds on y (relaxed)
y_lb0 = zeros(N,1);
y_ub0 = ones(N,1);

% apply user-specified fixed y entries: if y_fixed(i) is 0 or 1, fix bounds
fixed_count = 0;
for i = 1:N
    if ~isnan(y_fixed(i))
        if ~(y_fixed(i)==0 || y_fixed(i)==1)
            error('y_fixed entries must be NaN, 0 or 1.');
        end
        y_lb0(i) = y_fixed(i);
        y_ub0(i) = y_fixed(i);
        fixed_count = fixed_count + 1;
    end
end

if verbose
    fprintf('N (numero di rotori): %d\n', N);
    fprintf('Variabili y fissate all''inizio: %d\n', fixed_count);
    if fixed_count > 0
        fprintf('  y fissate: ');
        for i = 1:N
            if ~isnan(y_fixed(i))
                fprintf('y[%d]=%g ', i, y_fixed(i));
            end
        end
        fprintf('\n');
    end
    fprintf('Costo migliore iniziale: Inf\n');
end

% quadprog options (active-set)
options = optimoptions('quadprog', ...
                       'Algorithm','active-set', ...
                       'Display','off');

% start iterative branch & bound using explicit stack to avoid recursion
maxDepth = N + 5; % safe guard

% stack is an array of structs with fields: y_lb, y_ub, u_start, y_start, depth
emptyNode = struct( ...
    'y_lb', zeros(N,1), ...
    'y_ub', zeros(N,1), ...
    'u_start', zeros(N,1), ...
    'y_start', zeros(N,1), ...
    'depth', 0.0);

% Preallocate stack with some safe maximum size
stackSize = 2*(N+5);  % heuristic upper bound
stack = repmat(emptyNode, stackSize, 1);

stack_ptr = 1;
stack(stack_ptr).y_lb = y_lb0;
stack(stack_ptr).y_ub = y_ub0;
stack(stack_ptr).u_start = u_init;
stack(stack_ptr).y_start = y_init;
stack(stack_ptr).depth = 0;

if verbose
    fprintf('Nodo radice creato, profondità: 0\n');
    fprintf('Stack iniziale: 1 nodo\n\n');
end

node_count = 0;
leaf_count = 0;
prune_count = 0;
integer_count = 0;
branch_count = 0;

while stack_ptr > 0
    node_count = node_count + 1;
    
    if verbose
        fprintf('--- Nodo %d ---\n', node_count);
        fprintf('Stack size: %d\n', stack_ptr);
    end
    
    % Pop
    node = stack(stack_ptr);
    stack_ptr = stack_ptr - 1;
    y_lb = node.y_lb;
    y_ub = node.y_ub;
    u_start = node.u_start;
    y_start = node.y_start;
    depth = node.depth;
    
    if verbose
        fprintf('Profondità: %d\n', depth);
        fprintf('y bounds: ');
        for i = 1:N
            if y_lb(i) == y_ub(i)
                fprintf('y[%d]=%g ', i, y_lb(i));
            else
                fprintf('y[%d]∈[%g,%g] ', i, y_lb(i), y_ub(i));
            end
        end
        fprintf('\n');
    end
    
    % Guard recursion depth (converted to iterative guard)
    if depth > maxDepth
        if verbose
            fprintf('PRUNE: massima profondità raggiunta (%d)\n', maxDepth);
        end
        prune_count = prune_count + 1;
        continue;
    end

    % If all y are fixed by bounds: Check if it reached a leaf node
    if all(abs(y_lb - y_ub) < 1e-12)
        leaf_count = leaf_count + 1;
        y_fixed_now = round(y_lb); % integral
        
        if verbose
            fprintf('NODO FOGLIA - tutte y fissate:\n');
            for i = 1:N
                fprintf('  y[%d] = %g', i, y_fixed_now(i));
                if y_fixed_now(i) == 1.0
                    fprintf(' => u[%d] ∈ [%g,%g]', i, u_underbar, u_bar);
                else
                    fprintf(' => u[%d] ∈ [%g,%g]', i, -u_bar, -u_underbar);
                end
                fprintf('\n');
            end
        end
        
        % bounds for u given y_fixed
        lb_u = u_underbar.*y_fixed_now - u_bar.*(1 - y_fixed_now);
        ub_u = u_bar.*y_fixed_now - u_underbar.*(1 - y_fixed_now);

        % clip initial u
        u0 = max(lb_u, min(ub_u, u_start));

        % solve QP in u only
        if Aeq_on
            [u_node, fval_u, exitflag_u] = quadprog(objective.Q, objective.f, ...
                [],[], A, w, lb_u, ub_u, u0, options);
        else
            [u_node, fval_u, exitflag_u] = quadprog(objective.Q, objective.f, ...
                [],[], [], [], lb_u, ub_u, u0, options);
        end

        if exitflag_u ~= 1
            continue;
        end

        % fval_u is objective value = 0.5*u'*Q*u + f'*u
        
        if fval_u + 1e-12 < cost_best
            cost_best = fval_u;
            u_best = u_node;
            y_best = y_fixed_now;
        end
        continue;
    end

    % Solve relaxed QP in (u,y)
    if verbose
        fprintf('Risolvo QP rilassato per il nodo...\n');
    end
    
    [u_relax, y_relax, cost_relax, exitflag_relax] = solve_relaxed(N,A,objective,w,u_underbar,u_bar,y_lb,y_ub,u_start,y_start,options,Aeq_on);
                                           
    if exitflag_relax ~= 1
        if verbose
            fprintf('QP rilassato INFATTIBILE - PRUNE\n');
        end
        prune_count = prune_count + 1;
        continue;
    end
    
    if verbose
        fprintf('QP rilassato RISOLTO, costo: %g\n', cost_relax);
        fprintf('y rilassate: [');
        fprintf('%g ', y_relax);
        fprintf(']\n');
    end
    
    % prune if bound not better than best
    if cost_relax + 1e-12 >= cost_best 
        prune_count = prune_count + 1;
        continue;
    end

    % if all y integral within tolerance, becomes a leaf node -> update best or prune
    if all(abs(y_relax - round(y_relax)) < 1e-6)
        integer_count = integer_count + 1;
        y_int = round(y_relax);
        
        
        if cost_relax + 1e-12 < cost_best 
            cost_best = cost_relax;
            u_best = u_relax;
            y_best = y_int;
        end
        continue; %prune
    end

    % pick index to branch: among unfixed variables choose most fractional
    unfixed = find(y_lb + 1e-12 < y_ub); % indices that are not fixed by bounds
    
    if verbose
        fprintf('Variabili non fissate: %d/%d\n', length(unfixed), N);
    end
    
    if isempty(unfixed)
        continue;
    end
    
    fractional = abs(y_relax(unfixed) - round(y_relax(unfixed)));
    [~, imax] = max(fractional);
    idx = unfixed(imax);

    % Extra guard: ensure idx isn't already fixed by numerical noise
    if y_lb(idx) == y_ub(idx)
        continue;
    end
    
    if verbose
        fprintf('BRANCHING su y[%d]\n', idx);
        fprintf('  Valore rilassato: %g (frazionario: %g)\n', y_relax(idx), fractional(imax));
    end
    branch_count = branch_count + 1;

    % Branch y_idx = 0
    y_lb0 = y_lb; y_ub0 = y_ub;
    y_lb0(idx) = 0; y_ub0(idx) = 0;
    z_u0 = u_relax;
    z_y0 = y_relax;
    z_y0(idx) = 0;
    z_y0 = max(y_lb0, min(y_ub0, z_y0));
    [~,~,cost0,exit0] = solve_relaxed(N,A,objective,w,u_underbar,u_bar,y_lb0,y_ub0,z_u0,z_y0,options,Aeq_on);

    % Branch y_idx = 1
    y_lb1 = y_lb; y_ub1 = y_ub;
    y_lb1(idx) = 1; y_ub1(idx) = 1;
    z_u1 = u_relax;
    z_y1 = y_relax;
    z_y1(idx) = 1;
    z_y1 = max(y_lb1, min(y_ub1, z_y1));
    [~,~,cost1,exit1] = solve_relaxed(N,A,objective,w,u_underbar,u_bar,y_lb1,y_ub1,z_u1,z_y1,options,Aeq_on);

    % Order children by bound (smaller cost first) and push to stack (LIFO)
    children_costs = [];
    children_nodes = {};
    if exit0==1
        children_costs(end+1) = cost0; 
        children_nodes{end+1} = struct('y_lb',y_lb0,'y_ub',y_ub0,'u_start',z_u0,'y_start',z_y0,'depth',depth+1);
    end
    if exit1==1
        children_costs(end+1) = cost1; 
        children_nodes{end+1} = struct('y_lb',y_lb1,'y_ub',y_ub1,'u_start',z_u1,'y_start',z_y1,'depth',depth+1); 
    end
    if isempty(children_costs)
        continue; % both infeasible, prune
    end
    [~, order] = sort(children_costs);
    for k = numel(order):-1:1
        idxc = order(k);
        stack_ptr = stack_ptr + 1;
        stack(stack_ptr) = children_nodes{idxc};
    end
end

end
% ---------------- helper subfunction ----------------
function [u_opt, y_opt, cost_opt, exitflag] = solve_relaxed(N,A,objective,w,u_underbar,u_bar,y_lb,y_ub,u_start,y_start,options,Aeq_on)
    H = blkdiag(objective.Q, zeros(N));
    f = [objective.f; zeros(N,1)];

    if Aeq_on
        Aeq = [A, zeros(size(A,1),N)];
        beq = w;
    else
        Aeq = [];
        beq = [];
    end

    A1 = [-eye(N), (u_underbar + u_bar)*eye(N)];
    b1 = u_bar * ones(N,1);
    A2 = [ eye(N), (- u_underbar - u_bar)*eye(N)];
    b2 = -u_underbar * ones(N,1);

    Aineq = [A1; A2];
    bineq = [b1; b2];

    lb = [-inf(N,1); y_lb];
    ub = [ inf(N,1); y_ub];
    z0 = [u_start; y_start];
    z0 = max(lb, min(ub, z0));

    [z,fval,exitflag] = quadprog(H,f,Aineq,bineq,Aeq,beq,lb,ub,z0,options);

    if exitflag==1
        u_opt = z(1:N);
        y_opt = z(N+1:end);
        cost_opt = fval;
    else
        u_opt = zeros(N,1);
        y_opt = zeros(N,1);
        cost_opt = Inf;
        exitflag = -1;
    end
end
