function [best_fval, best_x_opt, best_W_map] = optimization_loop( ...
        x0, cf, ct, P, lambda, w_des, u_max)

    % Inizializzazione variabili risultati
    best_fval = Inf;
    best_x_opt = [];
    best_W_map = [];

    try
        % Ottimizzazione
        options = optimoptions('fmincon', ...
            'Display', 'off', ...
            'Algorithm', 'sqp', ...
            'MaxFunctionEvaluations', 10000);
        
        % CORREZIONE: usa length(x0) invece di n_vars
        n_vars = length(x0);
        lb = -pi/2 * ones(n_vars, 1); 
        ub = pi/2 * ones(n_vars, 1); 

        [x_opt_current, fval_current, exitflag] = fmincon( ...
            @(x) obj_min_singular(x, cf, ct, P, lambda), ...
            x0, [], [], [], [], lb, ub, ...
            @(x) nonlcon_singular(x, cf, ct, P, lambda, w_des, u_max), options);

        % Considera solo soluzioni con convergenza accettabile
        if exitflag > 0  
            
            % Calcola Wrench Map per la soluzione corrente
            W_map_current = myWrenchMapVariableTilt( ...
                x_opt_current, cf, ct, P, lambda);

            % Numero di condizionamento
            cond_current = cond(W_map_current);

            % Accetta solo soluzioni finite e non troppo mal condizionate
            if isfinite(cond_current) && cond_current < 1e10

                fprintf('fval = %8.6f, cond = %.4f, exit = %d\n', ...
                        fval_current, cond_current, exitflag);

                % Aggiornamento del migliore risultato
                if fval_current < best_fval
                    best_fval = fval_current;
                    best_x_opt = x_opt_current;
                    best_W_map = W_map_current;

                    fprintf('  → ★ Soluzione trovata! ★\n');
                end
            end
        end
        
    catch ME
        fprintf('Errore nell''ottimizzazione: %s\n', ME.message);
    end
end