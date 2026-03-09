function [hist_p, hist_v, hist_w, hist_err_p, hist_R_err, hist_R, hist_u_rot, hist_f_rot, hist_W_des, hist_W_real] = control_loop (B_ctrl, B_plant, CoM_sim, desired, Rdes, config )

    cf = config.uavParams.c_f;
    J_in = config.uavParams.inertia;      
    m_sys = config.system.mass;    
    J_inv = diag([1/J_in(1,1),1/J_in(2,2),1/J_in(3,3)]);
    B_ctrl = cf*B_ctrl;
    B_plant = cf*B_plant;
    
    pdes = desired(1:3,:);
    vdes = desired(4:6,:);
    ades = desired(7:9,:);
    omd = desired(10:12,:);
    domd = desired(13:15,:);
    tdes = desired(16,:);

    % Stato: p (3x1), v (3x1), R (3x3), omega (3x1)
    p_curr = pdes(:,1);
    v_curr = zeros(3, 1);
    R_curr = eye(3);
    w_curr = zeros(3, 1);
    e_p = zeros(3, 1); 
    ep_integral = 0;
    eR_integral = 0;
    
    dt = tdes(2) - tdes(1); % Time step derivato dal vettore tempo
    
    % Preallocazione
    N_steps = length(tdes);
    hist_p = zeros(3, N_steps);
    hist_v = zeros(3, N_steps);
    hist_w = zeros(3, N_steps);
    hist_err_p = zeros(1, N_steps);
    hist_R_err = zeros(1, N_steps);
    hist_R = zeros(3, 3, N_steps);
    hist_u_rot = zeros(8, N_steps);
    hist_f_rot = zeros(8, N_steps);
    hist_W_des = zeros(6, N_steps);
    hist_W_real = zeros(6, N_steps);

    fprintf('Simulating trajectory...\n');
    
    % 3. CONTROLLER GAINS (PID Geometrico)
    Kp1 = config.conrtolParams.gainsPID.Kp1; KR1 = config.conrtolParams.gainsPID.KR1;
    Kp2 = config.conrtolParams.gainsPID.Kp2; KR2 = config.conrtolParams.gainsPID.KR2;
    Kp3 = config.conrtolParams.gainsPID.Kp3; KR3 = config.conrtolParams.gainsPID.KR3;    
    
    for i = 1:N_steps        
        % --- 1. Get Reference at step i ---
        p_ref = pdes(:,i);
        v_ref = vdes(:,i);
        a_ref = ades(:,i);
        
        R_ref = Rdes(:,:,i);
        w_ref = omd(:,i);    % Desired Angular Velocity (Body Frame projected)
        dw_ref = domd(:,i);  % Desired Angular Acceleration
        
        % --- 2. Position Control (Geometric) ---
        e_p = p_curr - p_ref;
        ep_integral = ep_integral + e_p * dt;
        e_v = v_curr - v_ref;  
    
        % --- 3. Attitude Control (Geometric on SO(3)) ---
        % Errore di rotazione: 0.5 * vee(R_des'*R - R'*R_des)
        e_R = 0.5*vmap(R_curr, R_ref); % Vee map
        
        % Errore velocità angolare
        e_w = w_curr - R_curr' * R_ref * w_ref; 
        eR_integral = eR_integral + e_R*dt;
        
        % Virtual Control Input
        uv_p = a_ref ...
             - (Kp1(:) .* e_v) ...
             - (Kp2(:) .* e_p) ...
             - (Kp3(:) .* ep_integral);
        
        uv_R = dw_ref ...
             - (KR1(:) .* e_w) ...
             - (KR2(:) .* e_R) ...
             - (KR3(:) .* eR_integral);

    
        u_virt = [uv_p; uv_R];

        % Feedforward termine giroscopico
        term_gyr = -J_inv*cross(w_curr, J_in * w_curr);
        grav = -[0; 0; config.gravity];

        JR_mat = [R_curr*(1/m_sys), zeros(3); zeros(3),J_inv];
        J_mat = JR_mat*B_ctrl;
    
        % Feedback Linearization
        bias_forces = [grav; term_gyr];        
    
        % Bias
        bias =  - bias_forces + u_virt;

        % --- 4. Allocation ---
        u_rot = pinv(J_mat)* bias;
        % for k = 1:config.uavParams.np
        %     u_rot(k) = max(u_rot(k), -config.uavParams.maxPropSpeed^2); % se bidirezionale
        %     u_rot(k) = min(u_rot(k),  config.uavParams.maxPropSpeed^2);
        % end
        f_rot = u_rot*cf;
        f_body_des = B_ctrl(1:3,:) * u_rot;
        tau_body_des = B_ctrl(4:6,:) * u_rot;
        W_des = [f_body_des; tau_body_des];
        
        % --- 5. Forward Dynamics ---
        f_body_real = B_plant(1:3,:) * u_rot;
        tau_body_real = B_plant(4:6,:) * u_rot;
        W_real = [f_body_real; tau_body_real];
        p_ddt_real = grav + (1/m_sys) * R_curr * f_body_real; 
        w_dt_real = term_gyr + J_in \ tau_body_real;
        
        % --- 6. Integration (Euler) ---
        v_curr = v_curr + p_ddt_real * dt;
        p_curr = p_curr + v_curr * dt;
        w_curr = w_curr + w_dt_real * dt;
        
        % Aggiornamento Matrice Rotazione: R_new = R_curr * exp(w*dt)
        % Approssimazione skew: R_dot = R * skew(w)
        skew_w = skewMat(w_curr);
        Rdot = R_curr*skew_w;
        R_curr= R_curr + Rdot*dt;      
        [U,~,V] = svd(R_curr);
        R_curr = U*V';
        
        % Salvataggio dati
        hist_p(:,i) = p_curr;
        hist_v(:,i) = v_curr;
        hist_w(:,i) = w_curr;
        hist_err_p(i) = norm(e_p);
        hist_R_err(i) = norm(e_R);
        hist_R(:,:,i) = R_curr;
        hist_u_rot(:,i) = u_rot;
        hist_f_rot(:,i) = f_rot;
        hist_W_des(:,i) = W_des;
        hist_W_real(:,i) = W_real;
    end
end

