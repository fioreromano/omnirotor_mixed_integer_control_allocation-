function Track = trackObjectiveOverCoMShift(k, pi_B_Bcom_all, CoM_all)

eps_obj = 1e-12; 

if k == 1
    load("Results_Top01_Brescianini.mat", "Results_good");
else
    load("Results_Top01_Complete.mat", "Results_good");
end

Nref = Results_good(1).n;          % configurazione fissata
Ncom = size(pi_B_Bcom_all,3);
M = size(Nref,3);

Track.Nref = Nref;

F = nan(M, Ncom);
SIG = nan(M, Ncom);
COND = nan(M, Ncom);

for ic = 1:Ncom
    P = pi_B_Bcom_all(:,:,ic);
    for m = 1:M
        N = Nref(:,:,m);
        if k == 1
            B = AllocationMatrixNoDrag(N, P);
        else
            B = AllocationMatrixComplete(N, P);
        end
    
        svals = svd(B);
        smin  = max(min(svals), eps_obj);
        smax  = max(svals);
    
        SIG(m,ic)  = smin;
        COND(m,ic) = smax/smin;
        F(m,ic)    = -smin + 10*(smax/smin);
    end

    Track.F = F;
    Track.SIG = SIG;
    Track.COND = COND;

    shiftNorm = vecnorm(CoM_all - CoM_all(:,1), 2, 1).';
    Track.shift = shiftNorm;

    % Ottimo per F: min per colonna
    [Fbest, bestIdx] = min(F, [], 1);          % 1×Ncom
    Track.Fbest   = Fbest;
    Track.bestIdx = bestIdx;
    
    % Scostamento assoluto e relativo
    DeltaF = F - Fbest;                         % M×Ncom (broadcast)
    Track.DeltaF = DeltaF;
    
    % relativo (stabile): divido per max(|Fbest|, eps)
    den = max(abs(Fbest), 1e-12);
    Track.RelF = DeltaF ./ den;                 % M×Ncom  (0 = best)
    Track.RelF_pct = 100 * Track.RelF;
    
    % Scostamento rispetto a sigma/cond "ottimali"
    [SIGbest, bestIdxSIG] = max(SIG, [], 1);
    [CONDbest, bestIdxCOND] = min(COND, [], 1);
    
    Track.SIGbest = SIGbest;
    Track.CONDbest = CONDbest;
    Track.bestIdxSIG = bestIdxSIG;
    Track.bestIdxCOND = bestIdxCOND;
    
    Track.DeltaSIG = SIGbest - SIG;     % >=0 se lontano dal best (sigma più piccola)
    Track.DeltaCOND = COND - CONDbest;  % >=0 se lontano dal best (cond più grande)
end

end