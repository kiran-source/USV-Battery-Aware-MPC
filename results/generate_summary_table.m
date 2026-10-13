function summary_tbl = generate_summary_table(all_results, output_csv)
    % GENERATE_SUMMARY_TABLE Generates Formatted Comparative Table for All Experiments
    
    if nargin < 2, output_csv = 'final_results_summary.csv'; end
    
    N_exp = length(all_results);
    
    Experiment_Name  = cell(N_exp, 1);
    Initial_SOC_pct  = zeros(N_exp, 1);
    Final_SOC_pct    = zeros(N_exp, 1);
    Energy_Used_Wh   = zeros(N_exp, 1);
    Travel_Time_s    = zeros(N_exp, 1);
    Distance_m       = zeros(N_exp, 1);
    Speed_RMSE_mps   = zeros(N_exp, 1);
    CTE_RMSE_m       = zeros(N_exp, 1);
    Heading_RMSE_deg = zeros(N_exp, 1);
    Avg_Power_W      = zeros(N_exp, 1);
    Peak_Power_W     = zeros(N_exp, 1);
    Mission_Status   = cell(N_exp, 1);
    Feasibility      = cell(N_exp, 1);
    
    for i = 1:N_exp
        r = all_results{i};
        Experiment_Name{i}  = r.name;
        Initial_SOC_pct(i)  = r.SOC(1);
        Final_SOC_pct(i)    = r.summary.final_soc;
        Energy_Used_Wh(i)   = r.summary.energy_Wh;
        Travel_Time_s(i)    = r.summary.travel_time_s;
        Distance_m(i)       = r.summary.total_dist_m;
        Speed_RMSE_mps(i)   = r.summary.rmse_u;
        CTE_RMSE_m(i)       = r.summary.rmse_cte;
        Heading_RMSE_deg(i) = rad2deg(r.summary.rmse_psi);
        Avg_Power_W(i)      = r.summary.avg_power_W;
        Peak_Power_W(i)     = r.summary.peak_power_W;
        
        if r.summary.is_complete
            Mission_Status{i} = 'ARRIVED';
        elseif r.state(end) == 4
            Mission_Status{i} = 'RETURNED_TO_BASE';
        elseif r.state(end) == 3
            Mission_Status{i} = 'DOCKED_AT_CHARGER';
        else
            Mission_Status{i} = 'ABORTED / LOW_BATTERY';
        end
        
        if r.summary.is_feasible
            Feasibility{i} = 'FEASIBLE';
        else
            Feasibility{i} = 'MISSION AT RISK';
        end
    end
    
    summary_tbl = table(Experiment_Name, Initial_SOC_pct, Final_SOC_pct, ...
        Energy_Used_Wh, Travel_Time_s, Distance_m, Speed_RMSE_mps, CTE_RMSE_m, ...
        Heading_RMSE_deg, Avg_Power_W, Peak_Power_W, Mission_Status, Feasibility);
    
    writetable(summary_tbl, output_csv);
    writetable(summary_tbl, 'results_summary.csv'); % Also update base results_summary.csv
    
    fprintf('Saved summary metrics table to %s\n', output_csv);
    disp(summary_tbl);
end
