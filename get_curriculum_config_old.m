function [seg1_len, seg2_len, spacing, slip_enabled, agent_filename, prev_agent_filename] = get_curriculum_config(stage)
% GET_CURRICULUM_CONFIG  Tabel pengaturan tiap stage.
    switch stage
        case 1
            seg1_len = 5; seg2_len = 0; spacing = 0.1;
            slip_enabled = false;
            agent_filename = 'Agen_SAC_v3_stage1.mat';
            prev_agent_filename = '';
        case 2
            seg1_len = 5; seg2_len = 5; spacing = 0.1;
            slip_enabled = true;
            agent_filename = 'Agen_SAC_v3_stage2.mat';
            prev_agent_filename = 'Agen_SAC_v3_stage1.mat';   % warm start (hasil: JELEK, avg reward -200)
        case 3
            % EKSPERIMEN: path L + slip, TAPI TANPA warm start (mulai dari nol)
            % untuk membandingkan apakah warm start dari stage 1 justru
            % yang jadi biang keladi konvergensi buruk di stage 2.
            seg1_len = 5; seg2_len = 5; spacing = 0.1;
            slip_enabled = true;
            agent_filename = 'Agen_SAC_v3_stage2_fresh.mat';
            prev_agent_filename = '';   % <<< TIDAK warm start
        otherwise
            error('Stage %d belum didefinisikan.', stage);
    end
end