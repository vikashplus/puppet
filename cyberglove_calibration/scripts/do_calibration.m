run("true_values_n.m");
run("glove_values_n.m");
run("true_ranges.m");
run("glove_ranges.m");

trueNValue = true_values_n;
gloveNSamps = glove_samples_n;

n_raw = 22;
n_calib = 24;

map_raw{1} = [4 5 6 7 8 11];    % First finger
map_cal{1} = [3 4 5 6];
map_raw{2} = [8 9 10 11 12 15]; % Second finger
map_cal{2} = [7 8 9];
map_raw{3} = [12 13 14 15 19];  % Ring finger
map_cal{3} = [10 11 12 13];
map_raw{4} = [12 16 17 18 19];  % Little finger
map_cal{4} = [14 15 16 17];
map_raw{5} = [1 2 3 4 20 21];   % Thumb calibrate
map_cal{5} = [18 19 20 21];
map_raw{6} = [1 20 21 22];      % Wrist calibrate
map_cal{6} = [1 2];

calibration_octave = NaN(n_calib,n_raw+1);
for i=1:length(map_raw)
    map_raw_F	= map_raw{i};
    map_cal_F	= map_cal{i};
    calibration_octave(map_cal_F, :) = 0;
    calibration_octave(map_cal_F, [map_raw_F (n_raw+1)]) = trueNValue(map_cal_F,:)/...
        [gloveNSamps(map_raw_F,:); ones(1, size(gloveNSamps,2))];
end

dlmwrite('octave_output.handRange', true_ranges', 'delimiter', '\t','precision', '%1.5f');
dlmwrite('octave_output.userRange', glove_ranges', 'delimiter', '\t','precision', '%1.5f');
dlmwrite('octave_output.calib', calibration_octave, 'delimiter', '\t','precision', '%1.5f');