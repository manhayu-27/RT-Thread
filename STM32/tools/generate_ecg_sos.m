function generate_ecg_sos(outDir)
%GENERATE_ECG_SOS Generate separated ECG IIR SOS coefficients for firmware.

Fs = 500;
highpassHz = 1.5;
lowpassHz = 35.0;
notchHz = [48.0 49.0 50.0 51.0 52.0];
notchQ = 5.0;

if ~exist(outDir, 'dir')
    mkdir(outDir);
end

[z_hp, p_hp, k_hp] = butter(2, highpassHz / (Fs / 2), 'high');
[sos_hp, g_hp] = zp2sos(z_hp, p_hp, k_hp);
sos_hp(1, 1:3) = sos_hp(1, 1:3) * g_hp;

[z_lp, p_lp, k_lp] = butter(4, lowpassHz / (Fs / 2), 'low');
[sos_lp, g_lp] = zp2sos(z_lp, p_lp, k_lp);
sos_lp(1, 1:3) = sos_lp(1, 1:3) * g_lp;

sos_notch = [];
for i = 1:numel(notchHz)
    wo = notchHz(i) / (Fs / 2);
    bw = wo / notchQ;
    [b_notch, a_notch] = iirnotch(wo, bw);
    [one_sos, g_notch] = tf2sos(b_notch, a_notch);
    one_sos(1, 1:3) = one_sos(1, 1:3) * g_notch;
    sos_notch = [sos_notch; one_sos]; %#ok<AGROW>
end

sos = [sos_hp; sos_lp; sos_notch];

coeffCsv = fullfile(outDir, 'ecg_filter_separated_sos_coefficients.csv');
coeffTable = table((1:size(sos, 1))', sos(:, 1), sos(:, 2), sos(:, 3), sos(:, 5), sos(:, 6), ...
    'VariableNames', {'section', 'b0', 'b1', 'b2', 'a1', 'a2'});
writetable(coeffTable, coeffCsv);

coeffMat = fullfile(outDir, 'ecg_filter_separated_coefficients.mat');
save(coeffMat, 'Fs', 'highpassHz', 'lowpassHz', 'notchHz', 'notchQ', 'sos');

summaryPath = fullfile(outDir, 'ecg_filter_separated_summary.txt');
fid = fopen(summaryPath, 'w');
fprintf(fid, 'sample_rate_hz=%g\n', Fs);
fprintf(fid, 'highpass_hz=%g\n', highpassHz);
fprintf(fid, 'lowpass_hz=%g\n', lowpassHz);
fprintf(fid, 'notch_hz=%g,%g,%g,%g,%g\n', notchHz(1), notchHz(2), notchHz(3), notchHz(4), notchHz(5));
fprintf(fid, 'notch_q=%g\n', notchQ);
fprintf(fid, 'sections=%d\n', size(sos, 1));
fprintf(fid, 'coeff_csv=%s\n', coeffCsv);
fprintf(fid, 'coeff_mat=%s\n', coeffMat);
fclose(fid);
end
