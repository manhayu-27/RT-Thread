function process_ecg_filter(rawCsv, outDir)
%PROCESS_ECG_FILTER Design and apply ECG validation filters for 500 Hz ADC data.

Fs = 500;
bandpassHz = [0.5 40.0];
notchHz = 50.0;
notchQ = 35.0;

if ~exist(outDir, 'dir')
    mkdir(outDir);
end

rawTable = readtable(rawCsv);
timestamp_ms = rawTable.timestamp_ms;
raw = double(rawTable.raw);
raw_centered = raw - mean(raw, 'omitnan');

[z_bp, p_bp, k_bp] = butter(4, bandpassHz / (Fs / 2), 'bandpass');
[sos_bp, g_bp] = zp2sos(z_bp, p_bp, k_bp);

wo = notchHz / (Fs / 2);
bw = wo / notchQ;
[b_notch, a_notch] = iirnotch(wo, bw);
[sos_notch, g_notch] = tf2sos(b_notch, a_notch);

sos = [sos_bp; sos_notch];
gain = g_bp * g_notch;
sos(1, 1:3) = sos(1, 1:3) * gain;

filtered = raw_centered;
for i = 1:size(sos, 1)
    filtered = filter(sos(i, 1:3), sos(i, 4:6), filtered);
end

filteredCsv = fullfile(outDir, 'com12_filtered.csv');
filteredTable = table(timestamp_ms, raw, filtered, ...
    'VariableNames', {'timestamp_ms', 'raw', 'filtered'});
writetable(filteredTable, filteredCsv);

coeffCsv = fullfile(outDir, 'ecg_filter_sos_coefficients.csv');
coeffTable = table((1:size(sos, 1))', sos(:, 1), sos(:, 2), sos(:, 3), sos(:, 5), sos(:, 6), ...
    'VariableNames', {'section', 'b0', 'b1', 'b2', 'a1', 'a2'});
writetable(coeffTable, coeffCsv);

coeffMat = fullfile(outDir, 'ecg_filter_coefficients.mat');
save(coeffMat, 'Fs', 'bandpassHz', 'notchHz', 'notchQ', 'sos');

fig = figure('Visible', 'off', 'Position', [100 100 1400 800]);
t = timestamp_ms / 1000.0;
subplot(2, 1, 1);
plot(t, raw, 'Color', [0.15 0.15 0.15]);
grid on;
xlabel('Time (s)');
ylabel('ADC raw');
title('COM12 raw ADC data');

subplot(2, 1, 2);
plot(t, filtered, 'Color', [0.0 0.35 0.8]);
grid on;
xlabel('Time (s)');
ylabel('Filtered ADC');
title('Filtered data: 0.5-40 Hz bandpass + 50 Hz notch');

plotPng = fullfile(outDir, 'com12_raw_vs_filtered.png');
exportgraphics(fig, plotPng, 'Resolution', 150);
close(fig);

summaryPath = fullfile(outDir, 'ecg_filter_summary.txt');
fid = fopen(summaryPath, 'w');
fprintf(fid, 'raw_csv=%s\n', rawCsv);
fprintf(fid, 'filtered_csv=%s\n', filteredCsv);
fprintf(fid, 'coeff_csv=%s\n', coeffCsv);
fprintf(fid, 'coeff_mat=%s\n', coeffMat);
fprintf(fid, 'plot_png=%s\n', plotPng);
fprintf(fid, 'sample_rate_hz=%g\n', Fs);
fprintf(fid, 'valid_samples=%d\n', numel(raw));
fprintf(fid, 'bandpass_hz=%g,%g\n', bandpassHz(1), bandpassHz(2));
fprintf(fid, 'notch_hz=%g\n', notchHz);
fprintf(fid, 'notch_q=%g\n', notchQ);
fclose(fid);
end
