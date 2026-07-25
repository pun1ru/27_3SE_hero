% Compare full friction-wheel data with the 3000:3600 disturbance segment.
% Fs must match the logger configuration. The current logger uses 500 Hz.

clear; clc; close all;

Fs = 500;
disturbanceRange = 3000:3600;  % MATLAB row numbers after the CSV header
welchLength = 256;

cases = struct( ...
    'file', { ...
        '开启吊射摩擦轮编码器与角速度采样.csv', ...
        '打蛋扰动观测.csv'}, ...
    'label', {'Friction wheel enabled', 'Projectile disturbance'}, ...
    'key', {'friction_enabled', 'projectile_disturbance'}, ...
    'range', {[], disturbanceRange});

sensorVars = {'yaw_imu_d', 'yawenc'};
sensorLabels = {'Yaw-rate gyro', 'Yaw encoder'};

results = table();
for caseIndex = 1:numel(cases)
    data = readtable(cases(caseIndex).file, 'VariableNamingRule', 'preserve');
    if isempty(cases(caseIndex).range)
        sampleRange = 1:height(data);
        rangeLabel = sprintf('logged samples 0-%d', height(data) - 1);
    else
        sampleRange = cases(caseIndex).range;
        rangeLabel = sprintf('samples %d-%d', sampleRange(1), sampleRange(end));
    end
    assert(height(data) >= sampleRange(end), ...
        '%s has fewer than %d samples.', cases(caseIndex).file, sampleRange(end));

    for sensorIndex = 1:numel(sensorVars)
        variableName = sensorVars{sensorIndex};
        assert(ismember(variableName, data.Properties.VariableNames), ...
            'Column "%s" is missing from %s.', variableName, cases(caseIndex).file);

        raw = double(data{sampleRange, variableName});
        raw = raw(:);

        % Remove DC and linear drift. This retains periodic motor vibration
        % and the transient caused by the projectile disturbance.
        ac = detrend(raw, 'linear');

        % The full-record Hann-window FFT gives Fs / numel(ac) Hz bins.
        [amplitude, fFft] = oneSidedAmplitude(ac, Fs);

        % Welch PSD is the curve to use for noise-floor fitting and comparison.
        window = hann(welchLength, 'periodic');
        [psdValue, fPsd] = pwelch(ac, window, welchLength / 2, welchLength, Fs, 'onesided');
        psdDb = 10 * log10(psdValue);

        % Fit the broadband PSD only. Narrow peaks are excluded, because they
        % are deterministic vibration/disturbance content rather than white noise.
        [backgroundPsd, slopeDbPerDecade, interceptLog10] = fitBroadbandPsd(fPsd, psdValue);

        figureHandle = figure('Name', sprintf('%s - %s', cases(caseIndex).label, sensorLabels{sensorIndex}), ...
            'Color', 'w');
        figureHandle.ToolBar = 'none';
        tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

        axAmplitude = nexttile;
        axAmplitude.Toolbar.Visible = 'off';
        plot(fFft, amplitude, 'Color', [0.15 0.35 0.65], 'LineWidth', 1);
        grid on; xlim([0 Fs / 2]);
        xlabel('Frequency (Hz)'); ylabel('Amplitude');
        title({cases(caseIndex).label, sprintf('%s AC amplitude, %s', ...
            sensorLabels{sensorIndex}, rangeLabel)});

        axPsd = nexttile;
        axPsd.Toolbar.Visible = 'off';
        semilogx(fPsd(2:end), psdDb(2:end), 'Color', [0.10 0.10 0.10], 'LineWidth', 1); hold on;
        semilogx(fPsd(2:end), 10 * log10(backgroundPsd(2:end)), '--', ...
            'Color', [0.80 0.20 0.15], 'LineWidth', 1.3);
        grid on; xlim([max(0.5, fPsd(2)) Fs / 2]);
        xlabel('Frequency (Hz)'); ylabel('PSD (unit^2/Hz)');
        title(sprintf('Welch PSD; broadband slope = %.2f dB/decade', slopeDbPerDecade));
        legend('Measured PSD', 'Robust broadband fit', 'Location', 'best');
        exportgraphics(figureHandle, sprintf('yaw_spectrum_%s_%s_%s.png', ...
            cases(caseIndex).key, variableName, rangeFileLabel(sampleRange, height(data))), ...
            'Resolution', 160);

        peak = dominantPeak(fFft, amplitude);
        newRow = table(string(cases(caseIndex).label), string(variableName), ...
            mean(raw), std(ac), peak.frequency, peak.amplitude, slopeDbPerDecade, interceptLog10, ...
            'VariableNames', {'Condition', 'Signal', 'Mean', 'AcStd', ...
            'DominantFrequencyHz', 'DominantAmplitude', 'BroadbandSlopeDbPerDecade', ...
            'BroadbandInterceptLog10'});
        results = [results; newRow]; %#ok<AGROW>
    end
end

disp(results);
writetable(results, 'yaw_noise_spectrum_fit.csv');

function [P1, f] = oneSidedAmplitude(x, Fs)
    N = numel(x);
    nfft = N;
    w = hann(N, 'periodic');
    coherentGain = mean(w);
    Y = fft(x .* w, nfft);
    P2 = abs(Y) / (N * coherentGain);
    halfN = floor(nfft / 2);
    P1 = P2(1:halfN + 1);
    if rem(nfft, 2) == 0
        P1(2:end-1) = 2 * P1(2:end-1);
    else
        P1(2:end) = 2 * P1(2:end);
    end
    f = Fs * (0:halfN)' / nfft;
end

function [backgroundPsd, slopeDbPerDecade, interceptLog10] = fitBroadbandPsd(f, Pxx)
    % Log-log robust line: PSD(f) = 10^b * f^m. Exclude the strongest 15%%
    % bins to prevent vibration tones from biasing the estimated noise floor.
    valid = f > 0 & isfinite(Pxx) & Pxx > 0;
    logF = log10(f(valid));
    logP = log10(Pxx(valid));
    cutoff = prctile(logP, 85);
    inlier = logP <= cutoff;
    coefficient = polyfit(logF(inlier), logP(inlier), 1);
    backgroundPsd = 10 .^ polyval(coefficient, log10(max(f, eps)));
    slopeDbPerDecade = 10 * coefficient(1);
    interceptLog10 = coefficient(2);
end

function peak = dominantPeak(f, Pxx)
    valid = f > 0;
    [peak.amplitude, index] = max(Pxx(valid));
    frequencies = f(valid);
    peak.frequency = frequencies(index);
end

function label = rangeFileLabel(sampleRange, totalSamples)
    if numel(sampleRange) == totalSamples && sampleRange(1) == 1 && sampleRange(end) == totalSamples
        label = 'all_samples';
    else
        label = sprintf('%d_%d', sampleRange(1), sampleRange(end));
    end
end
