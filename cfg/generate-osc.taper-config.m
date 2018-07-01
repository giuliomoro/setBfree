
for drawbar = 1:9
if(0) % decide whether to load bin or wav
    % play a chromatic scale on the real organ with just one drawbar starting from the low C
    filename = sprintf('voicing-chromatic-drawbar%d.bin', drawbar);
    Fs = 44100; % audio sample rate
    BUFFER_SIZE_SAMPLES = 64;
    TOTAL_SCANNER_KEYS = 73;
    % binary file contains, for each buffer:
    % TOTAL_SCANNER_KEYS keys positions
    % BUFFER_SIZE_SAMPLES audio samples from the organ
    % BUFFER_SIZE_SAMPLES audio samples from the audio input

    numKeys = TOTAL_SCANNER_KEYS;
    numAudioFrames = BUFFER_SIZE_SAMPLES;
    bufferTot = numKeys + numAudioFrames * 2;

    fid=fopen(filename,'r');
    if fid < 0 
        continue
    end
    Ain = fread(fid, 'float'); % load in A the values logged to the binary file
    fclose(fid);

    clear organ backing
    totBuffers = length(Ain) / bufferTot;
    keys = nan(numKeys, bufferTot);
    for n = 1 : totBuffers
        start = (n-1) * bufferTot + 1;
        stop = start + bufferTot - 1;
        buffer = Ain(start:stop);
        keys(:, n) = buffer(1:numKeys);

        destart = numAudioFrames * (n-1) + 1;
        destop = destart + numAudioFrames - 1;
        sostart = numKeys + 1;
        sostop = numKeys + numAudioFrames;
        organ(destart : destop) = buffer(sostart:sostop);

        sostart = sostart + numAudioFrames;
        sostop = sostop + numAudioFrames;

        backing(destart : destop) = buffer(sostart:sostop);
    end

    backing = backing(10000:end);
    plot(backing)
    threshold = 0.0002;
    x = abs(backing);
else
    filename = sprintf('/Users/giulio/OneDrive - Queen Mary, University of London/matlab/phd/touch-response-test/voicingCalib/drawbars/voicing-chromatic-drawbar%d.wav', drawbar);
    x = abs(audioread(filename));
    threshold = 0.018;
end
    kSilence = 0;
    kNoteStart = 1;
    kNoteDone = 2;
    nWheels = 91;
    firstKey = 1;
    keyAmps = nan(61, 1);
    wheelAmps = nan(nWheels, 1);
    noteStarts = wheelAmps;
    noteEnds = wheelAmps;
    attackOffset = 300;
    win = 2048;
    n = 1;
    state = kSilence;
    currKey = firstKey - 1;

    % find a note
    while n < length(x) - win - 1
        if state == kSilence && rms(x(n:n+win-1)) > threshold
            state = kNoteStart;
            noteStart = n + attackOffset;
            currKey = currKey + 1;
        end
        if state == kNoteStart && rms(x(n:n+win-1)) < threshold
            % either the note is complete or we are towards the end
            % so let's see if there is energy in the next window
            n = n + win;
            if rms(x(n:n+win-1)) < threshold
                % if there is not much energy then it is definitely silence
                noteStarts(currKey) = noteStart;
                noteEnds(currKey) = n - 2 * win;
                lim = [noteStarts(currKey) noteEnds(currKey)];
    %             plot(t, x, lim, x(lim), 'o')            
%                 fprintf('Detected note %d, (%d:%d)\n', currKey, noteStarts(currKey), noteEnds(currKey));
                state = kSilence;
            end
        end
        n = n + win;
    end
    %%
    for n = 1:length(keyAmps)
        key = n;
        wheel = getWheel(key, drawbar);
        if wheel < 0
            error
        end
        amp = rms(x(noteStarts(n):noteEnds(n))) / 0.707 * 2;
        keyAmps(key) = amp;
        wheelAmps(wheel) = amp;
        fprintf('osc.taper.k%d.b%d.t%d = %.7f\n',...
            key - 1, drawbar - 1, wheel, amp ... % key-1 and drawbar-1 to match setBfree numbering : wheels are 1-based, keys and drawbars are 0-based
        );
    end

    %%
    t = 1 : length(x);
    clf
    plot(t, x)
    hold on
    plot([noteStarts(1:currKey) noteEnds(1:currKey)]', 0.2 * [ones(currKey,2) ]', 'r', 'linewidth', 2)
%     plot([noteStarts(1:currKey) noteEnds(1:currKey)]', [keyAmps(1:currKey) keyAmps(1:currKey)]', 'r', 'linewidth', 2)
    % plot([noteStarts(1:wheel)]', [wheelAmps(1:wheel)]', 'r-o')
    hold off
end

function wheel = getWheel(key, drawbar)
    nBusses = 9;
    wheelsToBus = [13 20 13 25 32 37 41 44 49 14 21 14 26 33 38 42 45 50 ...
        15 22 15 27 34 39 43 46 51 16 23 16 28 35 40 44 47 52 17 24 17 ...
        29 36 41 45 48 53 18 25 18 30 37 42 46 49 54 19 26 19 31 38 43 ... 
        47 50 55 20 27 20 32 39 44 48 51 56 21 28 21 33 40 45 49 52 57 ...
        22 29 22 34 41 46 50 53 58 23 30 23 35 42 47 51 54 59 24 31 24 ...
        36 43 48 52 55 60 13 32 25 37 44 49 53 56 61 14 33 26 38 45 50 ...
        54 57 62 15 34 27 39 46 51 55 58 63 16 35 28 40 47 52 56 59 64 ...
        17 36 29 41 48 53 57 60 65 18 37 30 42 49 54 58 61 66 19 38 31 ...
        43 50 55 59 62 67 20 39 32 44 51 56 60 63 68 21 40 33 45 52 57 ...
        61 64 69 22 41 34 46 53 58 62 65 70 23 42 35 47 54 59 63 66 71 ...
        24 43 36 48 55 60 64 67 72 25 44 37 49 56 61 65 68 73 26 45 38 ...
        50 57 62 66 69 74 27 46 39 51 58 63 67 70 75 28 47 40 52 59 64 ...
        68 71 76 29 48 41 53 60 65 69 72 77 30 49 42 54 61 66 70 73 78 ...
        31 50 43 55 62 67 71 74 79 32 51 44 56 63 68 72 75 80 33 52 45 ...
        57 64 69 73 76 81 34 53 46 58 65 70 74 77 82 35 54 47 59 66 71 ...
        75 78 83 36 55 48 60 67 72 76 79 84 37 56 49 61 68 73 77 80 85 ...
        38 57 50 62 69 74 78 81 86 39 58 51 63 70 75 79 82 87 40 59 52 ...
        64 71 76 80 83 88 41 60 53 65 72 77 81 84 89 42 61 54 66 73 78 ...
        82 85 90 43 62 55 67 74 79 83 86 91 44 63 56 68 75 80 84 87 80 ...
        45 64 57 69 76 81 85 88 81 46 65 58 70 77 82 86 89 82 47 66 59 ...
        71 78 83 87 90 83 48 67 60 72 79 84 88 91 84 49 68 61 73 80 85 ...
        89 80 85 50 69 62 74 81 86 90 81 86 51 70 63 75 82 87 91 82 87 ...
        52 71 64 76 83 88 80 83 88 53 72 65 77 84 89 81 84 89 54 73 66 ...
        78 85 90 82 85 90 55 74 67 79 86 91 83 86 91 56 75 68 80 87 80 ...
        84 87 80 57 76 69 81 88 81 85 88 81 58 77 70 82 89 82 86 89 82 ...
        59 78 71 83 90 83 87 90 83 60 79 72 84 91 84 88 91 84 61 80 73 ...
        85 80 85 89 80 85];
    idx = (key - 1) * nBusses + drawbar;
    wheel = wheelsToBus(idx);
end
