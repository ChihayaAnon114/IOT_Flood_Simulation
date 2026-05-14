clear; clc; close all;
rng('default');

snrVec = 0:0.2:12;             % SNR values in dB
numLoops = 500;               % Number of transmissions per SNR point
modulationScheme = 'BPSK';     % Options: 'BPSK' or '4QAM'

% FEC options:
% 'NONE'       : no forward error correction
% 'HAMMING'    : Hamming(7,4)
% 'REPETITION' : repetition code with majority voting
% 'LDPC'       : simple educational LDPC-style (12,6) code with bit-flipping decoding
fecScheme = 'HAMMING';
repetitionFactor = 3;          % Used only when fecScheme = 'REPETITION'. Use an odd integer.
ldpcMaxIter = 12;              % Used only when fecScheme = 'LDPC'.

% Optional received-signal power spectrum plot
plotRxPowerSpectrum = true;    % true: plot received signal spectrum at one selected SNR
spectrumSnrDb = 12;             % The SNR point used for the spectrum plot. Closest value is used.

% Error logging parameters
maxErrorLogs = 30;             % Maximum number of detailed error cases to store
printErrorLogToConsole = true; % Print detailed error log after the simulation
writeErrorLogToFile = true;    % Save detailed error log to a text file
errorLogFileName = 'iot_error_log.txt';

% Link-layer constants
START_FLAG = uint8(hex2dec('7E'));
END_FLAG   = uint8(hex2dec('7F'));
fragIndex  = uint8(0);         % No fragmentation: the only fragment is index 0
totalFrags = uint8(1);         % No fragmentation: total fragment count is 1

% Application-layer constants and randomized ranges
messageType = uint8(hex2dec('01'));  % 0x01: normal monitoring data
baseTimestamp = uint32(1715000000);
sensorIdRange   = [1, 10];           % Random sensor ID range
waterDepthRange = [0, 600];          % Random water depth in cm
flowSpeedRange  = [0, 300];          % Random flow speed in cm/s
statusValues    = uint8([0, 1, 2]);  % 0x00 normal, 0x01 warning, 0x02 sensor abnormal

% Spectrum SNR index. If spectrumSnrDb is not exactly in snrVec, use the closest SNR.
[~, spectrumSnrIndex] = min(abs(snrVec - spectrumSnrDb));
spectrumRxSymbols = [];
spectrumActualSnr = snrVec(spectrumSnrIndex);

fprintf('Modulation:        %s\n', modulationScheme);
fprintf('FEC scheme:        %s\n', upper(fecScheme));
if strcmpi(fecScheme, 'REPETITION')
    fprintf('Repetition factor: %d\n', repetitionFactor);
end
if strcmpi(fecScheme, 'LDPC')
    fprintf('LDPC decoder iterations: %d\n', ldpcMaxIter);
end
fprintf('SNR range:         %d to %d dB\n\n', snrVec(1), snrVec(end));

%% Main simulation
berVec = zeros(size(snrVec));       % Bit Error Rate after receiver-side FEC decoding
merVec = zeros(size(snrVec));       % Message Error Rate, based on decoded message mismatch

% Detailed error logs. Each entry records the Tx/Rx processing chain
% for transmissions with bit error, frame-structure error, or message error.
errorLogs = {};
examplePrinted = false;
messageIDCounter = uint16(1);

for iSNR = 1:numel(snrVec)
    snr = snrVec(iSNR);

    bitErr = 0;
    bitCnt = 0;
    msgErr = 0;

    for loopIdx = 1:numLoops
        % ---------------- Application-layer random message generation ----------------
        sensorID   = uint8(randi(sensorIdRange));
        timestamp  = uint32(double(baseTimestamp) + (iSNR-1)*numLoops + loopIdx);
        waterDepth = uint16(randi(waterDepthRange));
        flowSpeed  = uint16(randi(flowSpeedRange));
        status     = statusValues(randi(numel(statusValues)));

        appMsgBytes = build_sensor_message(messageType, sensorID, timestamp, ...
                                           waterDepth, flowSpeed, status);
        txMsg = parse_sensor_message(appMsgBytes);

        % Since MTU > message length, the whole application message is put into one frame.
        messageID = messageIDCounter;
        messageIDCounter = uint16(mod(double(messageIDCounter), 65535) + 1);

        txFrameBytes = build_frame(START_FLAG, END_FLAG, messageID, fragIndex, ...
                                   totalFrags, appMsgBytes);
        txFrameBits = bytes_to_bits(txFrameBytes);

        % ---------------- Optional FEC encoding ----------------
        [txChannelBits, fecInfo] = fec_encode(txFrameBits, fecScheme, repetitionFactor);

        % ---------------- Modulation ----------------
        txSymbols = modulate_bits(txChannelBits, modulationScheme);

        % ---------------- AWGN channel ----------------
        rxSymbols = add_awgn(txSymbols, snr);
        if plotRxPowerSpectrum && isempty(spectrumRxSymbols) && iSNR == spectrumSnrIndex && loopIdx == 1
            spectrumRxSymbols = rxSymbols;
        end

        % ---------------- Receiver demodulation ----------------
        rxChannelBits = demodulate_bits(rxSymbols, modulationScheme, numel(txChannelBits));

        % ---------------- Optional FEC decoding ----------------
        rxFrameBits = fec_decode(rxChannelBits, fecInfo, fecScheme, repetitionFactor, ldpcMaxIter);
        rxFrameBits = rxFrameBits(1:numel(txFrameBits));

        % BER is calculated by comparing the recovered frame bits with
        % the original transmitted frame bits.
        bitErrThis = sum(rxFrameBits ~= txFrameBits);
        hasBitError = bitErrThis > 0;
        bitErr = bitErr + bitErrThis;
        bitCnt = bitCnt + numel(txFrameBits);

        rxFrameBytes = bits_to_bytes(rxFrameBits);
        [frameOK, rxPayloadBytes, frameErrorReason] = parse_frame(rxFrameBytes, START_FLAG, END_FLAG);

        rxMsg = [];
        sameMsg = false;
        hasFrameStructureError = ~frameOK;
        hasMsgError = false;

        if ~frameOK
            msgErr = msgErr + 1;
            hasMsgError = true;
        else
            try
                rxMsg = parse_sensor_message(rxPayloadBytes);
                sameMsg = isequal(rxMsg, txMsg);
            catch parseErr
                sameMsg = false;
                frameOK = false;
                frameErrorReason = ['Payload parse failed: ', parseErr.message];
                hasFrameStructureError = true;
            end

            if ~sameMsg
                msgErr = msgErr + 1;
                hasMsgError = true;
            end
        end

        % Save detailed processing-chain log if any error occurs.
        if (hasBitError || hasFrameStructureError || hasMsgError) && numel(errorLogs) < maxErrorLogs
            errorLogs{end+1} = build_error_log_entry( ...
                numel(errorLogs)+1, snr, loopIdx, ...
                hasBitError, hasFrameStructureError, hasMsgError, bitErrThis, frameErrorReason, ...
                appMsgBytes, txFrameBytes, txFrameBits, txChannelBits, ...
                rxChannelBits, rxFrameBits, rxFrameBytes, rxPayloadBytes, ...
                txMsg, rxMsg, fecScheme, modulationScheme); %#ok<SAGROW>
        end

        if ~examplePrinted && frameOK && sameMsg && iSNR == numel(snrVec)
            fprintf('Example decoded message at SNR = %d dB:\n', snr);
            print_message_fields(rxMsg, '  ');
            fprintf('\n');
            examplePrinted = true;
        end
    end

    berVec(iSNR) = bitErr / bitCnt;
    merVec(iSNR) = msgErr / numLoops;

    fprintf('SNR = %3d dB | BER = %.4e | MER = %.4e\n', ...
        snr, berVec(iSNR), merVec(iSNR));
end

%% Detailed error log output
if isempty(errorLogs)
    fprintf('\nNo bit/frame-structure/message error was recorded under the current settings.\n');
else
    logText = strjoin(errorLogs, newline);

    if printErrorLogToConsole
        fprintf('\n================ Detailed Error Log ================\n');
        fprintf('%s\n', logText);
        fprintf('================ End of Error Log ==================\n');
    end

    if writeErrorLogToFile
        fid = fopen(errorLogFileName, 'w');
        if fid == -1
            warning('Could not open %s for writing.', errorLogFileName);
        else
            fprintf(fid, '%s\n', logText);
            fclose(fid);
            fprintf('Detailed error log saved to: %s\n', errorLogFileName);
        end
    end
end

%% BER / MER curves
figure;
semilogy(snrVec, berVec, '-o', 'LineWidth', 1.2); hold on;
semilogy(snrVec, merVec, '-^', 'LineWidth', 1.2);
xlabel('SNR (dB)');
ylabel('Error Rate');
title(['IoT Flood Monitoring Protocol over AWGN, ', modulationScheme, ...
       ', FEC=', upper(fecScheme)]);
legend('BER: bit error rate', 'MER: message error rate', 'Location', 'southwest');
grid on;

%% Optional received-signal power spectrum
if plotRxPowerSpectrum && ~isempty(spectrumRxSymbols)
    plot_power_spectrum(spectrumRxSymbols, spectrumActualSnr, modulationScheme, fecScheme);
end

%% ==========================================================
%  Helper functions
%% ==========================================================

function msgBytes = build_sensor_message(messageType, sensorID, timestamp, waterDepth, flowSpeed, status)
% Build the application-layer sensor message.
% Field order:
% [Message Type][Sensor ID][Timestamp][Water Depth][Flow Speed][Sensor Status]
%
% Multi-byte fields are encoded in big-endian order for clarity.
    msgBytes = [ ...
        uint8(messageType), ...
        uint8(sensorID), ...
        uint32_to_bytes_be(timestamp), ...
        uint16_to_bytes_be(waterDepth), ...
        uint16_to_bytes_be(flowSpeed), ...
        uint8(status) ...
    ];
end

function frameBytes = build_frame(startFlag, endFlag, messageID, fragIndex, totalFrags, payloadBytes)
% Build one link-layer frame without CRC.
% Frame format:
% [Start Flag][Message ID][Fragment Index][Total Fragments][Frame Payload][End Flag]
    headerAndPayload = [ ...
        uint16_to_bytes_be(messageID), ...
        uint8(fragIndex), ...
        uint8(totalFrags), ...
        uint8(payloadBytes) ...
    ];

    frameBytes = [uint8(startFlag), headerAndPayload, uint8(endFlag)];
end

function [ok, payloadBytes, reason] = parse_frame(frameBytes, startFlag, endFlag)
% Start Flag, End Flag, Fragment Index, Total Fragments, and payload length.
    ok = false;
    payloadBytes = uint8([]);
    reason = '';

    % Expected frame length:
    % Start(1) + MessageID(2) + FragIndex(1) + TotalFrags(1) + Payload(11) + End(1)
    expectedFrameLen = 17;
    expectedPayloadLen = 11;

    if numel(frameBytes) ~= expectedFrameLen
        reason = sprintf('Invalid frame length: expected %d bytes, got %d bytes', ...
                         expectedFrameLen, numel(frameBytes));
        return;
    end

    if frameBytes(1) ~= startFlag
        reason = sprintf('Invalid Start Flag: expected 0x%02X, got 0x%02X', startFlag, frameBytes(1));
        return;
    end

    if frameBytes(end) ~= endFlag
        reason = sprintf('Invalid End Flag: expected 0x%02X, got 0x%02X', endFlag, frameBytes(end));
        return;
    end

    body = frameBytes(2:end-1);       % Message ID + Frag info + Payload
    fragIndex = body(3);
    totalFrags = body(4);

    if fragIndex ~= 0 || totalFrags ~= 1
        reason = sprintf('Invalid fragment fields: Fragment Index=%d, Total Fragments=%d', ...
                         fragIndex, totalFrags);
        return;
    end

    payloadBytes = body(5:end);
    if numel(payloadBytes) ~= expectedPayloadLen
        reason = sprintf('Invalid payload length: expected %d bytes, got %d bytes', ...
                         expectedPayloadLen, numel(payloadBytes));
        payloadBytes = uint8([]);
        return;
    end

    ok = true;
end

function msg = parse_sensor_message(msgBytes)
% Decode the application-layer sensor message.
    if numel(msgBytes) ~= 11
        error('Invalid sensor message length. Expected 11 bytes.');
    end

    msg.messageType = msgBytes(1);
    msg.sensorID    = msgBytes(2);
    msg.timestamp   = bytes_be_to_uint32(msgBytes(3:6));
    msg.waterDepth  = bytes_be_to_uint16(msgBytes(7:8));
    msg.flowSpeed   = bytes_be_to_uint16(msgBytes(9:10));
    msg.status      = msgBytes(11);
end

function print_message_fields(msg, prefix)
% Print parsed message fields.
    if nargin < 2
        prefix = '';
    end
    fprintf('%sMessage Type : 0x%02X\n', prefix, msg.messageType);
    fprintf('%sSensor ID    : 0x%02X\n', prefix, msg.sensorID);
    fprintf('%sTimestamp    : %u\n', prefix, msg.timestamp);
    fprintf('%sWater Depth  : %u cm\n', prefix, msg.waterDepth);
    fprintf('%sFlow Speed   : %u cm/s\n', prefix, msg.flowSpeed);
    fprintf('%sStatus       : 0x%02X\n', prefix, msg.status);
end

function bits = bytes_to_bits(bytes)
% Convert uint8 bytes to a column vector of bits, MSB first.
    bytes = uint8(bytes(:));
    bits = zeros(numel(bytes) * 8, 1);
    idx = 1;
    for i = 1:numel(bytes)
        for b = 7:-1:0
            bits(idx) = bitget(bytes(i), b + 1);
            idx = idx + 1;
        end
    end
end

function bytes = bits_to_bytes(bits)
% Convert a bit vector to uint8 bytes, MSB first.
    bits = bits(:);
    if mod(numel(bits), 8) ~= 0
        error('Bit length must be a multiple of 8 for byte conversion.');
    end

    numBytes = numel(bits) / 8;
    bytes = zeros(1, numBytes, 'uint8');

    for i = 1:numBytes
        byteBits = bits((i-1)*8+1:i*8);
        value = uint8(0);
        for b = 1:8
            value = bitor(bitshift(value, 1), uint8(byteBits(b)));
        end
        bytes(i) = value;
    end
end

function symbols = modulate_bits(bits, scheme)
% Modulate bits using BPSK or 4QAM.
    bits = bits(:);
    switch upper(scheme)
        case 'BPSK'
            % 0 -> -1, 1 -> +1
            symbols = 2 * double(bits) - 1;

        case '4QAM'
            % 4QAM/QPSK mapping with unit average power.
            % Pair: [bI bQ]
            if mod(numel(bits), 2) ~= 0
                bits = [bits; 0]; %#ok<AGROW>
            end
            pairs = reshape(bits, 2, []).';
            iPart = 2 * double(pairs(:,1)) - 1;
            qPart = 2 * double(pairs(:,2)) - 1;
            symbols = (iPart + 1i*qPart) / sqrt(2);

        otherwise
            error('Unsupported modulation scheme. Use BPSK or 4QAM.');
    end
end

function bits = demodulate_bits(symbols, scheme, expectedNumBits)
% Hard-decision demodulation.
    switch upper(scheme)
        case 'BPSK'
            bits = real(symbols(:)) >= 0;
            bits = double(bits);
            bits = bits(1:expectedNumBits);

        case '4QAM'
            symbols = symbols(:);
            bI = real(symbols) >= 0;
            bQ = imag(symbols) >= 0;
            pairBits = [double(bI), double(bQ)].';
            bits = pairBits(:);
            bits = bits(1:expectedNumBits);

        otherwise
            error('Unsupported modulation scheme. Use BPSK or 4QAM.');
    end
end

function rx = add_awgn(tx, snrDb)
% Add complex or real AWGN based on measured signal power.
    sigPower = mean(abs(tx).^2);
    snrLinear = 10^(snrDb/10);
    noisePower = sigPower / snrLinear;

    if isreal(tx)
        noise = sqrt(noisePower) * randn(size(tx));
    else
        noise = sqrt(noisePower/2) * (randn(size(tx)) + 1i*randn(size(tx)));
    end

    rx = tx + noise;
end

function [encodedBits, info] = fec_encode(dataBits, fecScheme, repetitionFactor)
% Dispatch FEC encoding and store metadata needed by the decoder.
    dataBits = dataBits(:);
    info.originalLength = numel(dataBits);
    info.scheme = upper(fecScheme);

    switch upper(fecScheme)
        case 'NONE'
            encodedBits = dataBits;
            info.paddedLength = numel(dataBits);

        case 'HAMMING'
            [encodedBits, paddedLength] = hamming74_encode(dataBits);
            info.paddedLength = paddedLength;

        case 'REPETITION'
            validate_repetition_factor(repetitionFactor);
            encodedBits = repetition_encode(dataBits, repetitionFactor);
            info.paddedLength = numel(dataBits);

        case 'LDPC'
            [encodedBits, paddedLength] = simple_ldpc_encode(dataBits);
            info.paddedLength = paddedLength;

        otherwise
            error('Unsupported FEC scheme. Use NONE, HAMMING, REPETITION, or LDPC.');
    end
end

function decodedBits = fec_decode(rxBits, info, fecScheme, repetitionFactor, ldpcMaxIter)
% Dispatch FEC decoding.
    rxBits = rxBits(:);

    switch upper(fecScheme)
        case 'NONE'
            decodedBits = rxBits;

        case 'HAMMING'
            decodedBits = hamming74_decode(rxBits);

        case 'REPETITION'
            validate_repetition_factor(repetitionFactor);
            decodedBits = repetition_decode(rxBits, repetitionFactor);

        case 'LDPC'
            decodedBits = simple_ldpc_decode(rxBits, ldpcMaxIter);

        otherwise
            error('Unsupported FEC scheme. Use NONE, HAMMING, REPETITION, or LDPC.');
    end

    % Remove FEC padding and return at least originalLength bits.
    if numel(decodedBits) < info.originalLength
        error('Decoded bit length is shorter than the original frame length.');
    end
    decodedBits = decodedBits(1:info.originalLength);
end

function [encodedBits, paddedLength] = hamming74_encode(dataBits)
% Hamming(7,4) encoder.
% Data bits per block: [d1 d2 d3 d4]
% Codeword layout: [p1 p2 d1 p3 d2 d3 d4]
    dataBits = dataBits(:);
    if mod(numel(dataBits), 4) ~= 0
        padLen = 4 - mod(numel(dataBits), 4);
        dataBits = [dataBits; zeros(padLen,1)]; %#ok<AGROW>
    end
    paddedLength = numel(dataBits);

    blocks = reshape(dataBits, 4, []).';
    encoded = zeros(size(blocks,1), 7);

    for i = 1:size(blocks,1)
        d1 = blocks(i,1);
        d2 = blocks(i,2);
        d3 = blocks(i,3);
        d4 = blocks(i,4);

        p1 = mod(d1 + d2 + d4, 2);
        p2 = mod(d1 + d3 + d4, 2);
        p3 = mod(d2 + d3 + d4, 2);

        encoded(i,:) = [p1 p2 d1 p3 d2 d3 d4];
    end

    encodedBits = encoded.';
    encodedBits = encodedBits(:);
end

function decodedBits = hamming74_decode(rxBits)
% Hamming(7,4) hard-decision decoder.
% It corrects one bit error per 7-bit codeword.
    rxBits = rxBits(:);
    if mod(numel(rxBits), 7) ~= 0
        error('Hamming(7,4) input length must be a multiple of 7.');
    end

    blocks = reshape(rxBits, 7, []).';
    decoded = zeros(size(blocks,1), 4);

    for i = 1:size(blocks,1)
        c = blocks(i,:);

        s1 = mod(c(1) + c(3) + c(5) + c(7), 2);
        s2 = mod(c(2) + c(3) + c(6) + c(7), 2);
        s3 = mod(c(4) + c(5) + c(6) + c(7), 2);

        errorPosition = s1 * 1 + s2 * 2 + s3 * 4;

        if errorPosition >= 1 && errorPosition <= 7
            c(errorPosition) = 1 - c(errorPosition);
        end

        decoded(i,:) = [c(3) c(5) c(6) c(7)];
    end

    decodedBits = decoded.';
    decodedBits = decodedBits(:);
end

function validate_repetition_factor(repetitionFactor)
% Repetition decoding uses majority voting, so an odd repetition factor is preferred.
    if repetitionFactor < 1 || mod(repetitionFactor, 1) ~= 0 || mod(repetitionFactor, 2) == 0
        error('repetitionFactor must be a positive odd integer, such as 3 or 5.');
    end
end

function encodedBits = repetition_encode(dataBits, repetitionFactor)
% Repeat every bit repetitionFactor times.
    dataBits = dataBits(:).';
    encodedBits = repmat(dataBits, repetitionFactor, 1);
    encodedBits = encodedBits(:);
end

function decodedBits = repetition_decode(rxBits, repetitionFactor)
% Majority-vote repetition decoder.
    rxBits = rxBits(:);
    if mod(numel(rxBits), repetitionFactor) ~= 0
        error('Repetition-coded input length must be divisible by repetitionFactor.');
    end
    blocks = reshape(rxBits, repetitionFactor, []).';
    decodedBits = sum(blocks, 2) > (repetitionFactor / 2);
    decodedBits = double(decodedBits(:));
end

function [encodedBits, paddedLength] = simple_ldpc_encode(dataBits)
% Simple educational systematic LDPC-style (12,6) encoder.
%
% This is a lightweight demonstration code used to compare protocol behavior
% under optional LDPC-like FEC without depending on MATLAB Communications Toolbox.
% It uses a sparse parity-check matrix H = [P I], data length k=6,
% codeword length n=12, and parity p = P*d mod 2.
%
% For a standard-compliant LDPC code, replace this function and
% simple_ldpc_decode() with toolbox-based or standard-defined LDPC routines.
    dataBits = dataBits(:);
    k = 6;
    if mod(numel(dataBits), k) ~= 0
        padLen = k - mod(numel(dataBits), k);
        dataBits = [dataBits; zeros(padLen,1)]; %#ok<AGROW>
    end
    paddedLength = numel(dataBits);

    [P, ~] = simple_ldpc_matrices();
    blocks = reshape(dataBits, k, []).';
    encoded = zeros(size(blocks,1), 12);

    for i = 1:size(blocks,1)
        d = blocks(i,:).';
        p = mod(P * d, 2);
        encoded(i,:) = [d; p].';
    end

    encodedBits = encoded.';
    encodedBits = encodedBits(:);
end

function decodedBits = simple_ldpc_decode(rxBits, maxIter)
% Simple hard-decision bit-flipping decoder for the educational LDPC-style code.
% This is intentionally compact and suitable for protocol-level simulation.
    rxBits = rxBits(:);
    n = 12;
    k = 6;
    if mod(numel(rxBits), n) ~= 0
        error('LDPC-coded input length must be a multiple of 12.');
    end

    [~, H] = simple_ldpc_matrices();
    colDegree = sum(H, 1).';
    blocks = reshape(rxBits, n, []).';
    decoded = zeros(size(blocks,1), k);

    for i = 1:size(blocks,1)
        c = blocks(i,:).';
        for iter = 1:maxIter
            syndrome = mod(H * c, 2);
            if all(syndrome == 0)
                break;
            end

            unsatisfied = H.' * syndrome;
            flipMask = unsatisfied > (colDegree / 2);

            % If the threshold rule flips nothing, flip the most suspicious bit.
            if ~any(flipMask)
                [~, maxIdx] = max(unsatisfied);
                flipMask(maxIdx) = true;
            end

            c(flipMask) = 1 - c(flipMask);
        end
        decoded(i,:) = c(1:k).';
    end

    decodedBits = decoded.';
    decodedBits = decodedBits(:);
end

function [P, H] = simple_ldpc_matrices()
% Sparse parity structure for the educational (12,6) LDPC-style code.
% H = [P I]. Encoding uses p = P*d mod 2.
    P = [ ...
        1 1 0 1 0 0; ...
        0 1 1 0 1 0; ...
        1 0 1 0 0 1; ...
        1 0 0 1 1 0; ...
        0 1 0 1 0 1; ...
        0 0 1 0 1 1  ...
    ];
    H = [P eye(6)];
end

function plot_power_spectrum(rxSymbols, snrDb, modulationScheme, fecScheme)
% Plot a normalized frequency-power spectrum of the received symbol sequence.
% Frequency is normalized to cycles/sample because this baseband simulation
% does not define a physical sampling frequency.
    x = rxSymbols(:);
    nfft = 2^nextpow2(max(256, numel(x)));
    X = fftshift(fft(x, nfft));
    powerDb = 10 * log10((abs(X).^2 / nfft) + eps);
    freq = linspace(-0.5, 0.5, nfft);

    figure;
    plot(freq, powerDb, 'LineWidth', 1.2);
    xlabel('Normalized Frequency (cycles/sample)');
    ylabel('Power Spectrum (dB)');
    title(sprintf('Received Signal Power Spectrum at SNR = %d dB, %s, FEC=%s', ...
          snrDb, modulationScheme, upper(fecScheme)));
    grid on;
end

function entry = build_error_log_entry(logIndex, snr, loopIdx, hasBitError, hasFrameError, hasMsgError, ...
    bitErrThis, frameErrorReason, appMsgBytes, txFrameBytes, txFrameBits, txChannelBits, ...
    rxChannelBits, rxFrameBits, rxFrameBytes, rxPayloadBytes, txMsg, rxMsg, fecScheme, modulationScheme)
% Build one detailed text log entry.
    errTypes = {};
    if hasBitError
        errTypes{end+1} = 'BIT'; %#ok<AGROW>
    end
    if hasFrameError
        errTypes{end+1} = 'FRAME_STRUCT'; %#ok<AGROW>
    end
    if hasMsgError
        errTypes{end+1} = 'MESSAGE'; %#ok<AGROW>
    end
    errTypeText = strjoin(errTypes, '+');

    if isempty(frameErrorReason)
        frameErrorReason = 'None';
    end

    txMsgText = sensor_msg_to_text(txMsg);
    if isempty(rxMsg)
        rxMsgText = '<Frame invalid or payload cannot be parsed>';
    else
        rxMsgText = sensor_msg_to_text(rxMsg);
    end

    if isempty(rxPayloadBytes)
        rxPayloadHex = '<empty>';
    else
        rxPayloadHex = bytes_to_hex(rxPayloadBytes);
    end

    entry = sprintf([ ...
        '%02d Error Record | SNR = %d dB | Loop = %d | Error Type = %s | Bit Errors = %d\n', ...
        'Frount end:\n', ...
        '  Message field: %s\n', ...
        '  Message(hex): %s\n', ...
        '  Message -> Frame(hex): %s\n', ...
        '  Frame bits(before FEC): %s\n', ...
        '  Transmitted bits(after %s): %s\n', ...
        'Rare end:\n', ...
        '  Modulation scheme: %s\n', ...
        '  Received bits(before FEC decoding): %s\n', ...
        '  Recovered frame bits(hex): %s\n', ...
        '  Frame error reasons: %s\n', ...
        '  Message received(hex): %s\n', ...
        '  Message received: %s\n'], ...
        logIndex, snr, loopIdx, errTypeText, bitErrThis, ...
        txMsgText, bytes_to_hex(appMsgBytes), bytes_to_hex(txFrameBytes), ...
        bits_to_string(txFrameBits), upper(fecScheme), bits_to_string(txChannelBits), ...
        modulationScheme, bits_to_string(rxChannelBits),  ...
        bytes_to_hex(rxFrameBytes), frameErrorReason, rxPayloadHex, rxMsgText);
end

function text = sensor_msg_to_text(msg)
% Convert parsed sensor message to a compact text form.
    text = sprintf('Type=0x%02X, Sensor=0x%02X, Timestamp=%u, WaterDepth=%u cm, FlowSpeed=%u cm/s, Status=0x%02X', ...
        msg.messageType, msg.sensorID, msg.timestamp, msg.waterDepth, msg.flowSpeed, msg.status);
end

function s = bits_to_string(bits)
% Convert bit vector to a compact string. Long strings are shortened for readability.
    bits = bits(:).';
    maxLen = 220;
    chars = char(bits + '0');
    if numel(chars) > maxLen
        s = [chars(1:maxLen), '...'];
    else
        s = chars;
    end
end

function s = bytes_to_hex(bytes)
% Convert uint8 bytes to spaced uppercase hexadecimal text.
    bytes = uint8(bytes(:));
    if isempty(bytes)
        s = '<empty>';
        return;
    end
    parts = cell(numel(bytes), 1);
    for i = 1:numel(bytes)
        parts{i} = sprintf('%02X', bytes(i));
    end
    s = strjoin(parts, ' ');
end

function bytes = uint16_to_bytes_be(x)
% Convert uint16 to two big-endian bytes.
    x = uint16(x);
    bytes = uint8([bitshift(x, -8), bitand(x, uint16(255))]);
end

function bytes = uint32_to_bytes_be(x)
% Convert uint32 to four big-endian bytes.
    x = uint32(x);
    bytes = uint8([ ...
        bitshift(x, -24), ...
        bitand(bitshift(x, -16), uint32(255)), ...
        bitand(bitshift(x, -8), uint32(255)), ...
        bitand(x, uint32(255)) ...
    ]);
end

function value = bytes_be_to_uint16(bytes)
% Convert two big-endian bytes to uint16.
    bytes = uint8(bytes);
    value = bitor(bitshift(uint16(bytes(1)), 8), uint16(bytes(2)));
end

function value = bytes_be_to_uint32(bytes)
% Convert four big-endian bytes to uint32.
    bytes = uint8(bytes);
    value = bitor( ...
        bitor(bitshift(uint32(bytes(1)), 24), bitshift(uint32(bytes(2)), 16)), ...
        bitor(bitshift(uint32(bytes(3)), 8), uint32(bytes(4))) ...
    );
end
