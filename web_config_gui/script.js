// script.js
document.addEventListener('DOMContentLoaded', () => {
    const connectButton = document.getElementById('connect-button');
    const disconnectButton = document.getElementById('disconnect-button');
    const connectionStatus = document.getElementById('connection-status');
    const dataSection = document.getElementById('data-section');
    const settingsSection = document.getElementById('settings-section');

    // Live Data Spans
    const accelXSpan = document.getElementById('accelX');
    const accelYSpan = document.getElementById('accelY');
    const accelZSpan = document.getElementById('accelZ');
    const gyroXSpan = document.getElementById('gyroX');
    const gyroYSpan = document.getElementById('gyroY');
    const gyroZSpan = document.getElementById('gyroZ');

    const rollSpan = document.getElementById('roll');
    const pitchSpan = document.getElementById('pitch');
    const yawSpan = document.getElementById('yaw');

    const rcSpans = [];
    for (let i = 1; i <= 8; i++) {
        rcSpans.push(document.getElementById(`rc${i}`));
    }

    const motorSpans = [];
    for (let i = 1; i <= 4; i++) {
        motorSpans.push(document.getElementById(`motor${i}`));
    }

    // PID Inputs
    const rollPInput = document.getElementById('rollP');
    const rollIInput = document.getElementById('rollI');
    const rollDInput = document.getElementById('rollD');
    const pitchPInput = document.getElementById('pitchP');
    const pitchIInput = document.getElementById('pitchI');
    const pitchDInput = document.getElementById('pitchD');
    const yawPInput = document.getElementById('yawP');
    const yawIInput = document.getElementById('yawI');
    const yawDInput = document.getElementById('yawD');

    // Buttons
    const getPidButton = document.getElementById('get-pid-button');
    const setPidButton = document.getElementById('set-pid-button');
    const getSettingButton = document.getElementById('get-setting-button');
    const setSettingButton = document.getElementById('set-setting-button');
    const saveSettingsButton = document.getElementById('save-settings-button');
    const resetSettingsButton = document.getElementById('reset-settings-button');
    const rebootFcButton = document.getElementById('reboot-fc-button');

    // Generic Setting Inputs
    const settingKeyInput = document.getElementById('settingKey');
    const settingValueInput = document.getElementById('settingValue');
    const settingResultSpan = document.getElementById('settingResult');

    let port;
    let reader;
    let writer;
    let mspParser; // Will hold our MSP parser instance
    let isConnected = false;

    // MSP Command IDs (from firmware)
    const MSP_API_VERSION = 1;
    const MSP_FC_VARIANT = 2;
    const MSP_FC_VERSION = 3;
    const MSP_BOARD_INFO = 4;
    const MSP_BUILD_INFO = 5;
    const MSP_REBOOT = 6;
    const MSP_STATUS = 7;
    const MSP_MEM_STATS = 8;
    const MSP_GET_SETTING = 9;
    const MSP_SET_SETTING = 10;
    const MSP_PID = 11;
    const MSP_RAW_IMU = 102;
    const MSP_MOTOR = 104;
    const MSP_RC = 105;
    const MSP_ATTITUDE = 108;
    const MSP_SET_PID = 202;
    const MSP_EEPROM_WRITE = 200;
    const MSP_RESET_SETTINGS = 201;

    // PID Scaling Factor (from firmware config.h)
    const PID_SCALE_FACTOR = 100.0; // Example, adjust based on actual firmware value

    // MSP Parser State
    const MSP_IDLE = 0;
    const MSP_HEADER_START = 1;
    const MSP_HEADER_M = 2;
    const MSP_HEADER_DIR = 3; // '>' for response, '<' for command
    const MSP_HEADER_SIZE = 4;
    const MSP_HEADER_CMD = 5;
    const MSP_PAYLOAD = 6;
    const MSP_CRC = 7;

    let mspState = MSP_IDLE;
    let mspPayloadSize = 0;
    let mspCommandID = 0;
    let mspCRC = 0;
    let mspPayloadBuffer = [];
    let mspPayloadIndex = 0;

    function parseMspChar(char) {
        switch (mspState) {
            case MSP_IDLE:
                if (char === 0x24) { // '$'
                    mspState = MSP_HEADER_START;
                }
                break;
            case MSP_HEADER_START:
                if (char === 0x4D) { // 'M'
                    mspState = MSP_HEADER_DIR;
                } else {
                    mspState = MSP_IDLE;
                }
                break;
            case MSP_HEADER_DIR:
                if (char === 0x3E) { // '>' (response)
                    mspState = MSP_HEADER_SIZE;
                } else if (char === 0x3C) { // '<' (command, not expected here for incoming)
                    mspState = MSP_IDLE; // Should not happen for incoming response
                } else {
                    mspState = MSP_IDLE;
                }
                break;
            case MSP_HEADER_SIZE:
                mspPayloadSize = char;
                mspCRC = char;
                mspPayloadIndex = 0;
                mspPayloadBuffer = new Uint8Array(mspPayloadSize);
                mspState = MSP_HEADER_CMD;
                break;
            case MSP_HEADER_CMD:
                mspCommandID = char;
                mspCRC ^= char;
                mspState = (mspPayloadSize > 0) ? MSP_PAYLOAD : MSP_CRC;
                break;
            case MSP_PAYLOAD:
                mspPayloadBuffer[mspPayloadIndex++] = char;
                mspCRC ^= char;
                if (mspPayloadIndex >= mspPayloadSize) {
                    mspState = MSP_CRC;
                }
                break;
            case MSP_CRC:
                if (mspCRC === char) {
                    processMspMessage(mspCommandID, mspPayloadBuffer);
                } else {
                    console.warn('MSP CRC mismatch!');
                }
                mspState = MSP_IDLE;
                break;
            default:
                mspState = MSP_IDLE;
                break;
        }
    }

    function processMspMessage(commandID, payload) {
        // console.log(`Received MSP command: ${commandID}, payload:`, payload);

        switch (commandID) {
            case MSP_RAW_IMU:
                console.log('RAW_IMU Payload:', payload);
                if (payload.length === 18) {
                    accelXSpan.textContent = new Int16Array(payload.slice(0, 2).buffer)[0];
                    accelYSpan.textContent = new Int16Array(payload.slice(2, 4).buffer)[0];
                    accelZSpan.textContent = new Int16Array(payload.slice(4, 6).buffer)[0];
                    gyroXSpan.textContent = new Int16Array(payload.slice(6, 8).buffer)[0];
                    gyroYSpan.textContent = new Int16Array(payload.slice(8, 10).buffer)[0];
                    gyroZSpan.textContent = new Int16Array(payload.slice(10, 12).buffer)[0];
                    // Mag data (12-18) is ignored for MPU6050
                }
                break;
            case MSP_ATTITUDE:
                console.log('ATTITUDE Payload:', payload);
                if (payload.length === 6) {
                    rollSpan.textContent = (new Int16Array(payload.slice(0, 2).buffer)[0] / 10.0).toFixed(1);
                    pitchSpan.textContent = (new Int16Array(payload.slice(2, 4).buffer)[0] / 10.0).toFixed(1);
                    yawSpan.textContent = new Int16Array(payload.slice(4, 6).buffer)[0];
                }
                break;
            case MSP_RC:
                console.log('RC Payload:', payload);
                if (payload.length === 16) {
                    for (let i = 0; i < 8; i++) {
                        rcSpans[i].textContent = new Int16Array(payload.slice(i * 2, (i * 2) + 2).buffer)[0];
                    }
                }
                break;
            case MSP_MOTOR:
                console.log('MOTOR Payload:', payload);
                if (payload.length === 8) {
                    for (let i = 0; i < 4; i++) {
                        motorSpans[i].textContent = new Uint16Array(payload.slice(i * 2, (i * 2) + 2).buffer)[0];
                    }
                }
                break;
            case MSP_PID:
                if (payload.length === 9) {
                    rollPInput.value = (payload[0] / PID_SCALE_FACTOR).toFixed(2);
                    rollIInput.value = (payload[1] / PID_SCALE_FACTOR).toFixed(2);
                    rollDInput.value = (payload[2] / PID_SCALE_FACTOR).toFixed(2);
                    pitchPInput.value = (payload[3] / PID_SCALE_FACTOR).toFixed(2);
                    pitchIInput.value = (payload[4] / PID_SCALE_FACTOR).toFixed(2);
                    pitchDInput.value = (payload[5] / PID_SCALE_FACTOR).toFixed(2);
                    yawPInput.value = (payload[6] / PID_SCALE_FACTOR).toFixed(2);
                    yawIInput.value = (payload[7] / PID_SCALE_FACTOR).toFixed(2);
                    yawDInput.value = (payload[8] / PID_SCALE_FACTOR).toFixed(2);
                }
                break;
            case MSP_GET_SETTING:
                console.log('MSP_GET_SETTING Payload:', payload);
                // Payload: [key_length (1 byte)] [key_string (variable)] [value_length (1 byte)] [value_string (variable)]
                if (payload.length > 2) {
                    const keyLen = payload[0];
                    console.log('keyLen:', keyLen);
                    const keyBytes = payload.slice(1, 1 + keyLen);
                    console.log('keyBytes:', keyBytes);
                    const key = new TextDecoder().decode(keyBytes);
                    console.log('key:', key);

                    const valueLenIndex = 1 + keyLen;
                    console.log('valueLenIndex:', valueLenIndex);
                    const valueLen = payload[valueLenIndex];
                    console.log('valueLen:', valueLen);
                    const valueBytes = payload.slice(valueLenIndex + 1, valueLenIndex + 1 + valueLen);
                    console.log('valueBytes:', valueBytes);
                    const value = new TextDecoder().decode(valueBytes);
                    console.log('value:', value);

                    settingResultSpan.textContent = `${key}: ${value}`;
                    settingKeyInput.value = key;
                    settingValueInput.value = value;
                } else {
                    settingResultSpan.textContent = 'Error: Invalid setting payload';
                    console.error('MSP_GET_SETTING: Invalid setting payload, length:', payload.length);
                }
                break;
            case MSP_SET_SETTING:
            case MSP_SET_PID:
            case MSP_EEPROM_WRITE:
            case MSP_RESET_SETTINGS:
            case MSP_REBOOT:
                // These commands typically send an empty acknowledgement response
                console.log(`Command ${commandID} acknowledged.`);
                settingResultSpan.textContent = `Command ${commandID} acknowledged.`;
                break;
            case MSP_STATUS:
                // For now, just log status. Can parse more later.
                console.log('MSP_STATUS received:', new TextDecoder().decode(payload));
                break;
            case MSP_MEM_STATS:
                if (payload.length === 4) {
                    const freeHeap = new Uint32Array(payload.buffer)[0];
                    console.log(`Free Heap: ${freeHeap} bytes`);
                }
                break;
            default:
                console.log(`Unhandled MSP command: ${commandID}, payload:`, payload);
                break;
        }
    }

    async function sendMspCommand(commandID, payload = []) {
        if (!writer) {
            console.error('Serial port not open.');
            return;
        }

        const size = payload.length;
        let crc = size;
        crc ^= commandID;

        const buffer = new Uint8Array(6 + size); // $, M, <, size, cmd, payload..., crc
        let i = 0;
        buffer[i++] = 0x24; // $
        buffer[i++] = 0x4D; // M
        buffer[i++] = 0x3C; // < (command)
        buffer[i++] = size;
        buffer[i++] = commandID;

        for (let j = 0; j < size; j++) {
            buffer[i++] = payload[j];
            crc ^= payload[j];
        }
        buffer[i++] = crc;

        // console.log('Sending MSP command:', commandID, 'Payload:', payload, 'Buffer:', buffer);
        await writer.write(buffer);
    }

    async function readSerial() {
        while (port.readable && isConnected) {
            reader = port.readable.getReader();
            try {
                while (true) {
                    const { value, done } = await reader.read();
                    if (done) {
                        // Allow the serial port to be closed later.
                        break;
                    }
                    if (value) {
                        for (const char of value) {
                            console.log('Raw incoming byte:', char); // Log every incoming byte
                            parseMspChar(char);
                        }
                    }
                }
            } catch (error) {
                console.error('Error reading from serial port:', error);
                connectionStatus.textContent = `Error: ${error.message}`;
            } finally {
                reader.releaseLock();
            }
        }
        if (!isConnected) {
            console.log('Serial port reader stopped.');
        }
    }

    async function connectSerial() {
        try {
            port = await navigator.serial.requestPort();
            await port.open({ baudRate: 115200 }); // Standard baud rate for flight controllers
            isConnected = true;
            connectionStatus.textContent = 'Connected';
            connectButton.disabled = true;
            disconnectButton.disabled = false;
            dataSection.style.display = 'block';
            settingsSection.style.display = 'block';

            writer = port.writable.getWriter();
            readSerial();

            // Send MSP handshake to switch FC to MSP mode
            await writer.write(new TextEncoder().encode('$RMSP'));
            console.log('Sent $RMSP handshake.');

            // Start requesting data
            setInterval(() => {
                if (isConnected) {
                    sendMspCommand(MSP_RAW_IMU);
                    sendMspCommand(MSP_ATTITUDE);
                    sendMspCommand(MSP_RC);
                    sendMspCommand(MSP_MOTOR);
                }
            }, 100); // Request data every 100ms
            
        } catch (error) {
            console.error('Serial connection error:', error);
            connectionStatus.textContent = `Error: ${error.message}`;
        }
    }

    async function disconnectSerial() {
        if (writer) {
            await writer.releaseLock();
            writer = null;
        }
        if (reader) {
            await reader.cancel();
            reader = null;
        }
        if (port) {
            await port.close();
            port = null;
        }
        isConnected = false;
        connectionStatus.textContent = 'Disconnected';
        connectButton.disabled = false;
        disconnectButton.disabled = true;
        dataSection.style.display = 'none';
        settingsSection.style.display = 'none';
    }

    connectButton.addEventListener('click', connectSerial);
    disconnectButton.addEventListener('click', disconnectSerial);

    getPidButton.addEventListener('click', () => sendMspCommand(MSP_PID));
    setPidButton.addEventListener('click', async () => {
        const payload = new Uint8Array(9);
        payload[0] = parseFloat(rollPInput.value) * PID_SCALE_FACTOR;
        payload[1] = parseFloat(rollIInput.value) * PID_SCALE_FACTOR;
        payload[2] = parseFloat(rollDInput.value) * PID_SCALE_FACTOR;
        payload[3] = parseFloat(pitchPInput.value) * PID_SCALE_FACTOR;
        payload[4] = parseFloat(pitchIInput.value) * PID_SCALE_FACTOR;
        payload[5] = parseFloat(pitchDInput.value) * PID_SCALE_FACTOR;
        payload[6] = parseFloat(yawPInput.value) * PID_SCALE_FACTOR;
        payload[7] = parseFloat(yawIInput.value) * PID_SCALE_FACTOR;
        payload[8] = parseFloat(yawDInput.value) * PID_SCALE_FACTOR;
        await sendMspCommand(MSP_SET_PID, payload);
        await sendMspCommand(MSP_EEPROM_WRITE); // Save settings after setting PIDs
    });

    getSettingButton.addEventListener('click', async () => {
        const key = settingKeyInput.value;
        if (key) {
            const keyBytes = new TextEncoder().encode(key);
            const payload = new Uint8Array(1 + keyBytes.length);
            payload[0] = keyBytes.length;
            payload.set(keyBytes, 1);
            await sendMspCommand(MSP_GET_SETTING, payload);
        } else {
            settingResultSpan.textContent = 'Error: Setting key cannot be empty.';
        }
    });

    setSettingButton.addEventListener('click', async () => {
        const key = settingKeyInput.value;
        const value = settingValueInput.value;
        if (key && value) {
            const keyBytes = new TextEncoder().encode(key);
            const valueBytes = new TextEncoder().encode(value);
            const payload = new Uint8Array(1 + keyBytes.length + 1 + valueBytes.length);
            payload[0] = keyBytes.length;
            payload.set(keyBytes, 1);
            payload[1 + keyBytes.length] = valueBytes.length;
            payload.set(valueBytes, 1 + keyBytes.length + 1);
            await sendMspCommand(MSP_SET_SETTING, payload);
            await sendMspCommand(MSP_EEPROM_WRITE); // Save settings after setting
        } else {
            settingResultSpan.textContent = 'Error: Setting key and value cannot be empty.';
        }
    });

    saveSettingsButton.addEventListener('click', () => sendMspCommand(MSP_EEPROM_WRITE));
    resetSettingsButton.addEventListener('click', () => {
        if (confirm('Are you sure you want to reset all settings to default? This cannot be undone.')) {
            sendMspCommand(MSP_RESET_SETTINGS);
        }
    });
    rebootFcButton.addEventListener('click', () => sendMspCommand(MSP_REBOOT));

    // Check for Web Serial API support
    if ('serial' in navigator) {
        connectionStatus.textContent = 'Web Serial API supported.';
    } else {
        connectionStatus.textContent = 'Web Serial API NOT supported in this browser. Please use Chrome, Edge, or Opera.';
        connectButton.disabled = true;
    }
});