const noble = require('@abandonware/noble');
const fs = require('fs').promises;

// Configuration and Paths
// You can try different bins for testing.
const targetDeviceName = 'NICLA';
// const firmwarePath = '/mnt/dietpi_userdata/edge/.pio/build/nicla_sense_me/firmware.bin';
// const firmwarePath = '/mnt/dietpi_userdata/edge/.pio/build/nicla_sense_me/firmware_blue.bin';
// const firmwarePath = '/mnt/dietpi_userdata/edge/.pio/build/nicla_sense_me/firmware_green_red_bhy2.bin';
// const firmwarePath = '/mnt/dietpi_userdata/edge/.pio/build/nicla_sense_me/firmware_blue_red.bin';
const firmwarePath = '/mnt/dietpi_userdata/edge/.pio/build/nicla_sense_me/firmware_green_red.bin';
// const firmwarePath = '/mnt/dietpi_userdata/edge/.pio/build/nicla_sense_me/firmware_red_green.bin';
const CHUNK_SIZE = 232; // Chunk size for firmware data

// DFU Service and Characteristics UUIDs
const dfuServiceUuid = '34c2e3b834aa11ebadc10242ac120002';
const dfuInternalCharacteristicUuid = '34c2e3b934aa11ebadc10242ac120002';

let dfuInternalCharacteristic = null;

// Initialize noble and listen for state change
noble.on('stateChange', async (state) => {
    if (state === 'poweredOn') {
        console.log('Scanning for BLE devices...');
        await noble.startScanning([], false);
    } else {
        console.log('BLE is not powered on. Stopping scan.');
        await noble.stopScanning();
    }
});

// Discover NICLA device and connect
noble.on('discover', (peripheral) => {
    if (peripheral.advertisement.localName && peripheral.advertisement.localName.includes(targetDeviceName)) {
        console.log(`Found target device: ${peripheral.advertisement.localName} [${peripheral.address}]`);
        noble.stopScanning();
        connectToDevice(peripheral);
    }
});

async function connectToDevice(peripheral) {
    peripheral.connect(error => {
        if (error) {
            console.error(`Connection to ${targetDeviceName} failed: ${error}`);
            return;
        }
        console.log(`Connected to ${targetDeviceName}`);
        discoverServicesAndCharacteristics(peripheral);
    });
}

async function discoverServicesAndCharacteristics(peripheral) {
    peripheral.discoverServices([dfuServiceUuid], (error, services) => {
        if (error) {
            console.error(`Error discovering DFU service: ${error}`);
            return;
        }
        if (services.length === 0) {
            console.error('DFU service not found!');
            return;
        }
        console.log('DFU service discovered.');
        const dfuService = services[0];

        dfuService.discoverCharacteristics([dfuInternalCharacteristicUuid], (error, characteristics) => {
            if (error) {
                console.error(`Error discovering DFU characteristic: ${error}`);
                return;
            }
            dfuInternalCharacteristic = characteristics.find(c => c.uuid === dfuInternalCharacteristicUuid);
            if (dfuInternalCharacteristic) {
                console.log('DFU Internal Characteristic discovered. Starting firmware update...');
                startFirmwareUpdate();
            } else {
                console.error('DFU Internal Characteristic not found!');
            }
        });
    });
}

async function startFirmwareUpdate() {
    try {
        const firmwareData = await readFirmwareFile(firmwarePath);
        console.log(`Firmware data loaded, size: ${firmwareData.length} bytes`);
        const crc8 = calculateCRC8(firmwareData);
        console.log(`Computed 8-bit CRC: ${crc8}`);
        await sendFirmwareChunks(firmwareData, crc8);
    } catch (error) {
        console.error(`Firmware update failed: ${error}`);
    }
}

async function readFirmwareFile(filePath) {
    return fs.readFile(filePath);
}

function calculateCRC8(firmwareData) {
    let crc = 0;
    firmwareData.forEach(byte => crc ^= byte);
    return crc;
}

async function sendFirmwareChunks(firmwareData, crc8) {
    const totalChunks = Math.ceil(firmwareData.length / CHUNK_SIZE);
    console.log(`Total chunks to send: ${totalChunks}`);
    let progress = 0;

    for (let i = 0; i < totalChunks; i++) {
        const isLastChunk = (i === totalChunks - 1);
        const start = i * CHUNK_SIZE;
        const end = Math.min(start + CHUNK_SIZE, firmwareData.length);
        let chunkData = firmwareData.slice(start, end);

        // Adjust chunkHeader
        let chunkHeader = Buffer.alloc(3); // Assuming the first 3 bytes serve as control headers
        if (isLastChunk) {
            // If this is the last chunk, indicate it.
            chunkHeader[0] = 1; // Mark as last chunk
            const bytesLeft = chunkData.length + 1; // Including CRC
            chunkHeader[1] = bytesLeft & 0xff;
            chunkHeader[2] = (bytesLeft >> 8) & 0xff;
            
            // Append the CRC to the last chunk and add necessary padding
            chunkData = Buffer.concat([chunkData, Buffer.from([crc8])]);
            let padding = Buffer.alloc(CHUNK_SIZE + 3 - chunkData.length - 3, 0); // Ensure total size is 235 bytes
            chunkData = Buffer.concat([chunkHeader, chunkData, padding]);
        } else {
            chunkHeader[0] = 0; // Not the last chunk
            chunkHeader[1] = i & 0xff;
            chunkHeader[2] = (i >> 8) & 0xff;
            // Construct the chunk for non-final packets
            chunkData = Buffer.concat([chunkHeader, chunkData]);
        }

        // Ensure each chunk sent is exactly 235 byte.
        if (!isLastChunk) {
            let padding = Buffer.alloc(235 - chunkData.length, 0);
            chunkData = Buffer.concat([chunkData, padding]);
        }

        console.log(`Sending Chunk ${i + 1} of ${totalChunks}`);
        await writeChunk(dfuInternalCharacteristic, chunkData);
        console.log(`Chunk ${i + 1} of ${totalChunks} sent.`);

        progress = ((i + 1) / totalChunks) * 100;
        console.log(`Progress: ${progress.toFixed(2)}%`);

        await delay(10); // Mimic timing behavior as needed
    }
    console.log("Firmware update complete.");
    // Exit the script successfully
    process.exit(0);
}

async function writeChunk(characteristic, chunk) {
    return new Promise((resolve, reject) => {
        characteristic.write(chunk, false, error => {
            if (error) {
                reject(`Write failed: ${error}`);
            } else {
                resolve();
            }
        });
    });
}

function delay(ms) {
    return new Promise(resolve => setTimeout(resolve, ms));
}