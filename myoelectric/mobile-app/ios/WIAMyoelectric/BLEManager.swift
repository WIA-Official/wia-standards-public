/**
 * WIA Myoelectric iOS BLE Manager
 *
 * Handles BLE communication with WIA EMG hardware.
 */

import Foundation
import CoreBluetooth

// MARK: - Data Types

enum Gesture: Int, CaseIterable {
    case rest = 0
    case handOpen = 1
    case handClose = 2
    case wristFlexion = 3
    case wristExtension = 4
    case pinch = 5
    case tripod = 6
    case point = 7

    var displayName: String {
        switch self {
        case .rest: return "Rest"
        case .handOpen: return "Hand Open"
        case .handClose: return "Hand Close"
        case .wristFlexion: return "Wrist Flexion"
        case .wristExtension: return "Wrist Extension"
        case .pinch: return "Pinch"
        case .tripod: return "Tripod"
        case .point: return "Point"
        }
    }
}

struct EMGPacket {
    let timestamp: Date
    let channels: [Float]
    let gesture: Gesture
    let confidence: Float
}

// MARK: - Delegate Protocol

protocol BLEManagerDelegate: AnyObject {
    func bleManagerDidConnect(deviceName: String)
    func bleManagerDidDisconnect()
    func bleManagerDidReceiveEMGData(_ packet: EMGPacket)
    func bleManagerDidDetectGesture(_ gesture: Gesture, confidence: Float)
    func bleManagerDidEncounterError(_ error: String)
}

// MARK: - BLE Manager

class BLEManager: NSObject {

    // WIA Standard UUIDs
    static let wiaServiceUUID = CBUUID(string: "12345678-1234-5678-1234-56789abcdef0")
    static let emgDataCharUUID = CBUUID(string: "12345678-1234-5678-1234-56789abcdef1")
    static let gestureCharUUID = CBUUID(string: "12345678-1234-5678-1234-56789abcdef2")
    static let configCharUUID = CBUUID(string: "12345678-1234-5678-1234-56789abcdef3")

    weak var delegate: BLEManagerDelegate?

    private var centralManager: CBCentralManager!
    private var connectedPeripheral: CBPeripheral?
    private var emgDataCharacteristic: CBCharacteristic?
    private var gestureCharacteristic: CBCharacteristic?

    private(set) var isScanning = false
    private(set) var isConnected = false

    override init() {
        super.init()
        centralManager = CBCentralManager(delegate: self, queue: nil)
    }

    // MARK: - Public Methods

    func startScan() {
        guard centralManager.state == .poweredOn else {
            delegate?.bleManagerDidEncounterError("Bluetooth is not available")
            return
        }

        isScanning = true
        centralManager.scanForPeripherals(
            withServices: [BLEManager.wiaServiceUUID],
            options: [CBCentralManagerScanOptionAllowDuplicatesKey: false]
        )

        // Stop scan after timeout
        DispatchQueue.main.asyncAfter(deadline: .now() + 10) { [weak self] in
            self?.stopScan()
        }
    }

    func stopScan() {
        guard isScanning else { return }
        centralManager.stopScan()
        isScanning = false
    }

    func connect(to peripheral: CBPeripheral) {
        stopScan()
        connectedPeripheral = peripheral
        peripheral.delegate = self
        centralManager.connect(peripheral, options: nil)
    }

    func disconnect() {
        guard let peripheral = connectedPeripheral else { return }
        centralManager.cancelPeripheralConnection(peripheral)
    }

    // MARK: - Private Methods

    private func parseEMGData(_ data: Data) {
        guard data.count >= 20 else { return }

        var channels = [Float]()
        for i in 0..<4 {
            let offset = i * 4
            let value = data.subdata(in: offset..<(offset + 4))
                .withUnsafeBytes { $0.load(as: Float.self) }
            channels.append(value)
        }

        let gestureCode = Int(data[16])
        let confidence = Float(data[17]) / 255.0
        let gesture = Gesture(rawValue: gestureCode) ?? .rest

        let packet = EMGPacket(
            timestamp: Date(),
            channels: channels,
            gesture: gesture,
            confidence: confidence
        )

        DispatchQueue.main.async { [weak self] in
            self?.delegate?.bleManagerDidReceiveEMGData(packet)
        }
    }

    private func parseGestureData(_ data: Data) {
        guard data.count >= 2 else { return }

        let gestureCode = Int(data[0])
        let confidence = Float(data[1]) / 255.0
        let gesture = Gesture(rawValue: gestureCode) ?? .rest

        DispatchQueue.main.async { [weak self] in
            self?.delegate?.bleManagerDidDetectGesture(gesture, confidence: confidence)
        }
    }
}

// MARK: - CBCentralManagerDelegate

extension BLEManager: CBCentralManagerDelegate {

    func centralManagerDidUpdateState(_ central: CBCentralManager) {
        switch central.state {
        case .poweredOn:
            print("Bluetooth is powered on")
        case .poweredOff:
            delegate?.bleManagerDidEncounterError("Bluetooth is powered off")
        case .unauthorized:
            delegate?.bleManagerDidEncounterError("Bluetooth is unauthorized")
        case .unsupported:
            delegate?.bleManagerDidEncounterError("Bluetooth is not supported")
        default:
            break
        }
    }

    func centralManager(_ central: CBCentralManager,
                        didDiscover peripheral: CBPeripheral,
                        advertisementData: [String: Any],
                        rssi RSSI: NSNumber) {
        print("Discovered: \(peripheral.name ?? "Unknown")")
        connect(to: peripheral)
    }

    func centralManager(_ central: CBCentralManager,
                        didConnect peripheral: CBPeripheral) {
        isConnected = true
        peripheral.discoverServices([BLEManager.wiaServiceUUID])
    }

    func centralManager(_ central: CBCentralManager,
                        didDisconnectPeripheral peripheral: CBPeripheral,
                        error: Error?) {
        isConnected = false
        connectedPeripheral = nil
        DispatchQueue.main.async { [weak self] in
            self?.delegate?.bleManagerDidDisconnect()
        }
    }

    func centralManager(_ central: CBCentralManager,
                        didFailToConnect peripheral: CBPeripheral,
                        error: Error?) {
        delegate?.bleManagerDidEncounterError("Failed to connect: \(error?.localizedDescription ?? "Unknown")")
    }
}

// MARK: - CBPeripheralDelegate

extension BLEManager: CBPeripheralDelegate {

    func peripheral(_ peripheral: CBPeripheral,
                    didDiscoverServices error: Error?) {
        guard let service = peripheral.services?.first(where: { $0.uuid == BLEManager.wiaServiceUUID }) else {
            return
        }

        peripheral.discoverCharacteristics([
            BLEManager.emgDataCharUUID,
            BLEManager.gestureCharUUID
        ], for: service)
    }

    func peripheral(_ peripheral: CBPeripheral,
                    didDiscoverCharacteristicsFor service: CBService,
                    error: Error?) {
        guard let characteristics = service.characteristics else { return }

        for char in characteristics {
            switch char.uuid {
            case BLEManager.emgDataCharUUID:
                emgDataCharacteristic = char
                peripheral.setNotifyValue(true, for: char)

            case BLEManager.gestureCharUUID:
                gestureCharacteristic = char
                peripheral.setNotifyValue(true, for: char)

            default:
                break
            }
        }

        DispatchQueue.main.async { [weak self] in
            self?.delegate?.bleManagerDidConnect(deviceName: peripheral.name ?? "WIA EMG")
        }
    }

    func peripheral(_ peripheral: CBPeripheral,
                    didUpdateValueFor characteristic: CBCharacteristic,
                    error: Error?) {
        guard let data = characteristic.value else { return }

        switch characteristic.uuid {
        case BLEManager.emgDataCharUUID:
            parseEMGData(data)

        case BLEManager.gestureCharUUID:
            parseGestureData(data)

        default:
            break
        }
    }
}
