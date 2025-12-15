/**
 * BLE Manager for WIA EMG Hardware Communication
 */

package org.wia.myoelectric

import android.bluetooth.*
import android.bluetooth.le.*
import android.content.Context
import android.os.Handler
import android.os.Looper
import android.util.Log
import java.util.*

/**
 * Gesture types matching WIA standard
 */
enum class Gesture(val code: Int) {
    REST(0),
    HAND_OPEN(1),
    HAND_CLOSE(2),
    WRIST_FLEXION(3),
    WRIST_EXTENSION(4),
    PINCH(5),
    TRIPOD(6),
    POINT(7)
}

/**
 * EMG data packet from hardware
 */
data class EMGPacket(
    val timestamp: Long,
    val channels: FloatArray,
    val gesture: Gesture,
    val confidence: Float
)

/**
 * Listener for EMG events
 */
interface EMGListener {
    fun onConnected(deviceName: String)
    fun onDisconnected()
    fun onEMGData(packet: EMGPacket)
    fun onGestureDetected(gesture: Gesture, confidence: Float)
    fun onError(message: String)
}

/**
 * BLE Manager for WIA EMG communication
 */
class BLEManager(private val context: Context) {

    companion object {
        private const val TAG = "BLEManager"
        private const val SCAN_TIMEOUT_MS = 10000L
    }

    private var bluetoothGatt: BluetoothGatt? = null
    private var scanning = false
    private val handler = Handler(Looper.getMainLooper())
    private var listener: EMGListener? = null

    private val serviceUUID = UUID.fromString(WIAMyoelectricApp.WIA_EMG_SERVICE_UUID)
    private val emgDataCharUUID = UUID.fromString(WIAMyoelectricApp.WIA_EMG_DATA_CHAR_UUID)
    private val gestureCharUUID = UUID.fromString(WIAMyoelectricApp.WIA_GESTURE_CHAR_UUID)

    fun setListener(listener: EMGListener) {
        this.listener = listener
    }

    fun startScan() {
        val bluetoothAdapter = WIAMyoelectricApp.instance.bluetoothAdapter
        if (bluetoothAdapter == null || !bluetoothAdapter.isEnabled) {
            listener?.onError("Bluetooth not available")
            return
        }

        val scanner = bluetoothAdapter.bluetoothLeScanner
        if (scanner == null) {
            listener?.onError("BLE scanner not available")
            return
        }

        val filters = listOf(
            ScanFilter.Builder()
                .setServiceUuid(android.os.ParcelUuid(serviceUUID))
                .build()
        )

        val settings = ScanSettings.Builder()
            .setScanMode(ScanSettings.SCAN_MODE_LOW_LATENCY)
            .build()

        scanning = true
        scanner.startScan(filters, settings, scanCallback)
        Log.i(TAG, "Started BLE scan")

        handler.postDelayed({
            stopScan()
        }, SCAN_TIMEOUT_MS)
    }

    fun stopScan() {
        if (!scanning) return

        val scanner = WIAMyoelectricApp.instance.bluetoothAdapter?.bluetoothLeScanner
        scanner?.stopScan(scanCallback)
        scanning = false
        Log.i(TAG, "Stopped BLE scan")
    }

    fun connect(device: BluetoothDevice) {
        stopScan()
        bluetoothGatt = device.connectGatt(context, false, gattCallback)
        Log.i(TAG, "Connecting to ${device.name}")
    }

    fun disconnect() {
        bluetoothGatt?.disconnect()
        bluetoothGatt?.close()
        bluetoothGatt = null
    }

    private val scanCallback = object : ScanCallback() {
        override fun onScanResult(callbackType: Int, result: ScanResult) {
            val device = result.device
            Log.i(TAG, "Found device: ${device.name ?: "Unknown"}")
            connect(device)
        }

        override fun onScanFailed(errorCode: Int) {
            listener?.onError("Scan failed: $errorCode")
        }
    }

    private val gattCallback = object : BluetoothGattCallback() {
        override fun onConnectionStateChange(gatt: BluetoothGatt, status: Int, newState: Int) {
            when (newState) {
                BluetoothProfile.STATE_CONNECTED -> {
                    Log.i(TAG, "Connected to GATT server")
                    gatt.discoverServices()
                }
                BluetoothProfile.STATE_DISCONNECTED -> {
                    Log.i(TAG, "Disconnected from GATT server")
                    handler.post { listener?.onDisconnected() }
                }
            }
        }

        override fun onServicesDiscovered(gatt: BluetoothGatt, status: Int) {
            if (status == BluetoothGatt.GATT_SUCCESS) {
                val service = gatt.getService(serviceUUID)
                if (service != null) {
                    enableNotifications(gatt, service)
                    handler.post {
                        listener?.onConnected(gatt.device.name ?: "WIA EMG")
                    }
                }
            }
        }

        override fun onCharacteristicChanged(
            gatt: BluetoothGatt,
            characteristic: BluetoothGattCharacteristic
        ) {
            when (characteristic.uuid) {
                emgDataCharUUID -> parseEMGData(characteristic.value)
                gestureCharUUID -> parseGestureData(characteristic.value)
            }
        }
    }

    private fun enableNotifications(gatt: BluetoothGatt, service: BluetoothGattService) {
        listOf(emgDataCharUUID, gestureCharUUID).forEach { charUUID ->
            service.getCharacteristic(charUUID)?.let { char ->
                gatt.setCharacteristicNotification(char, true)
                char.getDescriptor(UUID.fromString("00002902-0000-1000-8000-00805f9b34fb"))?.let { desc ->
                    desc.value = BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE
                    gatt.writeDescriptor(desc)
                }
            }
        }
    }

    private fun parseEMGData(data: ByteArray) {
        if (data.size < 20) return

        // Parse WIA EMG packet format
        val timestamp = System.currentTimeMillis()
        val channels = FloatArray(4) { i ->
            // Little-endian float parsing
            val offset = i * 4
            java.lang.Float.intBitsToFloat(
                (data[offset].toInt() and 0xFF) or
                ((data[offset + 1].toInt() and 0xFF) shl 8) or
                ((data[offset + 2].toInt() and 0xFF) shl 16) or
                ((data[offset + 3].toInt() and 0xFF) shl 24)
            )
        }

        val gestureCode = data[16].toInt()
        val confidence = (data[17].toInt() and 0xFF) / 255f

        val packet = EMGPacket(
            timestamp = timestamp,
            channels = channels,
            gesture = Gesture.values().getOrElse(gestureCode) { Gesture.REST },
            confidence = confidence
        )

        handler.post { listener?.onEMGData(packet) }
    }

    private fun parseGestureData(data: ByteArray) {
        if (data.size < 2) return

        val gestureCode = data[0].toInt()
        val confidence = (data[1].toInt() and 0xFF) / 255f
        val gesture = Gesture.values().getOrElse(gestureCode) { Gesture.REST }

        handler.post { listener?.onGestureDetected(gesture, confidence) }
    }
}
