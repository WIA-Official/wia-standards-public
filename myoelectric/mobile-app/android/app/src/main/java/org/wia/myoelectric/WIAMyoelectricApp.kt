/**
 * WIA Myoelectric Android Application
 *
 * Main application class for the WIA Myoelectric control app.
 * Handles BLE connection to EMG hardware and gesture visualization.
 */

package org.wia.myoelectric

import android.app.Application
import android.bluetooth.BluetoothAdapter
import android.bluetooth.BluetoothManager
import android.content.Context
import android.util.Log

class WIAMyoelectricApp : Application() {

    companion object {
        const val TAG = "WIAMyoelectric"

        // BLE Service and Characteristic UUIDs (WIA Standard)
        const val WIA_EMG_SERVICE_UUID = "12345678-1234-5678-1234-56789abcdef0"
        const val WIA_EMG_DATA_CHAR_UUID = "12345678-1234-5678-1234-56789abcdef1"
        const val WIA_GESTURE_CHAR_UUID = "12345678-1234-5678-1234-56789abcdef2"
        const val WIA_CONFIG_CHAR_UUID = "12345678-1234-5678-1234-56789abcdef3"

        lateinit var instance: WIAMyoelectricApp
            private set
    }

    var bluetoothAdapter: BluetoothAdapter? = null
        private set

    override fun onCreate() {
        super.onCreate()
        instance = this

        initializeBluetooth()
        Log.i(TAG, "WIA Myoelectric App initialized")
    }

    private fun initializeBluetooth() {
        val bluetoothManager = getSystemService(Context.BLUETOOTH_SERVICE) as BluetoothManager
        bluetoothAdapter = bluetoothManager.adapter

        if (bluetoothAdapter == null) {
            Log.e(TAG, "Bluetooth not supported on this device")
        }
    }

    fun isBluetoothEnabled(): Boolean {
        return bluetoothAdapter?.isEnabled == true
    }
}
