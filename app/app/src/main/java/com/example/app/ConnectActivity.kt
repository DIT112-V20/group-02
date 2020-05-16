package com.example.app

import android.bluetooth.BluetoothAdapter
import android.bluetooth.BluetoothDevice
import android.bluetooth.BluetoothSocket
import android.content.Context
import android.os.*
import androidx.appcompat.app.AppCompatActivity
import android.util.Log
import androidx.annotation.RequiresApi
import kotlinx.android.synthetic.main.activity_connect.*
import org.jetbrains.anko.toast
import java.io.IOException
import java.util.*

private const val TAG = "Group 2 - Debug:"

class ConnectActivity : AppCompatActivity() {

    // Creates a companion object with values
    companion object {
        var m_myUUID: UUID = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB")
        var m_bluetoothSocket: BluetoothSocket? = null
        var m_bluetoothAdapter: BluetoothAdapter? = null
        var m_isConnected: Boolean = false
        var m_address: String? = null
    }

    private var vibrator: Vibrator? = null

    @RequiresApi(Build.VERSION_CODES.Q)
    override fun onCreate(savedInstanceState: Bundle?){
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_connect)
        // Set m_address to the car's MAC-address
        m_address = "FC:F5:C4:0F:87:62"
        // Run the ConnectToDevice method
        ConnectToDevice(this).execute()

        if(m_isConnected){
            toast("Connected to car")
        } else {
            toast("Not connected to car")
        }

        vibrator = getSystemService(Context.VIBRATOR_SERVICE) as Vibrator

        buttonForward.setOnClickListener {
            vibrateDevice(500)
            sendMessage("f")
        }
        buttonBackward.setOnClickListener {
            sendMessage("b")
            vibrateDevice(500)
        }
        buttonLeft.setOnClickListener {
            sendMessage("l")
            vibrateDevice(500)
        }
        buttonRight.setOnClickListener {
            sendMessage("r")
            vibrateDevice(500)
        }
        buttonStop.setOnClickListener {
            sendMessage("§")
            vibrateDevice(500)
        }
        buttonExit.setOnClickListener { disconnect() }
        toggleDriveMode.setOnClickListener{
            if (toggleDriveMode.isChecked) {
                toast("Automatic driving is WIP.")
            }
        }
    }

    // PulseCount should only be 1 or 2.
    @RequiresApi(Build.VERSION_CODES.Q)
    private fun vibrateDevice(duration: Long) {
        val effect = VibrationEffect.createOneShot(duration, VibrationEffect.DEFAULT_AMPLITUDE)
        vibrator!!.vibrate(effect)
    }

    private fun sendMessage(message: String) {
        if (m_bluetoothSocket != null) {
            try {
                m_bluetoothSocket!!.outputStream.write(message.toByteArray())
            } catch (e: IOException) {
                Log.e(TAG, "Error writing message")
            }
        }
    }

    private fun disconnect() {
        if (m_bluetoothSocket != null) {
            try {
                m_bluetoothSocket!!.close()
                m_bluetoothSocket = null
                m_isConnected = false
            } catch (e: IOException) {
                e.printStackTrace()
            }
        }
        finish()
    }

    //Class in charge of connecting the device with the car
    private class ConnectToDevice(c: Context) : AsyncTask<Void, Void, String>(){

        private var connectSuccess: Boolean = true
        private val context: Context = c

        override fun onPreExecute() {
            super.onPreExecute()
        }
        //Connect device to car
        override fun doInBackground(vararg params: Void?): String? {
            try {
                if (m_bluetoothSocket == null || !m_isConnected){
                    m_bluetoothAdapter = BluetoothAdapter.getDefaultAdapter()

                    // Creates a object representing the Bluetooth device with matching MAC Address
                    val device: BluetoothDevice = m_bluetoothAdapter!!.getRemoteDevice(m_address)

                    m_bluetoothSocket = device.createInsecureRfcommSocketToServiceRecord(m_myUUID)
                    // Stop looking for other devices to save battery
                    BluetoothAdapter.getDefaultAdapter().cancelDiscovery()
                    // Connect to the found Bluetooth socket
                    m_bluetoothSocket!!.connect()
                }
            } catch (e: IOException) {
                connectSuccess = false
                e.printStackTrace()
            }
            return null
        }
      
        override fun onPostExecute(result: String?) {
            super.onPostExecute(result)
            if(!connectSuccess){
                Log.i(TAG, "could not connect")
            } else {
                m_isConnected = true
            }
        }
    }
}