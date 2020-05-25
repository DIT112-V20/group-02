package com.example.app

import android.bluetooth.BluetoothAdapter
import android.bluetooth.BluetoothDevice
import android.bluetooth.BluetoothSocket
import android.content.Context
import android.media.MediaPlayer
import android.os.AsyncTask
import android.os.*
import androidx.appcompat.app.AppCompatActivity
import android.util.Log
import androidx.annotation.RequiresApi
import kotlinx.android.synthetic.main.activity_connect.*
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers.Default
import kotlinx.coroutines.launch
import org.jetbrains.anko.toast
import java.io.IOException
import java.util.*

class ConnectActivity : AppCompatActivity() {

    // Creates a companion object with values
    companion object {
        var m_myUUID: UUID = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB")
        var m_bluetoothSocket: BluetoothSocket? = null
        var m_bluetoothAdapter: BluetoothAdapter? = null
        var m_isConnected: Boolean = false
        var m_address: String? = null

    }

    //private const val TAG = "Group 2 - Debug:"
    private var automaticDriving: Boolean = false
    private var vibrator: Vibrator? = null
    private val t = Thread()

    override fun onCreate(savedInstanceState: Bundle?){
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_connect)
        // Set m_address to the car's MAC-address
        m_address = "FC:F5:C4:0F:87:62"
        // Run the ConnectToDevice method
        ConnectToDevice(this).execute()
        //ContinuousReading(this)



        vibrator = getSystemService(Context.VIBRATOR_SERVICE) as Vibrator

        buttonForward.setOnClickListener { sendMessage("f") }
        buttonBackward.setOnClickListener { sendMessage("b") }
        buttonLeft.setOnClickListener { sendMessage("l") }
        buttonRight.setOnClickListener { sendMessage("r") }
        buttonStop.setOnClickListener { sendMessage("§") }
        buttonAccelerate.setOnClickListener { sendMessage("i") }
        button_decrease_speed.setOnClickListener { sendMessage("d") }
        buttonExit.setOnClickListener { disconnect() }

        toggleDriveMode.setOnClickListener{

            if (toggleDriveMode.isChecked) {
                sendMessage("a")
                automaticDriving = true
                //Creates a Coroutine to run the messageToSound() continously.
                CoroutineScope(Default).launch {
                    messageToSound()
                }
                toast("Automatic driving is active.")

                // Make buttons un-clickable in automatic driving mode.
                buttonForward.isEnabled = false
                buttonBackward.isEnabled = false
                buttonLeft.isEnabled = false
                buttonRight.isEnabled = false
                buttonAccelerate.isEnabled = false
                button_decrease_speed.isEnabled = false
                buttonStop.isEnabled = false

            } else {
                sendMessage("m")
                automaticDriving = false
                toast("Manual driving is active.")

                // Make buttons clickable in manual driving mode.
                buttonForward.isEnabled = true
                buttonBackward.isEnabled = true
                buttonLeft.isEnabled = true
                buttonRight.isEnabled = true
                buttonAccelerate.isEnabled = true
                button_decrease_speed.isEnabled = true
                buttonStop.isEnabled = true

            }
        }
    }

    // PulseCount should only be 1 or 2.
    @RequiresApi(Build.VERSION_CODES.Q)
    private fun vibrateDevice(duration: Long) {
        val effect = VibrationEffect.createOneShot(duration, VibrationEffect.DEFAULT_AMPLITUDE)
        vibrator!!.vibrate(effect)
    }

    /*Sends message over bluetooth.*/
    private fun sendMessage(message: String) {
        if (m_bluetoothSocket != null) {
            try {
                m_bluetoothSocket!!.outputStream.write(message.toByteArray())
            } catch (e: IOException) {
                Log.e("data", "Error writing message")
            }
        }
    }

    /* Takes input from bluetooth and sends the input to the playSound-method.*/
    @RequiresApi(Build.VERSION_CODES.Q)
    private suspend fun messageToSound() {
        var previousMessage: String? = null
        while(automaticDriving)  {
            val soundToPlay = readMessage()
            //Checks if message is equal to the last played message to avoid repetition.
            if(soundToPlay != null && soundToPlay != previousMessage){
                playSound(soundToPlay)
                previousMessage = soundToPlay
            }
        }
    }

    /* Bluetooth byte to String translator. Reads byte input from bluetooth and returns a String.*/
    private suspend fun readMessage() : String? {
        val inputStream = m_bluetoothSocket!!.inputStream
        val buffer = ByteArray(1024)
        var bytes: Int
        var message: String?

        return try {
            bytes = inputStream.read(buffer)
            message = String(buffer, 0, bytes)
            message

        } catch (e: IOException) {
            e.printStackTrace()
            null
        }
    }

    /*Closes the bluetooth socket and the activity*/
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

    /*Takes a String as input and plays corresponding sound.*/
    @RequiresApi(Build.VERSION_CODES.Q)
    private fun playSound(input: String){

        if(input == "f"){
            var drivingForward = MediaPlayer.create(this, R.raw.driving_forward)
            drivingForward!!.start()
        } else if (input == "s"){
            var carStopped = MediaPlayer.create(this, R.raw.car_stopped)
            carStopped!!.start()
        } else if (input == "r"){
            var turningRight = MediaPlayer.create(this, R.raw.turning_right)
            turningRight!!.start()
        } else if (input == "l"){
            var turningLeft = MediaPlayer.create(this, R.raw.turning_left)
            turningLeft!!.start()
        } else if (input == "b"){
            var drivingBackwards = MediaPlayer.create(this, R.raw.driving_backwards)
            drivingBackwards!!.start()
        } else {
            return;
        }

        vibrateDevice(500)
    }

    /*Class in charge of connecting the device with the car.*/
    private class ConnectToDevice(c: Context) : AsyncTask<Void, Void, String>(){

        private var connectSuccess: Boolean = true
        private val context: Context = c

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
                Log.i("data", "could not connect")
            } else {
                m_isConnected = true
            }
        }
    }
}
