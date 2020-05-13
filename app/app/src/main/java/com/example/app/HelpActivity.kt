package com.example.app

import android.content.Intent
import android.os.Bundle
import android.view.View
import androidx.appcompat.app.AppCompatActivity
import kotlinx.android.synthetic.main.activity_help.*

class HelpActivity : AppCompatActivity() {

    override fun onCreate(savedInstanceState: Bundle?){
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_help)

        buttonExit.setOnClickListener{back()}
    }

    private fun back() {
        val intent = Intent(this, MainActivity::class.java)
        startActivity(intent)
    }
}