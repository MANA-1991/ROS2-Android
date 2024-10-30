package jp.ac.titech.e.sc.hfg.ros2driver

import android.app.Application
import android.content.Context

class Ros2Driver : Application() {
    override fun attachBaseContext(base: Context?) {
        super.attachBaseContext(base)
        com.secneo.sdk.Helper.install(this)
    }
}