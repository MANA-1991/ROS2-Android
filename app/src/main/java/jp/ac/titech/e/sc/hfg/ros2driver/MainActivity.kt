package jp.ac.titech.e.sc.hfg.ros2driver

import android.content.Context
import android.net.wifi.WifiManager
import android.net.wifi.WifiManager.MulticastLock
import android.os.Build
import android.os.Bundle
import android.os.Handler
import android.os.Looper
import android.util.Log
import android.view.View
import android.view.WindowInsets
import android.view.WindowInsetsController
import android.view.WindowManager
import android.widget.Toast
import androidx.activity.viewModels
import androidx.appcompat.app.AppCompatActivity
import androidx.core.content.ContextCompat
import androidx.viewpager2.widget.ViewPager2
import com.google.android.material.tabs.TabLayout
import com.google.android.material.tabs.TabLayoutMediator
import dji.sdk.keyvalue.key.FlightControllerKey
import dji.sdk.keyvalue.key.KeyTools
import dji.sdk.keyvalue.key.ProductKey
import dji.sdk.keyvalue.value.common.EmptyMsg
import dji.sdk.keyvalue.value.common.LocationCoordinate2D
import dji.sdk.keyvalue.value.common.LocationCoordinate3D
import dji.sdk.keyvalue.value.product.ProductType
import dji.v5.common.callback.CommonCallbacks
import dji.v5.common.error.IDJIError
import dji.v5.common.register.DJISDKInitEvent
import dji.v5.common.utils.GeoidManager
import dji.v5.manager.KeyManager
import dji.v5.manager.SDKManager
import dji.v5.manager.interfaces.SDKManagerCallback
import dji.v5.ux.core.communication.DefaultGlobalPreferences
import dji.v5.ux.core.communication.GlobalPreferencesManager
import dji.v5.ux.core.util.UxSharedPreferencesUtil
import jp.ac.titech.e.sc.hfg.ros2driver.databinding.ActivityMainBinding
import jp.ac.titech.e.sc.hfg.ros2driver.ros.FC
import jp.ac.titech.e.sc.hfg.ros2driver.ros.RosVM
import jp.ac.titech.e.sc.hfg.ros2driver.ui.ViewPagerAdapter
import dji.sdk.keyvalue.value.flightcontroller.GPSSignalLevel as GPSSignalLevel



open class MainActivity : AppCompatActivity() {
    private val logTag = "ros2driver"
    private lateinit var binding: ActivityMainBinding
    private val viewPagerAdapter by lazy { ViewPagerAdapter(this) }
    private val rosVM: RosVM by viewModels()
    //private lateinit var locationTextView: TextView
    private lateinit var multicastLock: MulticastLock


//    fun takeoffButtonClicked(view: View) {
//        FC.takeoffCmdCallback()
//    }
//    //landCmdCallback
//    fun autoLandButtonClicked(view: View) {
//        FC.landCmdCallback()
//    }
//    fun gotoHome(view: View){
//        KeyManager.getInstance().performAction(KeyTools.createKey(FlightControllerKey.KeyStartGoHome), object: CommonCallbacks.CompletionCallbackWithParam<EmptyMsg>{
//            override fun onSuccess(p0: EmptyMsg?) {
//                runOnUiThread {
//                    Toast.makeText(applicationContext, "Goto Home", Toast.LENGTH_LONG)
//                            .show()
//                }
//            }
//
//            override fun onFailure(error: IDJIError) {
//                runOnUiThread {
//                    Toast.makeText(applicationContext, "Failed to set home location: ${error.description()}", Toast.LENGTH_LONG).show()
//                }
//            }
//        })
//    }
//    fun setHomeLocation(view: View) {
//        val gpsSignalKey = KeyTools.createKey(FlightControllerKey.KeyGPSSignalLevel)
//        KeyManager.getInstance().getValue(gpsSignalKey, object : CommonCallbacks.CompletionCallbackWithParam<GPSSignalLevel> {
//            override fun onSuccess(gpsSignalLevel: GPSSignalLevel?) {
//                if (gpsSignalLevel != null && gpsSignalLevel != GPSSignalLevel.LEVEL_0) {
//                    val aircraftLocationKey = KeyTools.createKey(FlightControllerKey.KeyAircraftLocation)
//                    KeyManager.getInstance().getValue(aircraftLocationKey, object : CommonCallbacks.CompletionCallbackWithParam<LocationCoordinate2D>{
//                        override fun onSuccess(location: LocationCoordinate2D?) {
//                            if (location != null) {
//                                Log.d("DJI", "Current aircraft location: Latitude = ${location.latitude}, Longitude = ${location.longitude}")
//                                runOnUiThread {
//                                    Toast.makeText(applicationContext, "lat:${location.latitude},lon:${location.longitude}", Toast.LENGTH_LONG)
//                                            .show()
//                                }
//                                // Create a key for setting the home location
//                                val homeLocationKey = KeyTools.createKey(FlightControllerKey.KeyHomeLocation)
//                                KeyManager.getInstance().setValue(homeLocationKey, location, object : CommonCallbacks.CompletionCallback {
//                                    override fun onSuccess() {
//                                        runOnUiThread {
//                                            Toast.makeText(applicationContext, "Home location set successfully using the current aircraft location.", Toast.LENGTH_LONG).show()
//                                        }
//                                        Log.d("DJI", "Home location set successfully using the current aircraft location.")
//                                    }
//
//                                    override fun onFailure(error: IDJIError) {
//                                        // Handle the failure to set home location
//                                        runOnUiThread {
//                                            Toast.makeText(applicationContext, "Failed to set home location: ${error.description()}", Toast.LENGTH_LONG).show()
//                                        }
//                                        Log.e("DJI", "Failed to set home location: ${error.description()}")
//                                    }
//
//                                })
//
//                            }else{
//                                // Handle failure to retrieve current location
//                                Log.e("DJI", "Failed to retrieve current aircraft location. Location is null.")
//                                runOnUiThread {
//                                    Toast.makeText(applicationContext, "Failed to retrieve current aircraft location.", Toast.LENGTH_LONG).show()
//                                }
//                            }
//                        }
//
//                        override fun onFailure(error: IDJIError) {
//                            // Handle failure to retrieve aircraft location
//                            Log.e("DJI", "Failed to retrieve aircraft location: ${error.description()}")
//                            runOnUiThread {
//                                Toast.makeText(applicationContext, "Failed to retrieve current aircraft location.", Toast.LENGTH_LONG).show()
//                            }
//                        }
//                    })
//
//                }else{
//                    // Handle weak GPS signal
//                    Log.e("DJI", "GPS signal is too weak to set home location.")
//                    runOnUiThread {
//                        Toast.makeText(applicationContext, "GPS signal is too weak to set home location.", Toast.LENGTH_LONG).show()
//                    }
//                }
//            }
//
//            override fun onFailure(error: IDJIError) {
//                Log.e("DJI", "Failed to retrieve GPS signal level: ${error.description()}")
//                runOnUiThread {
//                    Toast.makeText(applicationContext, "Failed to retrieve GPS signal level.", Toast.LENGTH_LONG).show()
//                }
//            }
//
//        })
//    }
//
//    fun getGPSButtonClicked(view: View){
//        val gpsSignalKey = KeyTools.createKey(FlightControllerKey.KeyGPSSignalLevel)
//        KeyManager.getInstance().getValue(gpsSignalKey, object : CommonCallbacks.CompletionCallbackWithParam<GPSSignalLevel> {
//            override fun onSuccess(gpsSignalLevel: GPSSignalLevel?) {
//                // Ensure the GPS signal level is strong enough
//                if (gpsSignalLevel != null && gpsSignalLevel != GPSSignalLevel.LEVEL_0) {
//                    // Create a key for setting the home location
//                    val gpsLocationKey = KeyTools.createKey(FlightControllerKey.KeyAircraftLocation3D)
//
//                    KeyManager.getInstance().getValue(gpsLocationKey,  object : CommonCallbacks.CompletionCallbackWithParam<LocationCoordinate3D> {
//                        override fun onSuccess(p0: LocationCoordinate3D?) {
//                            p0?.let{
//                                val latitude = it.latitude
//                                val longitude = it.longitude
//                                val altitude = it.altitude
//                                runOnUiThread {
//                                    Toast.makeText(applicationContext, "lat:$latitude,lon:$longitude,alt:$altitude", Toast.LENGTH_LONG)
//                                            .show()
//                                }
//                                Log.d("DJI", "lat:$latitude,lon:$longitude,alt:$altitude")
//
//                            }
//                        }
//
//                        override fun onFailure(error: IDJIError) {
//                            // Handle the failure to set home location
//                            runOnUiThread {
//                                Toast.makeText(applicationContext, "Failed to get location.", Toast.LENGTH_LONG)
//                                        .show()
//                            }
//                            Log.e("DJI", "Failed to get location: ${error.description()}")
//                        }
//                    })
//                } else {
//                    // Handle weak GPS signal
//                    runOnUiThread {
//                        Toast.makeText(applicationContext, "GPS signal is too weak.", Toast.LENGTH_LONG)
//                                .show()
//                    }
//                    Log.e("DJI", "GPS signal is too weak.")
//                }
//            }
//
//            override fun onFailure(error: IDJIError) {
//                // Handle failure to retrieve GPS signal level
//                runOnUiThread {
//                    Toast.makeText(applicationContext, "Failed to retrieve GPS signal level.", Toast.LENGTH_LONG)
//                            .show()
//                }
//                Log.e("DJI", "Failed to retrieve GPS signal level: ${error.description()}")
//            }
//        })
//    }
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        lifecycle.addObserver(rosVM)
        binding = ActivityMainBinding.inflate(layoutInflater)
        setContentView(binding.root)

        setFullScreen()
        window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)

        rosVM.aircraftConnected.observe(this) {
            if (it == true) rosVM.makeRosNode()
            if (it == false) rosVM.disposeRosNode()
        }

        rosVM.isRosRun.observe(this) {
            if (it == true) Toast.makeText(applicationContext, "Start ROS", Toast.LENGTH_SHORT)
                .show()
            if (it == false) Toast.makeText(applicationContext, "Stop ROS", Toast.LENGTH_SHORT)
                .show()
        }
        val wifiManager = applicationContext.getSystemService(Context.WIFI_SERVICE) as WifiManager
        multicastLock = wifiManager.createMulticastLock("NatNet")
        multicastLock.setReferenceCounted(false)
        multicastLock.acquire()

        registerApp()
    }

    override fun onDestroy() {
        multicastLock.release()
        super.onDestroy()
    }

    private fun setFullScreen() {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.R) {
            window.decorView.windowInsetsController?.apply {
                hide(WindowInsets.Type.systemBars())
                systemBarsBehavior= WindowInsetsController.BEHAVIOR_SHOW_TRANSIENT_BARS_BY_SWIPE
            }
        }
        else {
            window.decorView.apply {
                systemUiVisibility =
                    View.SYSTEM_UI_FLAG_HIDE_NAVIGATION or View.SYSTEM_UI_FLAG_IMMERSIVE_STICKY or View.SYSTEM_UI_FLAG_FULLSCREEN or
                            View.SYSTEM_UI_FLAG_LAYOUT_FULLSCREEN
            }
            window.decorView.setOnSystemUiVisibilityChangeListener() {
                if (it and View.SYSTEM_UI_FLAG_FULLSCREEN == 0) {
                    window.decorView.systemUiVisibility = (View.SYSTEM_UI_FLAG_LAYOUT_STABLE or
                            View.SYSTEM_UI_FLAG_LAYOUT_HIDE_NAVIGATION or
                            View.SYSTEM_UI_FLAG_LAYOUT_FULLSCREEN or
                            View.SYSTEM_UI_FLAG_FULLSCREEN or
                            View.SYSTEM_UI_FLAG_IMMERSIVE or
                            View.SYSTEM_UI_FLAG_HIDE_NAVIGATION)
                }
            }
        }
    }

    private fun setupViews() {
        val viewPager: ViewPager2 = binding.mainViewPager
        viewPager.apply {
            adapter = viewPagerAdapter
            orientation = ViewPager2.ORIENTATION_HORIZONTAL
            offscreenPageLimit = viewPagerAdapter.itemCount
        }
        val tabLayout: TabLayout = binding.tabLayout
        TabLayoutMediator(tabLayout, viewPager) { tab, position ->
            tab.text = viewPagerAdapter.titleIds[position]
            tab.icon =
                ContextCompat.getDrawable(applicationContext, viewPagerAdapter.icons[position])
        }.attach()
    }

    private fun registerApp() {
        SDKManager.getInstance().init(this, object : SDKManagerCallback {
            override fun onRegisterSuccess() {
                Log.i(logTag, "myApp onRegisterSuccess")
                runOnUiThread {
                    Toast.makeText(applicationContext, "Succeed to register", Toast.LENGTH_LONG)
                        .show()
                }
            }

            override fun onRegisterFailure(error: IDJIError) {
                Log.i(logTag, "myApp onRegisterFailure")
                runOnUiThread {
                    Toast.makeText(applicationContext, "Failed to register", Toast.LENGTH_LONG)
                        .show()
                }
            }

            override fun onProductDisconnect(productId: Int) {
                Log.i(logTag, "myApp onProductDisconnect")
            }

            override fun onProductConnect(productId: Int) {
                Log.i(logTag, "myApp onProductConnect")
                runOnUiThread {
                    Toast.makeText(applicationContext, "Product Connected", Toast.LENGTH_LONG)
                        .show()
                    prepareUxActivity()
                    setupViews()
                }

                KeyManager.getInstance().listen(
                    KeyTools.createKey(FlightControllerKey.KeyConnection),
                    this@MainActivity
                ) { _, newValue ->
                    if (newValue == true) {
                        Handler(Looper.getMainLooper()).postDelayed({
                            rosVM.aircraftConnected.postValue(true)
                            rosVM.isAvailableRTK.postValue(
                                KeyManager.getInstance()
                                    .getValue(KeyTools.createKey(ProductKey.KeyProductType)) == ProductType.DJI_MAVIC_3_ENTERPRISE_SERIES
                            )
                        }, 5000)
                    } else {
                        rosVM.aircraftConnected.postValue(false)
                        rosVM.isAvailableRTK.postValue(false)
                    }
                }
            }

            override fun onProductChanged(productId: Int) {
                Log.i(logTag, "myApp onProductChanged")
            }

            override fun onInitProcess(event: DJISDKInitEvent, totalProcess: Int) {
                Log.i(logTag, "myApp onInitProcess")
                if (event == DJISDKInitEvent.INITIALIZE_COMPLETE) {
                    Log.i(logTag, "myApp start registerApp")
                    SDKManager.getInstance().registerApp()
                }
            }

            override fun onDatabaseDownloadProgress(current: Long, total: Long) {
                Log.i(logTag, "myApp onDatabaseDownloadProgress")
            }
        })
    }

    private fun prepareUxActivity() {
        UxSharedPreferencesUtil.initialize(this)
        GlobalPreferencesManager.initialize(DefaultGlobalPreferences(this))
        GeoidManager.getInstance().init(this)
    }
}
