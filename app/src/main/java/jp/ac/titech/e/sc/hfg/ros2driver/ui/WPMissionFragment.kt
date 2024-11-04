package jp.ac.titech.e.sc.hfg.ros2driver.ui


import android.os.Bundle
import android.util.Log
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.widget.Button
import android.widget.Toast
import androidx.fragment.app.Fragment
import androidx.fragment.app.activityViewModels
import dji.sdk.keyvalue.key.FlightControllerKey
import dji.sdk.keyvalue.key.KeyTools
import dji.sdk.keyvalue.value.common.EmptyMsg
import dji.sdk.keyvalue.value.common.LocationCoordinate2D
import dji.sdk.keyvalue.value.common.LocationCoordinate3D
import dji.sdk.keyvalue.value.flightcontroller.GPSSignalLevel
import dji.v5.common.callback.CommonCallbacks
import dji.v5.common.error.IDJIError
import dji.v5.manager.KeyManager
import dji.v5.utils.common.ThreadUtil.runOnUiThread
import jp.ac.titech.e.sc.hfg.ros2driver.R
import jp.ac.titech.e.sc.hfg.ros2driver.databinding.FragmentWpmissionBinding
import jp.ac.titech.e.sc.hfg.ros2driver.ros.FC
import jp.ac.titech.e.sc.hfg.ros2driver.ros.RosVM

class WPMissionFragment : Fragment() {
    private val logtag = "WayPointMissionFragment"
    private val rosVM: RosVM by activityViewModels()
    private lateinit var binding: FragmentWpmissionBinding

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        arguments?.let {
        }
    }

    override fun onCreateView(
            inflater: LayoutInflater, container: ViewGroup?,
            savedInstanceState: Bundle?
    ): View? {
        binding = FragmentWpmissionBinding.inflate(inflater, container, false)
        binding.rosVM = rosVM
        binding.lifecycleOwner = viewLifecycleOwner

        ////////////////////////////////////////////////////////////////////////////////////////////

        val openPickerButton: Button = binding.root.findViewById(R.id.btnOpenPicker)
//        val autoLandButton: Button = binding.root.findViewById(R.id.auto_land)
//        val gpsGetButton: Button = binding.root.findViewById(R.id.gps_get)
//        val homeSetButton: Button = binding.root.findViewById(R.id.home_set)
//        val gotoHomeButton: Button = binding.root.findViewById(R.id.goto_home)

        // Set onClick listeners
        openPickerButton.setOnClickListener { openPickerButtonClicked() }
//        autoLandButton.setOnClickListener { autoLandButtonClicked() }
//        gpsGetButton.setOnClickListener { getGPSButtonClicked() }
//        homeSetButton.setOnClickListener { setHomeLocation() }
//        gotoHomeButton.setOnClickListener { gotoHome() }

        ////////////////////////////////////////////////////////////////////////////////////////////

        return binding.root
    }
    ////////////////////////////////////////////////////////////////////////////////////////////////

    private fun openPickerButtonClicked() {
        runOnUiThread {
                    Toast.makeText(requireContext(), "Goto Home", Toast.LENGTH_LONG).show()
                }
    }
//    //landCmdCallback
//    private fun autoLandButtonClicked() {
//        FC.landCmdCallback()
//    }
//    private fun gotoHome(){
//        KeyManager.getInstance().performAction(KeyTools.createKey(FlightControllerKey.KeyStartGoHome), object: CommonCallbacks.CompletionCallbackWithParam<EmptyMsg>{
//            override fun onSuccess(p0: EmptyMsg?) {
//                runOnUiThread {
//                    Toast.makeText(requireContext(), "Goto Home", Toast.LENGTH_LONG)
//                            .show()
//                }
//            }
//
//            override fun onFailure(error: IDJIError) {
//                runOnUiThread {
//                    Toast.makeText(requireContext(), "Failed to set home location: ${error.description()}", Toast.LENGTH_LONG).show()
//                }
//            }
//        })
//    }
//    private fun setHomeLocation() {
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
//                                    Toast.makeText(requireContext(), "lat:${location.latitude},lon:${location.longitude}", Toast.LENGTH_LONG)
//                                            .show()
//                                }
//                                // Create a key for setting the home location
//                                val homeLocationKey = KeyTools.createKey(FlightControllerKey.KeyHomeLocation)
//                                KeyManager.getInstance().setValue(homeLocationKey, location, object : CommonCallbacks.CompletionCallback {
//                                    override fun onSuccess() {
//                                        runOnUiThread {
//                                            Toast.makeText(requireContext(), "Home location set successfully using the current aircraft location.", Toast.LENGTH_LONG).show()
//                                        }
//                                        Log.d("DJI", "Home location set successfully using the current aircraft location.")
//                                    }
//
//                                    override fun onFailure(error: IDJIError) {
//                                        // Handle the failure to set home location
//                                        runOnUiThread {
//                                            Toast.makeText(requireContext(), "Failed to set home location: ${error.description()}", Toast.LENGTH_LONG).show()
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
//                                    Toast.makeText(requireContext(), "Failed to retrieve current aircraft location.", Toast.LENGTH_LONG).show()
//                                }
//                            }
//                        }
//
//                        override fun onFailure(error: IDJIError) {
//                            // Handle failure to retrieve aircraft location
//                            Log.e("DJI", "Failed to retrieve aircraft location: ${error.description()}")
//                            runOnUiThread {
//                                Toast.makeText(requireContext(), "Failed to retrieve current aircraft location.", Toast.LENGTH_LONG).show()
//                            }
//                        }
//                    })
//
//                }else{
//                    // Handle weak GPS signal
//                    Log.e("DJI", "GPS signal is too weak to set home location.")
//                    runOnUiThread {
//                        Toast.makeText(requireContext(), "GPS signal is too weak to set home location.", Toast.LENGTH_LONG).show()
//                    }
//                }
//            }
//
//            override fun onFailure(error: IDJIError) {
//                Log.e("DJI", "Failed to retrieve GPS signal level: ${error.description()}")
//                runOnUiThread {
//                    Toast.makeText(requireContext(), "Failed to retrieve GPS signal level.", Toast.LENGTH_LONG).show()
//                }
//            }
//
//        })
//    }
//
//    private fun getGPSButtonClicked(){
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
//                                    Toast.makeText(requireContext(), "lat:$latitude,lon:$longitude,alt:$altitude", Toast.LENGTH_LONG)
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
//                                Toast.makeText(requireContext(), "Failed to get location.", Toast.LENGTH_LONG)
//                                        .show()
//                            }
//                            Log.e("DJI", "Failed to get location: ${error.description()}")
//                        }
//                    })
//                } else {
//                    // Handle weak GPS signal
//                    runOnUiThread {
//                        Toast.makeText(requireContext(), "GPS signal is too weak.", Toast.LENGTH_LONG)
//                                .show()
//                    }
//                    Log.e("DJI", "GPS signal is too weak.")
//                }
//            }
//
//            override fun onFailure(error: IDJIError) {
//                // Handle failure to retrieve GPS signal level
//                runOnUiThread {
//                    Toast.makeText(requireContext(), "Failed to retrieve GPS signal level.", Toast.LENGTH_LONG)
//                            .show()
//                }
//                Log.e("DJI", "Failed to retrieve GPS signal level: ${error.description()}")
//            }
//        })
//    }
//
//    ////////////////////////////////////////////////////////////////////////////////////////////////

    companion object {
        @JvmStatic
        fun newInstance() =

                WPMissionFragment().apply {
                    arguments = Bundle().apply {
                    }
                }
    }
}