package jp.ac.titech.e.sc.hfg.ros2driver.ros


import android.content.Context
import android.util.Log
import android.widget.Toast
import androidx.appcompat.app.AppCompatActivity
import dji.sdk.keyvalue.key.FlightControllerKey
import dji.sdk.keyvalue.key.KeyTools
import dji.sdk.keyvalue.key.ProductKey
import dji.sdk.keyvalue.key.RemoteControllerKey
import dji.sdk.keyvalue.value.common.EmptyMsg
import dji.sdk.keyvalue.value.common.LocationCoordinate3D
import dji.sdk.keyvalue.value.flightcontroller.FlightControlAuthorityChangeReason
import dji.sdk.keyvalue.value.flightcontroller.FlightCoordinateSystem
import dji.sdk.keyvalue.value.flightcontroller.FlightMode
import dji.sdk.keyvalue.value.flightcontroller.RollPitchControlMode
import dji.sdk.keyvalue.value.flightcontroller.VerticalControlMode
import dji.sdk.keyvalue.value.flightcontroller.VirtualStickFlightControlParam
import dji.sdk.keyvalue.value.flightcontroller.YawControlMode
import dji.sdk.keyvalue.value.product.ProductType
import dji.v5.common.callback.CommonCallbacks
import dji.v5.common.callback.CommonCallbacks.CompletionCallbackWithParam
import dji.v5.common.error.IDJIError
import dji.v5.manager.KeyManager
import dji.v5.manager.aircraft.perception.PerceptionManager
import dji.v5.manager.aircraft.perception.data.PerceptionDirection
import dji.v5.manager.aircraft.virtualstick.VirtualStickManager
import dji.v5.manager.aircraft.virtualstick.VirtualStickState
import dji.v5.manager.aircraft.virtualstick.VirtualStickStateListener
import dji.v5.utils.common.ThreadUtil.runOnUiThread


class FC: AppCompatActivity() {


    // State
    private var isVirtualStickEnable = false
    private var isVirtualStickAdvancedModeEnabled = false

    //lambda Function
    private val pilotingCmdCallback =  {  ->

        if (flightState == FlightState.FLIGHT) {

            val param = VirtualStickFlightControlParam()
            // roll pitch control mode
            param.rollPitchControlMode = RollPitchControlMode.VELOCITY
            param.rollPitchCoordinateSystem = FlightCoordinateSystem.BODY

            // yaw control mode
            param.yawControlMode = YawControlMode.ANGULAR_VELOCITY


            // vertical throttle control mode
            param.verticalControlMode = VerticalControlMode.VELOCITY
//            param.verticalThrottle = verticalThrottle

            Log.d(logTag, param.toString())
            stickManager.sendVirtualStickAdvancedParam(param)
        }
    }

    init {

        enableFlightControl()
        enableEmergencyButton()
        enableHomeReturnButton()
        flightState =
                if (keyManager.getValue(KeyTools.createKey(FlightControllerKey.KeyIsFlying)) == true) FlightState.FLIGHT else FlightState.LANDED
    }



    private fun enableFlightControl() {
        stickManager.setVirtualStickStateListener(virtualStickListener)
        stickManager.enableVirtualStick(enableVirtualStickCallback)
        stickManager.setVirtualStickAdvancedModeEnabled(true)
    }

    private val virtualStickListener = object : VirtualStickStateListener {
        override fun onVirtualStickStateUpdate(stickState: VirtualStickState) {
            isVirtualStickEnable = stickState.isVirtualStickEnable
            Log.d("myApp", "isVirtualStickEnable: $isVirtualStickEnable")

            isVirtualStickAdvancedModeEnabled = stickState.isVirtualStickAdvancedModeEnabled
            Log.d("myApp", "isVirtualStickAdvancedModeEnabled: $isVirtualStickAdvancedModeEnabled")
        }

        override fun onChangeReasonUpdate(reason: FlightControlAuthorityChangeReason) {
            Log.d("myApp", "FlightControlAuthorityChangeReason: $reason")
        }
    }

    companion object {
        private val logTag = FlightNode::class.java.name
        private val keyManager = KeyManager.getInstance()
        private val stickManager = VirtualStickManager.getInstance()
        private val perceptionManager = PerceptionManager.getInstance()

        enum class FlightState(val string: String) {
            LANDED("Land"), LANDING("Landing"), FLIGHT("Flight"), TAKEOFF("Takeoff"), EMERGENCY("EMERGENCY")
        }

        var flightState = FlightState.LANDED
            set(value) {
                flightStateChangeFun?.invoke(value)
                field = value
            }
        var flightStateChangeFun: ((value: FlightState) -> Unit)? = null
        var preFlightState = FlightState.LANDED
        //takeoff: this has to be binded with button

        val takeoffCmdCallback =  {  ->
            keyManager.performAction(
                    KeyTools.createKey(FlightControllerKey.KeyStartTakeoff), null, startTakeoffCallback
            )
        }
//        val getGPSCmdCallback: () -> Unit = {
//            KeyManager.getInstance().listen(
//                    KeyTools.createKey(FlightControllerKey.KeyAircraftLocation3D),
//                    this,
//                    object : CommonCallbacks.KeyListener<LocationCoordinate3D> {
//                        override fun onValueChange(oldValue: LocationCoordinate3D?, newValue: LocationCoordinate3D?) {
//                            newValue?.let {
//                                val latitude = it.latitude
//                                val longitude = it.longitude
//                                val altitude = it.altitude
//
//                                // Create a message to display in the Toast
//                                val message = "Lat: $latitude, Lon: $longitude, Alt: $altitude"
//
//                                // Display the Toast message
////                                Toast.makeText(this, message, Toast.LENGTH_LONG).show()
//                            } ?: run {
//                                // Optionally handle the case when newValue is null
//                                // Log.e(TAG, "Location data is null")
//                            }
//                        }
//                    }
//            )
//        }

        val landCmdCallback =  {  ->
            landingAction()
        }

        private fun landingAction() {
            if (flightState != FlightState.LANDING) {
                Thread {
                    if (keyManager.getValue(KeyTools.createKey(ProductKey.KeyProductType)) == ProductType.DJI_MAVIC_3_ENTERPRISE_SERIES) {
                        flightState = FlightState.LANDING
                        Log.d(logTag, "Landing Start")
                        stickManager.setVirtualStickAdvancedModeEnabled(false)
                        perceptionManager.setObstacleAvoidanceEnabled(
                                false,
                                PerceptionDirection.DOWNWARD,
                                disableDownwardObstacleAvoidanceCallback
                        )
                        val leftStick = stickManager.leftStick
                        leftStick.verticalPosition = -660

                        while (keyManager.getValue(KeyTools.createKey(FlightControllerKey.KeyIsFlying)) == true) {
                            if (flightState == FlightState.EMERGENCY) {
                                preFlightState = FlightState.FLIGHT
                                return@Thread
                            }
                            Thread.sleep(100)
                        }
                        leftStick.verticalPosition = 0
                        keyManager.performAction(
                                KeyTools.createKey(FlightControllerKey.KeyStartAutoLanding),
                                autoLandingCallback
                        )
                    } else if (keyManager.getValue(KeyTools.createKey(ProductKey.KeyProductType)) == ProductType.DJI_MINI_3) {
                        flightState = FlightState.LANDING
                        Log.d(logTag, "Landing Start")
                        keyManager.performAction(
                                KeyTools.createKey(FlightControllerKey.KeyStartAutoLanding),
                                autoLandingCallback
                        )
                    }
                }.start()
            }
        }


        private val emergencyCallback =  {  ->
            emergencyAction()
        }

        private fun emergencyAction() {
            if (flightState != FlightState.EMERGENCY) {
                preFlightState = flightState
            }
            flightState = FlightState.EMERGENCY
            when (keyManager.getValue(KeyTools.createKey(FlightControllerKey.KeyFlightMode))) {
                FlightMode.AUTO_TAKE_OFF, FlightMode.MOTOR_START -> {
                    keyManager.performAction(
                            KeyTools.createKey(FlightControllerKey.KeyStopTakeoff), stopTakeoffCallback
                    )
                }

                FlightMode.AUTO_LANDING -> keyManager.performAction(
                        KeyTools.createKey(FlightControllerKey.KeyStopAutoLanding),
                        stopAutoLandingCallback
                )

                else -> {
                    stickManager.setVirtualStickAdvancedModeEnabled(true)
                    val param = VirtualStickFlightControlParam()
                    param.rollPitchControlMode = RollPitchControlMode.VELOCITY
                    param.rollPitchCoordinateSystem = FlightCoordinateSystem.BODY
                    param.roll = 0.0
                    param.pitch = 0.0

                    // yaw control mode
                    param.yawControlMode = YawControlMode.ANGULAR_VELOCITY
                    param.yaw = 0.0

                    // vertical throttle control mode
                    param.verticalControlMode = VerticalControlMode.VELOCITY
                    param.verticalThrottle = 0.0

                    stickManager.sendVirtualStickAdvancedParam(param)
                }
            }
        }


        private val startTakeoffCallback = object : CompletionCallbackWithParam<EmptyMsg> {
            override fun onSuccess(t: EmptyMsg?) {
                Log.d(logTag, "StartTakeoff Success")
                flightState = FlightState.TAKEOFF
                Thread {
                    while (keyManager.getValue(KeyTools.createKey(FlightControllerKey.KeyFlightMode)) != FlightMode.AUTO_TAKE_OFF) {
                        if (flightState == FlightState.EMERGENCY) {
                            preFlightState =
                                    if (keyManager.getValue(KeyTools.createKey(FlightControllerKey.KeyIsFlying)) == true) FlightState.FLIGHT else FlightState.LANDED
                            return@Thread
                        }
                        Thread.sleep(50)
                    }
                    while (keyManager.getValue(KeyTools.createKey(FlightControllerKey.KeyFlightMode)) == FlightMode.AUTO_TAKE_OFF) {
                        if (flightState == FlightState.EMERGENCY) {
                            preFlightState =
                                    if (keyManager.getValue(KeyTools.createKey(FlightControllerKey.KeyIsFlying)) == true) FlightState.FLIGHT else FlightState.LANDED
                            return@Thread
                        }
                        Thread.sleep(50)
                    }
                    if (flightState == FlightState.EMERGENCY) preFlightState =
                            FlightState.FLIGHT else flightState = FlightState.FLIGHT
                    stickManager.setVirtualStickAdvancedModeEnabled(true)
                }.start()
            }

            override fun onFailure(error: IDJIError) {
                Log.e(logTag, "StartTakeoff Failure: ${error.errorCode()}")
            }

        }

        private val stopTakeoffCallback = object : CompletionCallbackWithParam<EmptyMsg> {
            override fun onSuccess(t: EmptyMsg?) {
                Log.d(logTag, "Stop Landing")
            }

            override fun onFailure(error: IDJIError) {
                Log.e(logTag, "Stop Landing Failure: ${error.hint()}")
            }
        }

        private val autoLandingCallback = object : CompletionCallbackWithParam<EmptyMsg> {
            override fun onSuccess(t: EmptyMsg?) {
                Log.d(logTag, "AutoLanding Success")
                val isLandingConfirmationNeeded =
                        keyManager.getValue(KeyTools.createKey(FlightControllerKey.KeyIsLandingConfirmationNeeded))
                Log.d(logTag, "IsLandingConfirmationNeeded: $isLandingConfirmationNeeded")
                if (isLandingConfirmationNeeded == true) {
                    keyManager.performAction(
                            KeyTools.createKey(FlightControllerKey.KeyConfirmLanding),
                            null,
                            confirmLandingCallback
                    )
                }
                flightState = FlightState.LANDED
            }

            override fun onFailure(error: IDJIError) {
                Log.e(logTag, "AutoLanding Failure: ${error.errorCode()}")
            }
        }

        private val stopAutoLandingCallback = object : CompletionCallbackWithParam<EmptyMsg> {
            override fun onSuccess(t: EmptyMsg?) {
                Log.d(logTag, "Stop Landing")
            }

            override fun onFailure(error: IDJIError) {
                Log.e(logTag, "Stop Landing Failure: ${error.errorCode()}")
            }
        }

        private val enableVirtualStickCallback = object : CommonCallbacks.CompletionCallback {
            override fun onSuccess() {
                Log.d(logTag, "EnableVirtualStick Success")
            }

            override fun onFailure(error: IDJIError) {
                Log.e(logTag, "EnableVirtualStick Failure: ${error.errorCode()}")
            }
        }

        private val disableVirtualStickCallback = object : CommonCallbacks.CompletionCallback {
            override fun onSuccess() {
                Log.d(logTag, "DisableVirtualStick Success")
            }

            override fun onFailure(error: IDJIError) {
                Log.e(logTag, "DisableVirtualStick Failure: ${error.errorCode()}")
            }
        }

        private val confirmLandingCallback = object : CompletionCallbackWithParam<EmptyMsg> {
            override fun onSuccess(t: EmptyMsg?) {
                Log.d(logTag, "ConfirmLanding Success")
            }

            override fun onFailure(error: IDJIError) {
                Log.e(logTag, "ConfirmLanding Failure: ${error.errorCode()}")
            }
        }

        private val disableDownwardObstacleAvoidanceCallback =
                object : CommonCallbacks.CompletionCallback {
                    override fun onSuccess() {
                        Log.d(logTag, "Disabled downward obstacle avoidance")
                    }

                    override fun onFailure(error: IDJIError) {
                        Log.e(
                                logTag, "Disable downward obstacle avoidance Failure: ${error.errorCode()}"
                        )
                    }
                }


        private val enableDownwardObstacleAvoidanceCallback =
                object : CommonCallbacks.CompletionCallback {
                    override fun onSuccess() {
                        Log.d(logTag, "Enable downward obstacle avoidance")
                    }

                    override fun onFailure(error: IDJIError) {
                        Log.e(
                                logTag, "Enable downward obstacle avoidance Failure: ${error.errorCode()}"
                        )
                    }
                }

        private fun enableEmergencyButton() {
            keyManager.listen(
                    KeyTools.createKey(RemoteControllerKey.KeyPauseButtonDown), this
            ) { _, newValue ->
                if (newValue == true) {
                    Log.d(logTag, "Press Emergency Button")
                    if (flightState != FlightState.EMERGENCY) {
                        preFlightState = flightState
                        flightState = FlightState.EMERGENCY
                        emergencyAction()
                    } else {
                        flightState = preFlightState
                    }
                }
            }

        }

        private fun enableHomeReturnButton() {
            keyManager.listen(
                    KeyTools.createKey(RemoteControllerKey.KeyGoHomeButtonDown), this
            ) { _, newValue ->
                if (newValue == true) {
                    Log.d(logTag, "Press GoHome Button")
                    landingAction()
                }
            }
        }

        private fun disableButtonListener() {
            keyManager.cancelListen(this)
        }

    }
}