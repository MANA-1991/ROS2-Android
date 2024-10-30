package jp.ac.titech.e.sc.hfg.ros2driver.ros

import android.util.Log
import dji.sdk.keyvalue.key.FlightControllerKey
import dji.sdk.keyvalue.key.KeyTools
import dji.sdk.keyvalue.key.ProductKey
import dji.sdk.keyvalue.key.RemoteControllerKey
import dji.sdk.keyvalue.value.common.EmptyMsg
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
import geometry_msgs.msg.Twist
import org.ros2.rcljava.consumers.Consumer
import org.ros2.rcljava.subscription.Subscription
import std_msgs.msg.Empty

class FlightNode(name: String?, private val rosVM: RosVM) : DisposableNode(name) {
    // Subscription
    private val takeoffCmdSubscription: Subscription<Empty>
    private val landCmdSubscription: Subscription<Empty>
    private val emergencyCmdSubscription: Subscription<Empty>
    private val pilotingCmdSubscription: Subscription<Twist>

    // Topic names
    private val takeoffCmdTopic = "/$name/takeoff"
    private val emergencyCmdTopic = "/$name/emergency"
    private val landCmdTopic = "/$name/land"
    private val pilotingCmdTopic = "/$name/cmd_vel"

    // State
    private var isVirtualStickEnable = false
    private var isVirtualStickAdvancedModeEnabled = false
    private val pilotingCmdCallback = Consumer<Twist> { msg ->
        rosVM.cmdVel.postValue(msg)
        if (flightState == FlightState.FLIGHT) {
            val roll = msg.linear.x
            val pitch = -msg.linear.y
            val verticalThrottle = msg.linear.z
            val yaw = -msg.angular.z
            val param = VirtualStickFlightControlParam()
            // roll pitch control mode
            param.rollPitchControlMode = RollPitchControlMode.VELOCITY
            param.rollPitchCoordinateSystem = FlightCoordinateSystem.BODY
            param.roll = roll
            param.pitch = pitch

            // yaw control mode
            param.yawControlMode = YawControlMode.ANGULAR_VELOCITY
            param.yaw = yaw * 180 / Math.PI

            // vertical throttle control mode
            param.verticalControlMode = VerticalControlMode.VELOCITY
            param.verticalThrottle = verticalThrottle

            Log.d(logTag, param.toString())
            stickManager.sendVirtualStickAdvancedParam(param)
        }
    }

    init {
        takeoffCmdSubscription =
            node.createSubscription(Empty::class.java, takeoffCmdTopic, takeoffCmdCallback)
        landCmdSubscription =
            node.createSubscription(Empty::class.java, landCmdTopic, landCmdCallback)
        emergencyCmdSubscription =
            node.createSubscription(Empty::class.java, emergencyCmdTopic, emergencyCallback)
        pilotingCmdSubscription =
            node.createSubscription(Twist::class.java, pilotingCmdTopic, pilotingCmdCallback)
        enableFlightControl()
        enableEmergencyButton()
        enableHomeReturnButton()
        flightState =
            if (keyManager.getValue(KeyTools.createKey(FlightControllerKey.KeyIsFlying)) == true) FlightState.FLIGHT else FlightState.LANDED
    }

    override fun dispose() {
        disableFlightControl()
        disableButtonListener()
        super.dispose()
    }

    private fun enableFlightControl() {
        stickManager.setVirtualStickStateListener(virtualStickListener)
        stickManager.enableVirtualStick(enableVirtualStickCallback)
        stickManager.setVirtualStickAdvancedModeEnabled(true)
    }

    private fun disableFlightControl() {
        stickManager.disableVirtualStick(disableVirtualStickCallback)
        stickManager.removeVirtualStickStateListener(virtualStickListener)
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

        private val takeoffCmdCallback = Consumer<Empty> { msg ->
            keyManager.performAction(
                KeyTools.createKey(FlightControllerKey.KeyStartTakeoff), null, startTakeoffCallback
            )
        }

        private val landCmdCallback = Consumer<Empty> { msg ->
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


        private val emergencyCallback = Consumer<Empty> { msg ->
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