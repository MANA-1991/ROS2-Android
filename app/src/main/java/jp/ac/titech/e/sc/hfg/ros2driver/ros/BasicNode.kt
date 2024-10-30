package jp.ac.titech.e.sc.hfg.ros2driver.ros

import android.util.Log
import dji.sdk.keyvalue.key.BatteryKey
import dji.sdk.keyvalue.key.FlightControllerKey
import dji.sdk.keyvalue.key.GimbalKey
import dji.sdk.keyvalue.key.KeyTools
import dji.sdk.keyvalue.value.common.Attitude
import dji.sdk.keyvalue.value.common.EmptyMsg
import dji.sdk.keyvalue.value.common.Velocity3D
import dji.sdk.keyvalue.value.gimbal.GimbalAngleRotation
import dji.sdk.keyvalue.value.gimbal.GimbalSpeedRotation
import dji.v5.common.callback.CommonCallbacks
import dji.v5.common.error.IDJIError
import dji.v5.manager.KeyManager
import geometry_msgs.msg.Vector3
import org.ros2.rcljava.consumers.Consumer
import org.ros2.rcljava.publisher.Publisher
import org.ros2.rcljava.subscription.Subscription
import sensor_msgs.msg.BatteryState
import std_msgs.msg.Bool
import std_msgs.msg.Int32
import java.util.concurrent.TimeUnit
import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.sin

class BasicNode(
    val name: String?,
    val rosVM: RosVM
) : DisposableNode(name) {
    // Topic
    private val gimbalCmdTopic = "/$name/gimbal_cmd"
    private val gimbalVelCmdTopic = "/$name/gimbal_vel_cmd"
    private val batteryStateTopic = "/$name/battery_state"
    private val velocityTopic = "/$name/vel"
    private val motorStateTopic = "/$name/motor_state"
    private val altitudeTopic = "/$name/AGL"
    private val onlineTopic = "/dji_online"

    // Publisher
    private val batteryStatePublisher: Publisher<BatteryState>
    private val velocityPublisher: Publisher<Vector3>
    private val motorStatePublisher: Publisher<Bool>
    private val altitudePublisher: Publisher<Int32>
    private val onlinePublisher: Publisher<std_msgs.msg.String>

    // Publish frequency
    private val batteryStatePublishFrequency = 1
    private val flightControllerPublishFrequency = 10
    private val onlinePublishFrequency = 24

    // Subscription
    private val gimbalCmdSubscription: Subscription<Vector3>
    private val gimbalVelCmdSubscription: Subscription<Vector3>

    init {
        // create publishers, subscribers and services here
        batteryStatePublisher = node.createPublisher(BatteryState::class.java, batteryStateTopic)
        velocityPublisher = node.createPublisher(Vector3::class.java, velocityTopic)
        motorStatePublisher = node.createPublisher(Bool::class.java, motorStateTopic)
        altitudePublisher = node.createPublisher(Int32::class.java, altitudeTopic)
        onlinePublisher = node.createPublisher(std_msgs.msg.String::class.java, onlineTopic)
        gimbalCmdSubscription =
            node.createSubscription(Vector3::class.java, gimbalCmdTopic, gimbalCmdCallback)
        gimbalVelCmdSubscription =
            node.createSubscription(Vector3::class.java, gimbalVelCmdTopic, gimbalVelCmdCallback)
        startRosTimer()
    }

    // Add publishers to timer
    private fun startRosTimer() {

        node.createWallTimer(
            (1000 / batteryStatePublishFrequency).toLong(),
            TimeUnit.MILLISECONDS,
            this::batteryStateTask
        )
        node.createWallTimer(
            (1000 / flightControllerPublishFrequency).toLong(),
            TimeUnit.MILLISECONDS,
            this::flightControllerTask
        )
        node.createWallTimer(
            (1000 / onlinePublishFrequency).toLong(),
            TimeUnit.MILLISECONDS,
            this::onlineTask
        )
    }


    private fun batteryStateTask() {
        val msg = BatteryState()
        val voltage = keyManager.getValue(KeyTools.createKey(BatteryKey.KeyVoltage))
        val temperature = keyManager.getValue(KeyTools.createKey(BatteryKey.KeyBatteryTemperature))
        val current = keyManager.getValue(KeyTools.createKey(BatteryKey.KeyCurrent))
        val charge = keyManager.getValue(KeyTools.createKey(BatteryKey.KeyChargeRemaining))
        val capacity = keyManager.getValue(KeyTools.createKey(BatteryKey.KeyFullChargeCapacity))
        val designCapacity = 5000 // for Mavic 3E
//        val designCapacity = 2453 // for Mini 3/ 3 Pro with Intelligent Flight Battery
//        val designCapacity = 3850 // for Mini 3/ 3 Pro with Intelligent Flight Battery Plus
        val percentage =
            keyManager.getValue(KeyTools.createKey(BatteryKey.KeyChargeRemainingInPercent))
        val powerSupplyState = 0
        val powerSupplyHealth = 0
        val powerSupplyTechnology = 2
        keyManager.getValue(KeyTools.createKey(BatteryKey.KeySmartBatterySocWarning))
        val present = keyManager.getValue(KeyTools.createKey(BatteryKey.KeyConnection))
        val cellNumber = keyManager.getValue(KeyTools.createKey(BatteryKey.KeyNumberOfCells))
        if (cellNumber != null) {
            val cellTemperature = mutableListOf<Float>()
            val cellVoltageList = mutableListOf<Float>()
            val cellVoltages = keyManager.getValue(KeyTools.createKey(BatteryKey.KeyCellVoltages))
            if (cellVoltages != null) {
                for (cellVoltage in cellVoltages) {
                    cellVoltageList.add(cellVoltage.toFloat())
                    cellTemperature.add(Float.NaN) // MSDK do not detect cell temperature
                }
            }
            msg.cellVoltage = cellVoltageList
            msg.cellTemperature = cellTemperature
        }
        val location = "onboard"
        val serialNumber = keyManager.getValue(KeyTools.createKey(BatteryKey.KeySerialNumber))

        msg.header.stamp = now()
        if (voltage != null) {
            msg.voltage = voltage.toFloat()
        }
        if (temperature != null) {
            msg.temperature = temperature.toFloat()
        }
        if (current != null) {
            msg.current = current.toFloat()
        }
        if (charge != null) {
            msg.charge = charge.toFloat()
        }
        if (capacity != null) {
            msg.capacity = capacity.toFloat()
        }
        msg.designCapacity = designCapacity.toFloat()
        if (percentage != null) {
            msg.percentage = percentage.toFloat()
        }
        msg.powerSupplyStatus = powerSupplyState.toByte()
        msg.powerSupplyHealth = powerSupplyHealth.toByte()
        msg.powerSupplyTechnology = powerSupplyTechnology.toByte()
        if (present != null) {
            msg.present = present
        }
        msg.location = location
        msg.serialNumber = serialNumber
        batteryStatePublisher.publish(msg)
    }

    private fun flightControllerTask() {
        val velocityMsg = Vector3()
        val velocityNED =
            keyManager.getValue(KeyTools.createKey(FlightControllerKey.KeyAircraftVelocity))
        val attitude =
            keyManager.getValue(KeyTools.createKey(FlightControllerKey.KeyAircraftAttitude))
        val velocityBody = transformVector(velocityNED, attitude)

        if (velocityBody != null) {
            // Convert to ROS coordinate system and to rad
            velocityMsg.x = velocityBody.x
            velocityMsg.y = -velocityBody.y
            velocityMsg.z = -velocityBody.z
        }
        rosVM.bodyVel.postValue(velocityMsg)
        velocityPublisher.publish(velocityMsg)

        val motorStateMsg = Bool()
        val areMotorsOn =
            keyManager.getValue(KeyTools.createKey(FlightControllerKey.KeyAreMotorsOn))
        if (areMotorsOn != null) {
            motorStateMsg.data = areMotorsOn
        }
//        Confirmed not working
//        val motorEscmState =
//            keyManager.getValue(KeyTools.createKey(FlightControllerKey.KeyMotorEscmState))
//        val escState = keyManager.getValue(KeyTools.createKey(FlightControllerKey.KeyEscState))
//        val escStatus = keyManager.getValue(KeyTools.createKey(FlightControllerKey.KeyEscStatus))
//        val speedratio = keyManager.getValue(KeyTools.createKey(FlightControllerKey.KeyMotorSpeedRatio))
        motorStatePublisher.publish(motorStateMsg)

        val altitudeMsg = Int32()
        val altitude =
            keyManager.getValue(KeyTools.createKey(FlightControllerKey.KeyUltrasonicHeight))
        if (altitude != null) altitudeMsg.data = altitude
        altitudePublisher.publish(altitudeMsg)
    }

    private fun transformVector(vel: Velocity3D?, attitude: Attitude?): Velocity3D? {
        if (vel == null || attitude == null) return null
        val newVelocity = Velocity3D()
        val phi = attitude.roll * Math.PI / 180
        val theta = attitude.pitch * Math.PI / 180
        val psi = attitude.yaw * Math.PI / 180
        newVelocity.x =
            cos(psi) * cos(theta) * vel.x + cos(theta) * sin(psi) * vel.y - sin(theta) * vel.z
        newVelocity.y =
            (cos(psi) * sin(theta) * sin(phi) - cos(phi) * sin(psi)) * vel.x + (cos(psi) * cos(phi) + sin(
                psi
            ) * sin(theta) * sin(phi)) * vel.y + cos(theta) * sin(phi) * vel.z
        newVelocity.z =
            (sin(psi) * sin(phi) + cos(psi) * cos(phi) * sin(theta)) * vel.x + (cos(phi) * sin(psi) * sin(
                theta
            ) - cos(psi) * sin(phi)) * vel.y + cos(theta) * cos(phi) * vel.z

        return newVelocity
    }

    private fun onlineTask() {
        val msg = std_msgs.msg.String()
        msg.data = "/" + (this.name ?: "")
        onlinePublisher.publish(msg)
    }

    companion object {
        private val tag = BasicNode::class.java.name
        private val keyManager = KeyManager.getInstance()

        /*  Callback functions of subscribers
         *
         */
        private val rotateByAngleCallback =
            object : CommonCallbacks.CompletionCallbackWithParam<EmptyMsg> {
                override fun onSuccess(t: EmptyMsg?) {
                    // Log.d(tag, "rotateByAngle Success")
                }

                override fun onFailure(error: IDJIError) {
                    Log.e(tag, "rotateByAngle Failure: ${error.errorCode()}")
                }
            }

        private val gimbalCmdCallback = Consumer<Vector3> { msg ->
//             Log.d(tag, "Receive $msg")
            val cmd = GimbalAngleRotation()
            cmd.roll = msg.x * 180 / PI
            cmd.pitch = -msg.y * 180 / PI
            cmd.yaw = -msg.z * 180 / PI
            keyManager.performAction(
                KeyTools.createKey(GimbalKey.KeyRotateByAngle),
                cmd,
                rotateByAngleCallback
            )
        }

        private val gimbalVelCmdCallback = Consumer<Vector3> { msg ->
//             Log.d(tag, "Receive $msg")
            val cmd = GimbalSpeedRotation()
            cmd.roll = msg.x * 180 / PI
            cmd.pitch = -msg.y * 180 / PI
            cmd.yaw = -msg.z * 180 / PI
            keyManager.performAction(
                KeyTools.createKey(GimbalKey.KeyRotateBySpeed),
                cmd,
                rotateByAngleCallback
            )
        }
    }
}
