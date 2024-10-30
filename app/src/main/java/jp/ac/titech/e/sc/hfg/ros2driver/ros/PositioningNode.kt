package jp.ac.titech.e.sc.hfg.ros2driver.ros

import android.util.Log
import dji.sdk.keyvalue.key.FlightControllerKey
import dji.sdk.keyvalue.key.GimbalKey
import dji.sdk.keyvalue.key.KeyTools
import dji.sdk.keyvalue.key.ProductKey
import dji.sdk.keyvalue.value.common.Attitude
import dji.sdk.keyvalue.value.product.ProductType
import dji.v5.manager.KeyManager
import geometry_msgs.msg.Pose
import geometry_msgs.msg.PoseStamped
import geometry_msgs.msg.Quaternion
import geometry_msgs.msg.Vector3
import geometry_msgs.msg.Vector3Stamped
import org.ros2.rcljava.publisher.Publisher
import java.util.concurrent.TimeUnit
import kotlin.math.abs
import kotlin.math.atan
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

abstract class PositioningNode(name: String?, val rosVM: RosVM) : DisposableNode(name) {

    private val positionPublisher: Publisher<PoseStamped>
    private val gimbalStatePublisher: Publisher<Vector3Stamped>

    var position: Pose = Pose()
        private set(value) {
            rosVM.position.postValue(value)
            field = value
        }

    // topic
    private val positionTopic = "/$name/pos"
    private val gimbalStateTopic = "/$name/gimbal_state"
    private val positionPublishFrequency = 20
    private val gimbalStatePublishFrequency = 20


    init {
        positionPublisher = node.createPublisher(PoseStamped::class.java, positionTopic)
        gimbalStatePublisher = node.createPublisher(Vector3Stamped::class.java, gimbalStateTopic)
        node.createWallTimer(
            (1000 / positionPublishFrequency).toLong(),
            TimeUnit.MILLISECONDS
        ) {
            positionTask()
        }
        node.createWallTimer(
            (1000 / gimbalStatePublishFrequency).toLong(),
            TimeUnit.MILLISECONDS
        ) {
            gimbalStateTask()
        }

    }

    abstract fun getPosition(): PoseStamped

    private fun positionTask() {
        val msg = getPosition()
        msg.header.stamp = now()
        msg.header.frameId = "world"
        position = msg.pose
        positionPublisher.publish(msg)
    }

    private fun gimbalStateTask() {
        val gimbalAttitude =
            keyManager.getValue(KeyTools.createKey(GimbalKey.KeyGimbalAttitude))
        val aircraftAttitude =
            keyManager.getValue(KeyTools.createKey(FlightControllerKey.KeyAircraftAttitude))
        val yawRelativeToAircraft =
            keyManager.getValue(KeyTools.createKey(GimbalKey.KeyYawRelativeToAircraftHeading))

        if (gimbalAttitude != null && aircraftAttitude != null && yawRelativeToAircraft != null) {
//            Log.d(tag,"gimbalAttitude:roll:${gimbalAttitude.roll},pitch:${gimbalAttitude.pitch},yaw:${gimbalAttitude.yaw}")
//            Log.d(tag,"aircraftAttitude:roll:${aircraftAttitude.roll},pitch:${aircraftAttitude.pitch},yaw:${aircraftAttitude.yaw}")
//            Log.d(tag,"YawRelativeToAircraft:${keyManager.getValue(KeyTools.createKey(GimbalKey.KeyYawRelativeToAircraftHeading))}")
            gimbalAttitude.yaw =
                aircraftAttitude.yaw + yawRelativeToAircraft //TODO: gimbalAttitudeのヨーが離陸後に何故かずれる問題の対処　msdk v5.9~??
            val aircraftToGimbalQuaternion = mulQuaternion(
                attitudeToQuaternion(gimbalAttitude),
                invQuaternion(attitudeToQuaternion(aircraftAttitude))
            )
            val msg = Vector3Stamped().apply {
                vector = calcGimbalLink(aircraftToGimbalQuaternion)
                header.stamp = now()
            }
            gimbalStatePublisher.publish(msg)
        }
    }


    companion object {
        private val keyManager = KeyManager.getInstance()
        private val productType: ProductType? =
            keyManager.getValue(KeyTools.createKey(ProductKey.KeyProductType))
        private const val tag = "Positioning"
        private fun Double.equalsDelta(other: Double) = abs(this - other) < 1e-9

        private fun attitudeToQuaternion(attitude: Attitude): Quaternion {
            val roll = attitude.roll * Math.PI / 180
            val pitch = -attitude.pitch * Math.PI / 180
            val yaw = -attitude.yaw * Math.PI / 180
            val sr = sin(roll / 2)
            val cr = cos(roll / 2)
            val sp = sin(pitch / 2)
            val cp = cos(pitch / 2)
            val sy = sin(yaw / 2)
            val cy = cos(yaw / 2)
            return Quaternion().apply {
                w = -sr * sp * sy + cr * cp * cy
                x = cr * sp * sy + sr * cp * cy
                y = -sr * cp * sy + cr * sp * cy
                z = cr * cp * sy + sr * sp * cy
            }
        }

        private fun calcGimbalLink(q: Quaternion): Vector3 =
            when (productType) {
                ProductType.DJI_MAVIC_3_ENTERPRISE_SERIES, ProductType.DJI_MAVIC_3 -> calcGimbalLinkMavic3E(
                    q
                )

                ProductType.DJI_MINI_3 -> calcGimbalLinkMini3(q)
                ProductType.DJI_MINI_3_PRO -> calcGimbalLinkMini3Pro(q)
                else -> calcGimbalLinkMavic3E(q)
            }

        private fun calcGimbalLinkMavic3E(q: Quaternion): Vector3 {
            val n1 = -0.353505
            val n3 = 0.935433
            q.run {
                val tanPitch =
                    (n3 * (w * w - x * x - y * y + z * z) + 2 * n1 * (w * y + x * z) - sqrt(-4 * (n1 * (w * z - x * y) - n3 * (y * z + w * x)) * (n1 * (w * z - x * y) - n3 * (y * z + w * x)) + n3 * n3)) / (2 * (n1 * (w * w + x * x) - n3 * (w * y - x * z)))
                val pitch = 2 * atan(tanPitch)
                val tanRoll =
                    if ((w * z).equalsDelta(x * y)) (x - z) / (w - y) else (w * tanPitch - y) / (x * tanPitch - z)
                val roll = 2 * atan(tanRoll)
                val tanYaw =
                    if ((w * z).equalsDelta(x * y)) 0.0 else -2 * (w * tanPitch - y) / (n3 * sin(
                        roll
                    ) * (w + x * tanRoll + tanPitch * (y + z * tanRoll)))
                val yaw = 2 * atan(tanYaw)
                return Vector3().apply {
                    x = roll
                    y = pitch
                    z = yaw
                }
            }
        }

        private fun calcGimbalLinkMini3(q: Quaternion): Vector3 {
            val n1 = 0.362817
            val n3 = 0.931860
            val d = n1 / n3
            q.run {
                val tanPitch =
                    ((w * w + x * x - y * y - z * z) + 2 * d * (w * y - x * z) - sqrt(1 - 4 * (d * d + 1) * (w * z + x * y) * (w * z + x * y))) / (2 * (d * (w * w + x * x) - (w * y - x * z)))
                val pitch = 2 * atan(tanPitch)
                val tanRoll =
                    if ((w * z).equalsDelta(-(x * y))) (x + z) / (w - y) else (y - w * tanPitch) / (x * tanPitch + z)
                val roll = 2 * atan(tanRoll)
                val tanYaw =
                    if ((w * z).equalsDelta(-(x * y))) 0.0 else -2 * (w * tanPitch - y) / (n3 * sin(
                        roll
                    ) * (w + x * tanRoll + tanPitch * (y - z * tanRoll)))
                val yaw = 2 * atan(tanYaw)
                return Vector3().apply {
                    x = roll
                    y = pitch
                    z = yaw
                }
            }
        }

        private fun calcGimbalLinkMini3Pro(q: Quaternion): Vector3 {
            val n1 = 0.360035
            val n3 = 0.932939
            val d = n1 / n3
            q.run {
                val tanPitch =
                    ((w * w + x * x - y * y - z * z) + 2 * d * (w * y - x * z) - sqrt(1 - 4 * (d * d + 1) * (w * z + x * y) * (w * z + x * y))) / (2 * (d * (w * w + x * x) - (w * y - x * z)))
                val pitch = 2 * atan(tanPitch)
                val tanRoll =
                    if ((w * z).equalsDelta(-(x * y))) (x + z) / (w - y) else (y - w * tanPitch) / (x * tanPitch + z)
                val roll = 2 * atan(tanRoll)
                val tanYaw =
                    if ((w * z).equalsDelta(-(x * y))) 0.0 else -2 * (w * tanPitch - y) / (n3 * sin(roll) * (w + x * tanRoll + tanPitch * (y - z * tanRoll)))
                val yaw = 2 * atan(tanYaw)
                return Vector3().apply {
                    x = roll
                    y = pitch
                    z = yaw
                }
            }
        }

        private fun mulQuaternion(a: Quaternion, b: Quaternion): Quaternion {
            return Quaternion().apply {
                w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z
                x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y
                y = a.w * b.y + a.y * b.w + a.z * b.x - a.x * b.z
                z = a.w * b.z + a.z * b.w + a.x * b.y - a.y * b.x
            }
        }

        private fun invQuaternion(q: Quaternion): Quaternion {
            return Quaternion().apply {
                w = q.w
                x = -q.x
                y = -q.y
                z = -q.z
            }
        }
    }
}
