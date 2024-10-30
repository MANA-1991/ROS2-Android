package jp.ac.titech.e.sc.hfg.ros2driver.ros

import android.content.SharedPreferences
import android.location.Location
import android.os.Handler
import android.os.Looper
import android.widget.Toast
import dji.sdk.keyvalue.key.FlightControllerKey
import dji.sdk.keyvalue.key.KeyTools
import dji.sdk.keyvalue.value.common.Attitude
import dji.v5.manager.KeyManager
import dji.v5.manager.aircraft.rtk.RTKCenter
import dji.v5.manager.aircraft.rtk.RTKLocationInfoListener
import geometry_msgs.msg.PoseStamped
import org.ros2.rcljava.Time
import org.ros2.rcljava.consumers.Consumer
import org.ros2.rcljava.publisher.Publisher
import org.ros2.rcljava.subscription.Subscription
import org.ros2.rcljava.timer.WallTimer
import sensor_msgs.msg.NavSatFix
import sensor_msgs.msg.NavSatStatus
import std_msgs.msg.Float64MultiArray
import java.util.concurrent.TimeUnit
import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.sin

class RTKNode(name: String?, rosVM: RosVM, private val preferences: SharedPreferences) :
    PositioningNode(name, rosVM) {
    private val tag = "RTK"
    private val keyManager = KeyManager.getInstance()
    private val rtkCenter = RTKCenter.getInstance()

    // Topic
    private val originLocationTopic = "/setOriginLocation"
    private val gpsTopic = "/$name/GPS"

    // Subscription
    private val originLocationSubscriber: Subscription<Float64MultiArray>

    // Publisher
    private val originLocationPublisher: Publisher<Float64MultiArray> =
        node.createPublisher(Float64MultiArray::class.java, originLocationTopic)
    private val gpsPublisher: Publisher<NavSatFix> =
        node.createPublisher(NavSatFix::class.java, gpsTopic)

    // Publish frequency
    private val gpsPublishFrequency = 20.0


    var originLocation: Location = Location(null).apply {
        latitude = Double.fromBits(preferences.getLong("origin_location_latitude", 0.0.toRawBits()))
        longitude =
            Double.fromBits(preferences.getLong("origin_location_longitude", 0.0.toRawBits()))
        altitude = Double.fromBits(preferences.getLong("origin_location_altitude", 0.0.toRawBits()))
        bearing = preferences.getFloat("origin_location_bearing", 0.0f)
    }
        private set(value) {
            // save preferences
            value.let {
                with(preferences.edit()) {
                    putLong("origin_location_latitude", it.latitude.toRawBits())
                    putLong("origin_location_longitude", it.longitude.toRawBits())
                    putLong("origin_location_altitude", it.altitude.toRawBits())
                    putFloat("origin_location_bearing", it.bearing)
                    apply()
                }
            }
            field = value
        }

    var currentLocation: Location = Location(null)
        set(value) {
            field = value
            rosVM.currentLocationLat.postValue(String.format("%18.15f", field.latitude))
            rosVM.currentLocationLon.postValue(String.format("%18.15f", field.longitude))
            rosVM.currentLocationAlt.postValue(String.format("%18.15f", field.altitude))
            rosVM.currentLocationHeading.postValue(field.bearing.toString())
        }


    private val gpsPublisherTimer: WallTimer
    private val gpsData: NavSatFix = NavSatFix()

    private val setOriginLocationCallback = Consumer<Float64MultiArray> { msg ->
        if (msg.data.size != 4) return@Consumer
        originLocation = Location(null).apply {
            latitude = msg.data[0]
            longitude = msg.data[1]
            altitude = msg.data[2]
            bearing = msg.data[3].toFloat()
        }
        rosVM.editTextLat.postValue(originLocation.latitude.toString())
        rosVM.editTextLon.postValue(originLocation.longitude.toString())
        rosVM.editTextAlt.postValue(originLocation.altitude.toString())
        rosVM.editTextHeading.postValue(originLocation.bearing.toString())
    }

    private val rtkLocationInfoListenerFun: RTKLocationInfoListener =
        RTKLocationInfoListener { locationInfo ->
            run {
                locationInfo?.let {
                    currentLocation = Location(null).apply {
                        latitude = it.real3DLocation.latitude ?: 0.0
                        longitude = it.real3DLocation.longitude ?: 0.0
                        altitude = it.real3DLocation.altitude ?: 0.0
                        bearing = it.realHeading?.toFloat() ?: 0.0f
                    }
                    gpsData.apply {
                        header.stamp = now()
                        latitude = it.real3DLocation.latitude
                        longitude = it.real3DLocation.longitude
                        altitude = it.real3DLocation.altitude
                        positionCovariance = listOf(
                            it.rtkLocation?.stdLatitude ?: 0.0, 0.0, 0.0,
                            0.0, it.rtkLocation?.stdLongitude ?: 0.0, 0.0,
                            0.0, 0.0, it.rtkLocation?.stdAltitude ?: 0.0
                        )
                        status = NavSatStatus().apply {
                            // TODO
                        }

                    }
                }
            }
        }


    init {
        originLocationSubscriber = node.createSubscription(
            Float64MultiArray::class.java,
            originLocationTopic,
            setOriginLocationCallback
        )

        gpsPublisherTimer =
            node.createWallTimer(
                (1000 / gpsPublishFrequency).toLong(),
                TimeUnit.MILLISECONDS,
                this::gpsPositionTask
            )

        rtkCenter.addRTKLocationInfoListener(rtkLocationInfoListenerFun)
    }

    override fun dispose() {
        super.dispose()
        rtkCenter.removeRTKLocationInfoListener(rtkLocationInfoListenerFun)
    }

    private fun gpsPositionTask() {
        gpsPublisher.publish(gpsData)
    }

    fun setNewOriginLocation(location: Location) {
        originLocation = location
        publishOriginLocation()
    }

    fun publishOriginLocation() {
        originLocationPublisher.publish(Float64MultiArray().apply {
            data =
                listOf(
                    originLocation.latitude,
                    originLocation.longitude,
                    originLocation.altitude,
                    originLocation.bearing.toDouble()
                )
        })

        Handler(Looper.getMainLooper()).post {
            Toast.makeText(
                rosVM.getApplication(),
                "published origin location",
                Toast.LENGTH_SHORT
            ).show()
        }
    }

    override fun getPosition(): PoseStamped {
        val msg = PoseStamped()
        // East  -> x
        // North -> y
        val bearing: Float = originLocation.bearingTo(currentLocation) - originLocation.bearing
        val distance: Float = originLocation.distanceTo(currentLocation)
        msg.pose.position.x = distance * sin(bearing * PI / 180.0)
        msg.pose.position.y = distance * cos(bearing * PI / 180.0)
        msg.pose.position.z = currentLocation.altitude - originLocation.altitude

        val attitude: Attitude? =
            keyManager.getValue(KeyTools.createKey(FlightControllerKey.KeyAircraftAttitude))
        attitude?.let {
            // Convert to ROS coordinate system and to rad
            val roll = it.roll * PI / 180 / 2
            val pitch = -it.pitch * PI / 180 / 2
            val yaw = -(it.yaw - originLocation.bearing - 90) * PI / 180 / 2 // East is X

            // convert quaternion
            val cphi = cos(roll)
            val sphi = sin(roll)
            val ctheta = cos(pitch)
            val stheta = sin(pitch)
            val cpsi = cos(yaw)
            val spsi = sin(yaw)
            msg.pose.orientation.w = cphi * ctheta * cpsi + sphi * stheta * spsi
            msg.pose.orientation.x = sphi * ctheta * cpsi - cphi * stheta * spsi
            msg.pose.orientation.y = cphi * stheta * cpsi + sphi * ctheta * spsi
            msg.pose.orientation.z = cphi * ctheta * spsi - sphi * stheta * cpsi
        }

        return msg
    }

}
