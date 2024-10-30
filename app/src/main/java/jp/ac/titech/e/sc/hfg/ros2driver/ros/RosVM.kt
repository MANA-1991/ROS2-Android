package jp.ac.titech.e.sc.hfg.ros2driver.ros

import android.app.Application
import android.content.Context
import android.location.Location
import androidx.lifecycle.AndroidViewModel
import androidx.lifecycle.DefaultLifecycleObserver
import androidx.lifecycle.LifecycleOwner
import androidx.lifecycle.MutableLiveData
import androidx.lifecycle.asFlow
import androidx.lifecycle.viewModelScope
import geometry_msgs.msg.Pose
import geometry_msgs.msg.Twist
import geometry_msgs.msg.Vector3
import jp.ac.titech.e.sc.hfg.ros2driver.R
import kotlinx.coroutines.flow.launchIn
import kotlinx.coroutines.flow.onEach
import org.ros2.rcljava.RCLJava
import org.ros2.rcljava.executors.Executor
import org.ros2.rcljava.executors.MultiThreadedExecutor


class RosVM(application: Application) : AndroidViewModel(application), DefaultLifecycleObserver {
    val aircraftConnected: MutableLiveData<Boolean> by lazy { MutableLiveData<Boolean>() }
    val isAvailableRTK: MutableLiveData<Boolean> by lazy { MutableLiveData<Boolean>() }

    val namespace: MutableLiveData<String> by lazy { MutableLiveData<String>() }
    val isRosRun: MutableLiveData<Boolean> by lazy { MutableLiveData<Boolean>() }
    val flightState: MutableLiveData<String> by lazy { MutableLiveData<String>() }
    val position: MutableLiveData<Pose> by lazy { MutableLiveData<Pose>() }
    val cmdVel: MutableLiveData<Twist> by lazy { MutableLiveData<Twist>() }
    val bodyVel: MutableLiveData<Vector3> by lazy { MutableLiveData<Vector3>() }

    private val sharedPreferences =
        application.applicationContext.getSharedPreferences("RosSetting", Context.MODE_PRIVATE)

    val nodeManager: NodeManager by lazy {
        NodeManager(
            executor,
            imageExecutor,
            this,
            sharedPreferences
        )
    }
    private val executor: Executor = MultiThreadedExecutor()
    private val imageExecutor: Executor = MultiThreadedExecutor()
    private lateinit var rosThread: Thread
    private lateinit var rosImageThread: Thread


    // RTK Fragment
    val editTextLat: MutableLiveData<String> by lazy { MutableLiveData() }
    val editTextLon: MutableLiveData<String> by lazy { MutableLiveData() }
    val editTextAlt: MutableLiveData<String> by lazy { MutableLiveData() }
    val editTextHeading: MutableLiveData<String> by lazy { MutableLiveData() }
    val currentLocationLat: MutableLiveData<String> by lazy { MutableLiveData() }
    val currentLocationLon: MutableLiveData<String> by lazy { MutableLiveData() }
    val currentLocationAlt: MutableLiveData<String> by lazy { MutableLiveData() }
    val currentLocationHeading: MutableLiveData<String> by lazy { MutableLiveData() }

    val isEnabledSetOriginLocationButton: MutableLiveData<Boolean> by lazy { MutableLiveData() }
    val isEnabledResetOriginLocationButton: MutableLiveData<Boolean> by lazy { MutableLiveData() }
    val isEnabledPublishOriginLocationButton: MutableLiveData<Boolean> by lazy { MutableLiveData() }

    // ROS Setting Fragment
    val textCmdVelX: MutableLiveData<String> by lazy { MutableLiveData() }
    val textCmdVelY: MutableLiveData<String> by lazy { MutableLiveData() }
    val textCmdVelZ: MutableLiveData<String> by lazy { MutableLiveData() }
    val textCmdVelYaw: MutableLiveData<String> by lazy { MutableLiveData() }

    private fun validateEditOrigin(): Boolean {
        val lat = editTextLat.value?.toDoubleOrNull()
        val lon = editTextLon.value?.toDoubleOrNull()
        val alt = editTextAlt.value?.toDoubleOrNull()
        val heading = editTextHeading.value?.toFloatOrNull()
        if (lat == null || lat > 90.0 || lat < -90.0 || lon == null || lon > 180.0 || lon < -180 || alt == null || heading == null || heading >= 360 || heading < 0) return false
        return true
    }

    private fun isSyncOriginLocation(): Boolean {
        if (!validateEditOrigin()) return false
        val lat = editTextLat.value!!.toDoubleOrNull()
        val lon = editTextLon.value!!.toDoubleOrNull()
        val alt = editTextAlt.value!!.toDoubleOrNull()
        val heading = editTextHeading.value!!.toFloatOrNull()
        if (nodeManager.rtkNode?.originLocation?.latitude == lat && nodeManager.rtkNode?.originLocation?.longitude == lon && nodeManager.rtkNode?.originLocation?.altitude == alt && nodeManager.rtkNode?.originLocation?.bearing == heading) return true
        return false
    }

    fun onClickSetOrigin() {
        if (!validateEditOrigin()) return
        val originLocation = Location(null).apply {
            latitude = editTextLat.value!!.toDouble()
            longitude = editTextLon.value!!.toDouble()
            altitude = editTextAlt.value!!.toDouble()
            bearing = editTextHeading.value!!.toFloat()
        }
        nodeManager.rtkNode?.setNewOriginLocation(originLocation)
        updateRtkFragmentState()
    }

    fun onClickResetOrigin() {
        editTextLat.value = nodeManager.rtkNode?.originLocation?.latitude.toString()
        editTextLon.value = nodeManager.rtkNode?.originLocation?.longitude.toString()
        editTextAlt.value = nodeManager.rtkNode?.originLocation?.altitude.toString()
        editTextHeading.value = nodeManager.rtkNode?.originLocation?.bearing.toString()
        updateRtkFragmentState()
    }

    fun onClickPublishOrigin() {
        nodeManager.rtkNode?.publishOriginLocation()
        updateRtkFragmentState()
    }

    fun onClickCurrentPositionToOrigin() {
        if (nodeManager.rtkNode?.currentLocation == null) return
        editTextLat.value = currentLocationLat.value
        editTextLon.value = currentLocationLon.value
        editTextAlt.value = currentLocationAlt.value
        editTextHeading.value = currentLocationHeading.value
        updateRtkFragmentState()
    }


    private fun initRtkFragment() {
        listOf(editTextLat, editTextLon, editTextAlt, editTextHeading).forEach { mutableLiveData ->
            mutableLiveData.asFlow().onEach { updateRtkFragmentState() }.launchIn(viewModelScope)
        }
    }

    private fun initROSSettingFragment(owner: LifecycleOwner) {
        cmdVel.observe(owner) {
            textCmdVelX.postValue(String.format("%.2f", it.linear.x))
            textCmdVelY.postValue(String.format("%.2f", it.linear.y))
            textCmdVelZ.postValue(String.format("%.2f", it.linear.z))
            textCmdVelYaw.postValue(String.format("%.2f", it.angular.z))
        }
    }

    fun updateRtkFragmentState() {
        isEnabledSetOriginLocationButton.value = !isSyncOriginLocation() && validateEditOrigin()
        isEnabledResetOriginLocationButton.value = !isSyncOriginLocation()
        isEnabledPublishOriginLocationButton.value = isSyncOriginLocation()
    }

    // ROS setting fragment
    val positioningTypeLiveData: MutableLiveData<Int> by lazy {
        MutableLiveData(
            when (nodeManager.positioningType) {
//                NodeManager.Companion.PositioningType.NatNet -> R.id.radioNatNet
                NodeManager.Companion.PositioningType.RTK -> R.id.radioRTK
            }
        )
    }

    fun onClickStartRosButton() {
        if (isRosRun.value != true) makeRosNode()
        else disposeRosNode()
    }

    fun onClickChangeNameSpace() {
        namespace.value?.let { nodeManager.changeNameSpace(it) }
    }

    fun onClickPositioningType(id: Int) {
        val newPositioningType = when (id) {
//            R.id.radioNatNet -> NodeManager.Companion.PositioningType.NatNet
            R.id.radioRTK -> NodeManager.Companion.PositioningType.RTK
            else -> NodeManager.Companion.PositioningType.RTK
        }
        nodeManager.changePositioningType(newPositioningType)
    }

    override fun onCreate(owner: LifecycleOwner) {
        super.onCreate(owner)
        isRosRun.value = false
        aircraftConnected.value = false
        isAvailableRTK.value = false
        namespace.value = nodeManager.name

        initRtkFragment()
        initROSSettingFragment(owner)

        RCLJava.rclJavaInit()
        rosThread = Thread {
            try {
                while (RCLJava.ok()) {
                    executor.spinSome()
                    Thread.sleep(SPINNER_PERIOD_MS)
                }
            } catch (ex: InterruptedException) {
                Thread.currentThread().interrupt()
            }
        }
        rosImageThread = Thread {
            try {
                while (RCLJava.ok()) {
                    imageExecutor.spinSome()
                    Thread.sleep(SPINNER_PERIOD_MS)
                }
            } catch (ex: InterruptedException) {
                Thread.currentThread().interrupt()
            }
        }
        rosThread.start()
        rosImageThread.start()
    }

    override fun onDestroy(owner: LifecycleOwner) {
        super.onDestroy(owner)
        rosThread.interrupt()
        rosImageThread.interrupt()
    }

    fun makeRosNode() {
        nodeManager.startAllNode()
        onClickResetOrigin()
    }

    fun disposeRosNode() {
        nodeManager.stopAllNode()
    }

    companion object {
        private const val SPINNER_PERIOD_MS: Long = 10
    }


}