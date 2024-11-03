package jp.ac.titech.e.sc.hfg.ros2driver.ros

import android.content.SharedPreferences
import android.util.Log
import org.ros2.rcljava.executors.Executor

class NodeManager(
    private val executor: Executor,
    private val imageExecutor: Executor,
    private val rosVM: RosVM,
    private val preferences: SharedPreferences
) {
    private val tag = "NodeManager"

    var name: String = preferences.getString("name_space", "mavic_1") ?: "mavic_1"
    var positioningType: PositioningType =
        preferences.getString("positioning_type", PositioningType.RTK.name)
            ?.let { PositioningType.valueOf(it) } ?: PositioningType.RTK
        private set(value) {
            // save preference
            with(preferences.edit()) {
                putString("positioning_type", value.name)
                apply()
            }
            field = value
        }

    // Node
    var basicNode: BasicNode? = null
    var flightNode: FlightNode? = null
    var imageNode: ImageNode? = null
//    var natNetNode: NatNetNode? = null
    var rtkNode: RTKNode? = null


    init {
        FlightNode.flightStateChangeFun = { value ->
            rosVM.flightState.postValue(value.string)
        }
    }


    fun startAllNode() {
        stopAllNode()
        if (rosVM.aircraftConnected.value != true) {
            Log.e(tag, "Aircraft not connected")
            return
        }
        basicNode = BasicNode(name, rosVM).also { executor.addNode(it) }
        flightNode = FlightNode(name, rosVM).also { executor.addNode(it) }
        imageNode = ImageNode(name).also { imageExecutor.addNode(it) }
//        when (positioningType) {
//            PositioningType.NatNet -> natNetNode =
//                NatNetNode(name, rosVM).also { executor.addNode(it) }
//
//            PositioningType.RTK -> rtkNode =
//                RTKNode(name, rosVM, preferences).also { executor.addNode(it) }
//        }
        rosVM.isRosRun.postValue(true)
    }

    fun stopAllNode() {
        listOf(basicNode, flightNode, rtkNode).forEach {
            executor.removeNode(it)
            it?.dispose()
        }
        imageNode.let {
            imageExecutor.removeNode(it)
            it?.dispose()
        }
        basicNode = null;flightNode = null;imageNode = null;rtkNode = null;
        rosVM.isRosRun.postValue(false)
    }

    fun changePositioningType(_positioningType: PositioningType) {
        positioningType = _positioningType
        if (rosVM.isRosRun.value == true) startAllNode()
    }

    fun changeNameSpace(nodeName: String) {
        name = nodeName
        with(preferences.edit()) {
            putString("name_space", name)
            apply()
        }
        if (rosVM.isRosRun.value == true) startAllNode()
    }

    companion object {
        enum class PositioningType {
            RTK
        }
    }
}