//package jp.ac.titech.e.sc.hfg.ros2driver.ros
//
//import android.util.Log
//import geometry_msgs.msg.PoseStamped
////import jp.ac.titech.e.sc.hfg.mobilenatnet.DataDescriptions
////import jp.ac.titech.e.sc.hfg.mobilenatnet.NatNetClient
//
//
//@OptIn(ExperimentalUnsignedTypes::class)
//class NatNetNode(val name: String?, rosVM: RosVM) : PositioningNode(name, rosVM) {
//    private val rigidBodyHandler = RigidBodyHandler()
//    private val tag = "NatNet"
//    private val streamClient: NatNetClient = NatNetClient().apply {
//        localIpAddress = "127.0.0.1"
//        serverIpAddress = "192.168.208.20"
//        useMulticast = true
//        multicastAddress = "239.255.42.99"
//        rigidBodyListener =
//            { id: Int, pos: ArrayList<Double>, rot: ArrayList<Double> ->
//                rigidBodyHandler.updateRigidBody(id, pos, rot)
//            }
//        dataDescriptionsListener = { dataDescs: DataDescriptions ->
//            Log.d(tag, "Received data desc")
//            for (i in dataDescs.rigidBodyList) {
//                Log.d(
//                    tag,
//                    "Unpacked data desc, name: ${i.szName}, id: ${i.idNum}, pos: ${i.pos}"
//                )
//            }
//            dataDescs.rigidBodyList.forEach {
//                rigidBodyHandler.setNameById(it.idNum, it.szName)
//            }
//        }
//    }
//    private val natNetThread: Thread = Thread {
//        streamClient.run()
//        while(true){
//            try {
//                Thread.sleep(1000)
//            }
//            catch (e: InterruptedException){
//                streamClient.shutdown()
//            }
//        }
//    }
//
//    init {
//        natNetThread.start()
//    }
//
//    override fun dispose() {
//        natNetThread.interrupt()
//        super.dispose()
//    }
//
//    override fun getPosition(): PoseStamped {
//        val msg = PoseStamped()
//        name?.let {
//            val pos = rigidBodyHandler.getPositionByName(it)
//            if (pos != null) {
//                if (pos.size >= 7) {
//                    msg.pose.position.x = pos[0]
//                    msg.pose.position.y = pos[1]
//                    msg.pose.position.z = pos[2]
//                    msg.pose.orientation.x = pos[3]
//                    msg.pose.orientation.y = pos[4]
//                    msg.pose.orientation.z = pos[5]
//                    msg.pose.orientation.w = pos[6]
//                }
//            }
//        }
//        return msg
//    }
//
//    class RigidBodyHandler {
//        private var rigidBodyMap = mutableMapOf<Int, Data>()
//        private var nameMap = mutableMapOf<String,Int>()
//        private val tag = "NatNet"
//
//        fun updateRigidBody(newId: Int, pos: ArrayList<Double>, rot: ArrayList<Double>) {
//            if (newId in rigidBodyMap.keys) {
//                rigidBodyMap[newId]?.pos = pos
//                rigidBodyMap[newId]?.rot = rot
//                Log.d(tag, "Update map")
//            } else if (newId !in rigidBodyMap.keys) {
//                val rigidBodyData = Data(newId, getNameById(newId), pos, rot)
//                rigidBodyMap[newId] = rigidBodyData
//                Log.d(tag, "Create map, newId: $newId, dict: ${rigidBodyMap.keys}")
//            }
//            showAsText()
//        }
//
//        fun setNameById(id: Int, name: String) {
//            Log.d(tag, "bind name: $name to id $id")
//            rigidBodyMap[id]?.name = name
//            nameMap[name]=id
//        }
//
//        fun getNameById(id: Int): String {
//            return rigidBodyMap[id]?.name ?: "Unknown"
//        }
//
//        fun getFirstIdByName(name: String): Int {
//            return nameMap.getOrDefault(name,-1)
//        }
//
//
//        fun getPositionByName(name: String): ArrayList<Double>? {
//            val id = getFirstIdByName(name)
//            val pos = rigidBodyMap[id]?.pos
//            val rot = rigidBodyMap[id]?.rot
//            return if (pos != null && rot != null) {
//                ArrayList(pos + rot)
//            } else {
//                null
//            }
//        }
//
//        private fun showAsText() {
//            var outStr = ""
//            for (i in rigidBodyMap.keys) {
//                val data = rigidBodyMap[i]
//                outStr += data?.let {
//                    "%s, id: %2d, position: %2.2f, %2.2f, %2.2f, rotation: %2.2f, %2.2f, %2.2f\n".format(
//                        it.name, it.id,
//                        it.pos[0], it.pos[1], it.pos[2],
//                        it.rot[0], it.rot[1], it.rot[2]
//                    )
//                }
//            }
//            Log.d(tag, outStr)
//        }
//
//        private data class Data(
//            var id: Int,
//            var name: String?,
//            var pos: ArrayList<Double>,
//            var rot: ArrayList<Double>,
//        )
//    }
//
//}