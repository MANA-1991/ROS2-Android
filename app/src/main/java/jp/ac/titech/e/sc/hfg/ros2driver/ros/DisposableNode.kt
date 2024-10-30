package jp.ac.titech.e.sc.hfg.ros2driver.ros

import org.ros2.rcljava.node.BaseComposableNode

open class DisposableNode(name:String?):BaseComposableNode(name) {

    open fun dispose(){
        node.timers.clear()
        node.dispose()
    }
}