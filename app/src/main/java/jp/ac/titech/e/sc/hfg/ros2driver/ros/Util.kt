package jp.ac.titech.e.sc.hfg.ros2driver.ros

import java.time.Duration
import java.time.Instant

fun now(delta: Long = 0): builtin_interfaces.msg.Time {
    val instant: Instant = Instant.now() - Duration.ofMillis(delta)
    val msg = builtin_interfaces.msg.Time()
    msg.sec = instant.epochSecond.toInt()
    msg.nanosec = instant.nano
    return msg
}