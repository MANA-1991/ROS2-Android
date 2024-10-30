package jp.ac.titech.e.sc.hfg.ros2driver.ros

import android.util.Log
import android.view.SurfaceView
import dji.sdk.keyvalue.key.CameraKey
import dji.sdk.keyvalue.key.FlightControllerKey
import dji.sdk.keyvalue.key.KeyTools
import dji.sdk.keyvalue.key.ProductKey
import dji.sdk.keyvalue.value.camera.CameraMode
import dji.sdk.keyvalue.value.common.ComponentIndexType
import dji.sdk.keyvalue.value.common.EmptyMsg
import dji.sdk.keyvalue.value.product.ProductType
import dji.v5.common.callback.CommonCallbacks
import dji.v5.common.error.IDJIError
import dji.v5.common.video.channel.VideoChannelState
import dji.v5.common.video.channel.VideoChannelType
import dji.v5.common.video.interfaces.StreamDataListener
import dji.v5.manager.KeyManager
import dji.v5.manager.datacenter.camera.CameraStreamManager
import dji.v5.manager.datacenter.video.VideoStreamManager
import dji.v5.manager.interfaces.ICameraStreamManager
import ffmpeg_image_transport_msgs.msg.FFMPEGPacket
import jp.ac.titech.e.sc.hfg.ros2driver.CameraParameter
import jp.ac.titech.e.sc.hfg.ros2driver.CameraParameters
import org.ros2.rcljava.consumers.Consumer
import org.ros2.rcljava.publisher.Publisher
import org.ros2.rcljava.subscription.Subscription
import sensor_msgs.msg.CameraInfo
import sensor_msgs.msg.RegionOfInterest
import std_msgs.msg.Bool
import std_msgs.msg.Empty
import java.util.concurrent.TimeUnit

class ImageNode(
    val name: String?
) : DisposableNode(name) {
    // Topic
    private val ffmpegStreamTopic = "/$name/image/ffmpeg"
    private val cameraInfoTopic = "/$name/image/camera_info"
    private val shootTopic = "/$name/shoot"
    private val recordTopic = "/$name/record"

    // Publisher
    private val ffmpegPacketPublisher: Publisher<FFMPEGPacket>
    private val cameraInfoPublisher: Publisher<CameraInfo>

    // Publish frequency
    private val cameraInfoPublishFrequency = 1

    // Subscription
    private val shootSubscription: Subscription<Empty>
    private val recordSubscription: Subscription<Bool>

    private var onPublishImage: Boolean = false
    private var onDispose: Boolean = false
    private var width = -1
    private var height = -1
    private var isStreamStart = false

    private val serialNumber: String? =
        keyManager.getValue(KeyTools.createKey(FlightControllerKey.KeySerialNumber))

    private val cameraParameter: CameraParameter

    //    private val receiveStreamListener: ReceiveStreamListener
    private val streamDataListener: StreamDataListener


    init {
        // create publishers, subscribers and services here
        ffmpegPacketPublisher = node.createPublisher(FFMPEGPacket::class.java, ffmpegStreamTopic)
        cameraInfoPublisher = node.createPublisher(CameraInfo::class.java, cameraInfoTopic)
        shootSubscription = node.createSubscription(Empty::class.java, shootTopic, shootCallback)
        recordSubscription = node.createSubscription(Bool::class.java, recordTopic, recordCallback)
//        receiveStreamListener =
//            ReceiveStreamListener { data, offset, length, info ->
//                if (onDispose) return@ReceiveStreamListener
//                width = info.width
//                height = info.height
//                val msg = FFMPEGPacket().apply {
//                    header.stamp = now()
//                    header.frameId = "/$name/camera_link"
//                    width = info.width
//                    height = info.height
//                    encoding = "libx264"
//                    pts = info.presentationTimeMs
//                    flags = if (info.isKeyFrame) 1 else 0
//                    setData(data)
//                    isBigendian = true
//                }
//                onPublishImage = true
//                ffmpegPacketPublisher.publish(msg)
//                onPublishImage = false
//                isStreamStart = true
//            }
        val streamLatency: Long =
            when (keyManager.getValue(KeyTools.createKey(ProductKey.KeyProductType))) {
                ProductType.DJI_MAVIC_3_ENTERPRISE_SERIES, ProductType.DJI_MINI_3 -> 200
                ProductType.DJI_MINI_3_PRO -> 120
                else -> 200
            }
        streamDataListener = StreamDataListener {
            if (onDispose) return@StreamDataListener
            width = it.width
            height = it.height
            val msg = FFMPEGPacket().apply {
                header.stamp = now(streamLatency)
                header.frameId = "/$name/camera_link"
                width = it.width
                height = it.height
                encoding = "libx264"
                pts = it.pts
                flags = if (it.isIFrame) 1 else 0
                setData(it.data)
                isBigendian = true
            }
            onPublishImage = true
            ffmpegPacketPublisher.publish(msg)
            onPublishImage = false
            isStreamStart = true
//            Log.d(tag,msg.header.stamp.sec.toString()+"."+msg.header.stamp.nanosec.toString())
        }
        serialNumber?.let { Log.d(tag, it) }
        cameraParameter = CameraParameters.getOrDefault(
            serialNumber,
            CameraParameters["default"]!!
        )
        initVideoStream()
        startRosTimer()
    }

    override fun dispose() {
        onDispose = true
        while (onPublishImage) {
            Log.d(tag, "wait for finish publish")
            Thread.sleep(50)
        }
//        cameraStreamManager.removeReceiveStreamListener(receiveStreamListener)
        videoStreamManager.getAvailableVideoChannel(VideoChannelType.PRIMARY_STREAM_CHANNEL)
            ?.removeStreamDataListener(streamDataListener)
        super.dispose()
    }

    // Add publishers to timer
    private fun startRosTimer() {
        node.createWallTimer(
            (1000 / cameraInfoPublishFrequency).toLong(),
            TimeUnit.MILLISECONDS,
            this::cameraInfoTask
        )
    }

    private fun cameraInfoTask() {
        if (isStreamStart) {
            // region of interest
            val roi = RegionOfInterest()
            roi.xOffset = 0
            roi.yOffset = 0
            roi.height = height
            roi.width = width
            roi.doRectify = false
            // camera info
            val msg = CameraInfo()
            msg.header.stamp = now()
            msg.header.frameId = "/$name/camera_link"
            msg.height = height
            msg.width = width
            msg.distortionModel = "plumb_bob"
            msg.d = cameraParameter.D.asList()
            msg.k = cameraParameter.K.asList()
            msg.r = cameraParameter.R.asList()
            msg.p = cameraParameter.P.asList()
            msg.binningX = 1
            msg.binningY = 1
            msg.roi = roi
            cameraInfoPublisher.publish(msg)
        }
    }

    // start video stream of the drone
    private fun initVideoStream() {
//        cameraStreamManager.addReceiveStreamListener(
//            ComponentIndexType.LEFT_OR_MAIN,
//            receiveStreamListener
//        )
        val channel =
            videoStreamManager.getAvailableVideoChannel(VideoChannelType.PRIMARY_STREAM_CHANNEL)
        if (channel?.videoChannelStatus == VideoChannelState.CLOSE) channel.startChannel(
            channel.streamSource,
            startChannelCallback
        )
        channel?.addStreamDataListener(streamDataListener)
    }

    fun startStreamSurface(surfaceView: SurfaceView) {
        cameraStreamManager.putCameraStreamSurface(
            ComponentIndexType.LEFT_OR_MAIN,
            surfaceView.holder.surface,
            surfaceView.width,
            surfaceView.height,
            ICameraStreamManager.ScaleType.CENTER_INSIDE
        )
    }

    fun stopStreamSurface(surfaceView: SurfaceView) {
        cameraStreamManager.removeCameraStreamSurface(surfaceView.holder.surface)
    }

    companion object {
        private val tag = ImageNode::class.java.name
        private val keyManager = KeyManager.getInstance()
        private val cameraStreamManager = CameraStreamManager.getInstance()
        private val videoStreamManager = VideoStreamManager.getInstance()

        private val shootCallback = Consumer<Empty> { msg ->
            keyManager.setValue(
                KeyTools.createKey(CameraKey.KeyCameraMode),
                CameraMode.PHOTO_NORMAL,
                cameraModeCallback
            )
            keyManager.performAction(
                KeyTools.createKey(CameraKey.KeyStartShootPhoto),
                startShootPhotoCallback
            )
        }

        private val recordCallback = Consumer<Bool> { msg ->
            if (msg.data) {
                keyManager.setValue(
                    KeyTools.createKey(CameraKey.KeyCameraMode),
                    CameraMode.VIDEO_NORMAL,
                    cameraModeCallback
                )
                keyManager.performAction(
                    KeyTools.createKey(CameraKey.KeyStartRecord),
                    startRecordCallback
                )
            } else if (!msg.data) {
                val recordingTime = keyManager.getValue(
                    KeyTools.createKey(CameraKey.KeyRecordingTime),
                )
                Log.d(tag, "Video recorded for ${recordingTime}s")
                keyManager.performAction(
                    KeyTools.createKey(CameraKey.KeyStopRecord),
                    stopRecordCallback
                )
            }
        }

        private val cameraModeCallback =
            object : CommonCallbacks.CompletionCallback {
                override fun onSuccess() {
                    Log.d(tag, "CameraMode success")
                }

                override fun onFailure(error: IDJIError) {
                    Log.e(tag, "CameraMode Failure: ${error.errorCode()}")
                }

            }

        private val startShootPhotoCallback =
            object : CommonCallbacks.CompletionCallbackWithParam<EmptyMsg> {
                override fun onSuccess(t: EmptyMsg?) {
                    Log.d(tag, "StartShootPhoto success")
                }

                override fun onFailure(error: IDJIError) {
                    Log.e(tag, "StartShootPhoto Failure: ${error.errorCode()}")
                }

            }

        private val startRecordCallback =
            object : CommonCallbacks.CompletionCallbackWithParam<EmptyMsg> {
                override fun onSuccess(t: EmptyMsg?) {
                    Log.d(tag, "StartRecord success")
                }

                override fun onFailure(error: IDJIError) {
                    Log.e(tag, "StartRecord Failure: ${error.errorCode()}")
                }

            }

        private val stopRecordCallback =
            object : CommonCallbacks.CompletionCallbackWithParam<EmptyMsg> {
                override fun onSuccess(t: EmptyMsg?) {
                    Log.d(tag, "StopRecord success")
                }

                override fun onFailure(error: IDJIError) {
                    Log.e(tag, "StopRecord Failure: ${error.errorCode()}")
                }

            }
        private val startChannelCallback =
            object : CommonCallbacks.CompletionCallback {
                override fun onSuccess() {
                    Log.d(tag, "StartChannel success")
                }

                override fun onFailure(error: IDJIError) {
                    Log.e(tag, "StartChannel Failure: ${error.errorCode()}")
                }
            }

    }
}
