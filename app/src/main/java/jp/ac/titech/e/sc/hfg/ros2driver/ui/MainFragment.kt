package jp.ac.titech.e.sc.hfg.ros2driver.ui

import android.os.Bundle
import android.view.LayoutInflater
import android.view.Surface
import android.view.SurfaceHolder
import android.view.View
import android.view.ViewGroup
import androidx.fragment.app.Fragment
import androidx.fragment.app.activityViewModels
import dji.v5.ux.core.extension.hide
import dji.v5.ux.core.extension.toggleVisibility
import io.reactivex.rxjava3.android.schedulers.AndroidSchedulers
import io.reactivex.rxjava3.disposables.CompositeDisposable
import jp.ac.titech.e.sc.hfg.ros2driver.databinding.FragmentMainBinding
import jp.ac.titech.e.sc.hfg.ros2driver.ros.RosVM

class MainFragment : Fragment() {
    private val logtag = "MainFragment"
    private val rosVM: RosVM by activityViewModels()
    private lateinit var binding: FragmentMainBinding
    private var surface: Surface? = null
    private var width = -1
    private var height = -1


    private var compositeDisposable: CompositeDisposable? = null

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
    }

    override fun onCreateView(
        inflater: LayoutInflater, container: ViewGroup?,
        savedInstanceState: Bundle?
    ): View {
        super.onCreateView(inflater, container, savedInstanceState)
        binding = FragmentMainBinding.inflate(inflater, container, false)
        rosVM.flightState.observe(viewLifecycleOwner) {
            binding.textFlightState.text = it
        }

        rosVM.isRosRun.observe(viewLifecycleOwner) {
            if (it) {
                rosVM.nodeManager.imageNode?.startStreamSurface(binding.surfaceView)
            } else {
                rosVM.nodeManager.imageNode?.stopStreamSurface(binding.surfaceView)
            }
        }
        binding.surfaceView.holder.addCallback(cameraSurfaceCallback)

        initClickListener()
        return binding.root
    }

    override fun onResume() {
        super.onResume()

        compositeDisposable = CompositeDisposable()
        compositeDisposable?.add(
            binding.widgetPanelSystemStatusList.closeButtonPressed()
                .observeOn(AndroidSchedulers.mainThread())
                .subscribe { pressed -> if (pressed) binding.widgetPanelSystemStatusList.hide() })
    }

    override fun onPause() {
        super.onPause()
        compositeDisposable?.dispose()
        compositeDisposable = null
    }

    private fun initClickListener() {
        binding.panelTopBar.systemStatusWidget?.setOnClickListener {
            binding.widgetPanelSystemStatusList.toggleVisibility()
        }
    }

    private val cameraSurfaceCallback = object : SurfaceHolder.Callback {
        override fun surfaceCreated(holder: SurfaceHolder) {
            surface = holder.surface
        }

        override fun surfaceChanged(holder: SurfaceHolder, format: Int, width: Int, height: Int) {
            this@MainFragment.width = width
            this@MainFragment.height = height
            updateCameraStream()
        }

        override fun surfaceDestroyed(holder: SurfaceHolder) {
            width = 0
            height = 0
            updateCameraStream()
        }
    }

    private fun updateCameraStream() {
        if (width <= 0 || height <= 0 || surface == null) {
            if (surface != null) {
                rosVM.nodeManager.imageNode?.stopStreamSurface(binding.surfaceView)
            }
            return
        }
        rosVM.nodeManager.imageNode?.startStreamSurface(binding.surfaceView)
    }

    companion object {
        @JvmStatic
        fun newInstance() =
            MainFragment().apply {
            }
    }
}