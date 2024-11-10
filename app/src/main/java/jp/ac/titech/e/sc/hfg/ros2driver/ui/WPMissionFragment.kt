package jp.ac.titech.e.sc.hfg.ros2driver.ui


import android.content.ContentResolver
import android.net.Uri
import android.os.Bundle
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.widget.Button
import android.widget.TextView
import android.widget.Toast
import androidx.fragment.app.Fragment
import androidx.fragment.app.activityViewModels
import com.dji.wpmzsdk.common.data.Template
import com.dji.wpmzsdk.manager.WPMZManager
import dji.sdk.wpmz.value.mission.WaylineMission
import dji.sdk.wpmz.value.mission.WaylineMissionConfig
import dji.v5.common.callback.CommonCallbacks
import dji.v5.common.error.IDJIError
import dji.v5.manager.aircraft.waypoint3.WaypointMissionManager
import dji.v5.utils.common.ContextUtil
import dji.v5.utils.common.DiskUtil
import dji.v5.utils.common.ThreadUtil.runOnUiThread
import jp.ac.titech.e.sc.hfg.ros2driver.R
import jp.ac.titech.e.sc.hfg.ros2driver.databinding.FragmentWpmissionBinding
import jp.ac.titech.e.sc.hfg.ros2driver.ros.RosVM
import jp.ac.titech.e.sc.hfg.ros2driver.utils.KMZTestUtil
import jp.ac.titech.e.sc.hfg.ros2driver.utils.KMZTestUtil.createWaylineMission
import jp.ac.titech.e.sc.hfg.ros2driver.utils.wpml.WaypointInfoModel
import java.io.File

import java.nio.file.Files
import java.nio.file.Path
import java.nio.file.StandardCopyOption

class WPMissionFragment : Fragment() {
    private val rosVM: RosVM by activityViewModels()
    private lateinit var binding: FragmentWpmissionBinding

    private val WAYPOINT_SAMPLE_FILE_DIR: String = "waypoint/"
    val rootDir = DiskUtil.getExternalCacheDirPath(ContextUtil.getContext(), WAYPOINT_SAMPLE_FILE_DIR)
    private val showWaypoints : ArrayList<WaypointInfoModel> = ArrayList()

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        arguments?.let {
        }
    }

    override fun onCreateView(
            inflater: LayoutInflater, container: ViewGroup?,
            savedInstanceState: Bundle?
    ): View? {
        binding = FragmentWpmissionBinding.inflate(inflater, container, false)
        binding.rosVM = rosVM
        binding.lifecycleOwner = viewLifecycleOwner

        ////////////////////////////////////////////////////////////////////////////////////////////

        val saveKmzButton: Button = binding.root.findViewById(R.id.saveKmz)
//        var showKmzfilePath:TextView = binding.root.findViewById(R.id.show_kmzfilePath)
        // Set onClick listeners
        saveKmzButton.setOnClickListener { saveKMZ() }

        return binding.root
    }
    ////////////////////////////////////////////////////////////////////////////////////////////////

    private fun saveKMZ() {

        val kmzOutPath = rootDir + "generate_test.kmz"
        val waylineMission: WaylineMission = createWaylineMission()
        val missionConfig: WaylineMissionConfig = KMZTestUtil.createMissionConfig()
        val template: Template = KMZTestUtil.createTemplate(showWaypoints)
        WPMZManager.getInstance()
            .generateKMZFile(kmzOutPath, waylineMission, missionConfig, template)

        runOnUiThread {
                    Toast.makeText(requireContext(), "Save Kmz Success Path is : $kmzOutPath", Toast.LENGTH_LONG).show()
                }
        WaypointMissionManager.getInstance().pushKMZFileToAircraft(kmzOutPath, object :
            CommonCallbacks.CompletionCallbackWithProgress<Double> {
            override fun onProgressUpdate(progress: Double) {
//                missionUploadState.value = MissionUploadStateInfo(updateProgress = progress)
//                refreshMissionState()
                runOnUiThread {
                    Toast.makeText(requireContext(), "Uploading progress: $progress", Toast.LENGTH_LONG).show()
                }
            }

            override fun onSuccess() {
//                missionUploadState.value = MissionUploadStateInfo(tips = "Mission Upload Success")
//                refreshMissionState()
                runOnUiThread {
                    Toast.makeText(requireContext(), "Upload Complete", Toast.LENGTH_LONG).show()
                }
            }

            override fun onFailure(error: IDJIError) {
//                missionUploadState.value = MissionUploadStateInfo(error = error)
//                refreshMissionState()
                runOnUiThread {
                    Toast.makeText(requireContext(), "Error : ${error.description()}", Toast.LENGTH_LONG).show()
                }
            }

        })
    }


    companion object {
        @JvmStatic
        fun newInstance() =

                WPMissionFragment().apply {
                    arguments = Bundle().apply {
                    }
                }
    }
}