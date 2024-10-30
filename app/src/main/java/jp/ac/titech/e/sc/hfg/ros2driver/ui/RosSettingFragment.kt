package jp.ac.titech.e.sc.hfg.ros2driver.ui

import ArrowShapeRenderer
import android.graphics.Color
import android.os.Bundle
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import androidx.fragment.app.Fragment
import androidx.fragment.app.activityViewModels
import com.github.mikephil.charting.charts.ScatterChart
import com.github.mikephil.charting.data.Entry
import com.github.mikephil.charting.data.ScatterData
import com.github.mikephil.charting.data.ScatterDataSet
import com.github.mikephil.charting.formatter.ValueFormatter
import geometry_msgs.msg.Pose
import geometry_msgs.msg.Quaternion
import geometry_msgs.msg.Vector3
import jp.ac.titech.e.sc.hfg.ros2driver.databinding.FragmentRosSettingBinding
import jp.ac.titech.e.sc.hfg.ros2driver.ros.RosVM
import kotlin.math.asin
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin

class RosSettingFragment : Fragment() {
    private val logtag = "RosSettingFragment"
    private val rosVM: RosVM by activityViewModels()
    private lateinit var binding: FragmentRosSettingBinding
    private lateinit var chart: ScatterChart

    override fun onCreateView(
        inflater: LayoutInflater, container: ViewGroup?, savedInstanceState: Bundle?
    ): View {
        binding = FragmentRosSettingBinding.inflate(inflater, container, false)
        binding.rosVM = rosVM
        binding.lifecycleOwner = viewLifecycleOwner
        chart = binding.scatterChart
        rosVM.position.observe(viewLifecycleOwner) {
            updateChart(it, rosVM.bodyVel.value ?: Vector3())
        }
        return binding.root
    }

    private fun updateChart(pose: Pose, velocity: Vector3) {
        // position scatter
        val positionEntry: ArrayList<Entry> =
            arrayListOf(Entry(pose.position.x.toFloat(), pose.position.y.toFloat()).apply {
                data = pose
            })
        val positionDataset = ScatterDataSet(positionEntry, "position")
        positionDataset.color = Color.RED
        positionDataset.valueFormatter = valueFormatter

        // velocity arrow
        val bodyVelX = velocity.x
        val bodyVelY = velocity.y
        val theta = pose.orientation.yaw
        val velocityGain = 2.0
        val worldVelX = bodyVelX * cos(theta) - bodyVelY * sin(theta)
        val worldVelY = bodyVelX * sin(theta) + bodyVelY * cos(theta)
        val velEntry = if (bodyVelY != 0.0 || bodyVelX != 0.0) arrayListOf(
            Entry(
                pose.position.x.toFloat(), pose.position.y.toFloat(), floatArrayOf(
                    (pose.position.x + velocityGain * worldVelX).toFloat(),
                    (pose.position.y + velocityGain * worldVelY).toFloat()
                )
            )
        ) else arrayListOf()
        val velDataset = ScatterDataSet(velEntry, "velocity")
        velDataset.color = Color.BLUE
        velDataset.shapeRenderer =
            ArrowShapeRenderer(chart.getTransformer(velDataset.axisDependency))

        chart.apply {
            data = ScatterData(positionDataset, velDataset)
            axisRight.isEnabled = false
            xAxis.axisMinimum =
                if (-3 < pose.position.x && pose.position.x < 3) -3.0f else pose.position.x.toFloat() - 3.0f
            xAxis.axisMaximum =
                if (-3 < pose.position.x && pose.position.x < 3) 3.0f else pose.position.x.toFloat() + 3.0f
            axisLeft.axisMinimum =
                if (-3 < pose.position.y && pose.position.y < 3) -3.0f else pose.position.y.toFloat() - 3.0f
            axisLeft.axisMaximum =
                if (-3 < pose.position.y && pose.position.y < 3) 3.0f else pose.position.y.toFloat() + 3.0f
            setDrawGridBackground(true)
            setPinchZoom(true)
            description.isEnabled = false
            invalidate()
        }
    }

    companion object {
        private val valueFormatter = object : ValueFormatter() {
            override fun getPointLabel(entry: Entry?): String {
                val data = entry?.data as? Pose
                if (data != null) {
                    val x = String.format("%8.2f", data.position.x)
                    val y = String.format("%8.2f", data.position.y)
                    val z = String.format("%8.2f", data.position.z)
                    return "(${x}, ${y}, ${z})"
                }
                return super.getPointLabel(entry)
            }
        }

        @JvmStatic
        fun newInstance() = RosSettingFragment().apply {
            arguments = Bundle().apply {}
        }

        val Quaternion.roll: Double
            get() = atan2(2 * (y * z + w * x), w * w - x * x - y * y + z * z)

        val Quaternion.pitch: Double
            get() = -asin(2 * (x * z - w * y))

        val Quaternion.yaw: Double
            get() = atan2(2 * (x * y + w * z), w * w + x * x - y * y - z * z)
    }
}