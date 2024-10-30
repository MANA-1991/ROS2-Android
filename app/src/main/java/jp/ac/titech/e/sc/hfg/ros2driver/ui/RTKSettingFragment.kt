package jp.ac.titech.e.sc.hfg.ros2driver.ui

import android.os.Bundle
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import androidx.fragment.app.Fragment
import androidx.fragment.app.activityViewModels
import jp.ac.titech.e.sc.hfg.ros2driver.databinding.FragmentRtkSettingBinding
import jp.ac.titech.e.sc.hfg.ros2driver.ros.RosVM

class RTKSettingFragment : Fragment() {
    private val logtag = "RTKSettingFragment"
    private val rosVM: RosVM by activityViewModels()
    private lateinit var binding: FragmentRtkSettingBinding

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        arguments?.let {
        }
    }

    override fun onCreateView(
        inflater: LayoutInflater, container: ViewGroup?,
        savedInstanceState: Bundle?
    ): View {
        binding = FragmentRtkSettingBinding.inflate(inflater, container, false)
        binding.rosVM = rosVM
        binding.lifecycleOwner = viewLifecycleOwner
        rosVM.updateRtkFragmentState()
        return binding.root
    }

    companion object {
        @JvmStatic
        fun newInstance() =
            RTKSettingFragment().apply {
                arguments = Bundle().apply {
                }
            }
    }
}