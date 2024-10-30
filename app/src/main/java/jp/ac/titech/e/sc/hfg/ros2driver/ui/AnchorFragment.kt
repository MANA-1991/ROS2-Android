package jp.ac.titech.e.sc.hfg.ros2driver.ui

import android.os.Bundle
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import androidx.fragment.app.Fragment
import androidx.fragment.app.activityViewModels
import jp.ac.titech.e.sc.hfg.ros2driver.databinding.FragmentAnchorBinding
import jp.ac.titech.e.sc.hfg.ros2driver.ros.RosVM

class AnchorFragment : Fragment() {
    private val logtag = "AnchorFragment"
    private val rosVM: RosVM by activityViewModels()
    private lateinit var binding: FragmentAnchorBinding

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        arguments?.let {
        }
    }

    override fun onCreateView(
            inflater: LayoutInflater, container: ViewGroup?,
            savedInstanceState: Bundle?
    ): View {
        binding = FragmentAnchorBinding.inflate(inflater, container, false)
        binding.rosVM = rosVM
        binding.lifecycleOwner = viewLifecycleOwner
        return binding.root
    }

    companion object {
        @JvmStatic
        fun newInstance() =
                AnchorFragment().apply {
                    arguments = Bundle().apply {
                    }
                }
    }
}