package jp.ac.titech.e.sc.hfg.ros2driver.ui

import androidx.fragment.app.Fragment
import androidx.fragment.app.FragmentActivity
import androidx.viewpager2.adapter.FragmentStateAdapter
import jp.ac.titech.e.sc.hfg.ros2driver.R

class ViewPagerAdapter(fragmentActivity: FragmentActivity) :
    FragmentStateAdapter(fragmentActivity) {
    val titleIds = listOf("Summary", "ROS", "RTK","Anchor","WaypointMission")
    val icons = listOf(
            dji.v5.ux.R.drawable.uxsdk_ic_topbar_flight_mode, R.drawable.ros_icon,
            dji.v5.ux.R.drawable.uxsdk_ic_topbar_gps,
            dji.v5.ux.R.drawable.uxsdk_arrow_right,
            dji.v5.ux.R.drawable.uxsdk_mission_edit_panel_tab_bg_selected
    )
    private val fragments = listOf(
            MainFragment.newInstance(),
            RosSettingFragment.newInstance(),
            RTKSettingFragment.newInstance(),
            AnchorFragment.newInstance(),
            WPMissionFragment.newInstance()
    )

    override fun getItemCount(): Int {
        return fragments.size
    }

    override fun createFragment(position: Int): Fragment {
        return fragments[position]
    }
}