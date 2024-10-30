package jp.ac.titech.e.sc.hfg.ros2driver.ros;

import org.ros2.rcljava.node.BaseComposableNode;
import org.ros2.rcljava.subscription.Subscription;

public class ListenerNode extends BaseComposableNode {
    private final String topic;
    private Subscription<geometry_msgs.msg.Vector3> subscriber;

    public ListenerNode(final String name, final String topic) {
        super(name);
        this.topic = topic;
        this.subscriber = this.node.<geometry_msgs.msg.Vector3>createSubscription(geometry_msgs.msg.Vector3.class, this.topic, ListenerNode::subCallback);
    }

    public static void subCallback(final geometry_msgs.msg.Vector3 msg) {
    }
}