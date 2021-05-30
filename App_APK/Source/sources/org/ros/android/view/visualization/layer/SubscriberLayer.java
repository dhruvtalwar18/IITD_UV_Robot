package org.ros.android.view.visualization.layer;

import com.google.common.base.Preconditions;
import org.ros.android.view.visualization.VisualizationView;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.topic.Subscriber;

public class SubscriberLayer<T> extends DefaultLayer {
    private final String messageType;
    private Subscriber<T> subscriber;
    private final GraphName topicName;

    public SubscriberLayer(GraphName topicName2, String messageType2) {
        this.topicName = topicName2;
        this.messageType = messageType2;
    }

    public void onStart(VisualizationView view, ConnectedNode connectedNode) {
        super.onStart(view, connectedNode);
        this.subscriber = connectedNode.newSubscriber(this.topicName, this.messageType);
    }

    public void onShutdown(VisualizationView view, Node node) {
        this.subscriber.shutdown();
        super.onShutdown(view, node);
    }

    public Subscriber<T> getSubscriber() {
        Preconditions.checkNotNull(this.subscriber);
        return this.subscriber;
    }
}
