package org.ros.android.view.visualization;

import android.content.Context;
import android.opengl.GLSurfaceView;
import android.util.AttributeSet;
import android.view.MotionEvent;
import com.google.common.base.Preconditions;
import com.google.common.collect.Lists;
import geometry_msgs.TransformStamped;
import java.util.Collections;
import java.util.List;
import org.ros.android.view.visualization.layer.Layer;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.NodeMainExecutor;
import org.ros.rosjava_geometry.FrameTransformTree;
import tf2_msgs.TFMessage;

public class VisualizationView extends GLSurfaceView implements NodeMain {
    private static final boolean DEBUG = false;
    private final XYOrthographicCamera camera = new XYOrthographicCamera(this.frameTransformTree);
    private ConnectedNode connectedNode;
    /* access modifiers changed from: private */
    public final FrameTransformTree frameTransformTree = new FrameTransformTree();
    private List<Layer> layers;
    /* access modifiers changed from: private */
    public final Object mutex = new Object();
    private XYOrthographicRenderer renderer;

    public VisualizationView(Context context) {
        super(context);
    }

    public VisualizationView(Context context, AttributeSet attrs) {
        super(context, attrs);
    }

    public void onCreate(List<Layer> layers2) {
        this.layers = layers2;
        setDebugFlags(1);
        setEGLConfigChooser(8, 8, 8, 8, 0, 0);
        getHolder().setFormat(-3);
        this.renderer = new XYOrthographicRenderer(this);
        setRenderer(this.renderer);
    }

    public void init(NodeMainExecutor nodeMainExecutor) {
        Preconditions.checkNotNull(this.layers);
        for (Layer layer : this.layers) {
            layer.init(nodeMainExecutor);
        }
    }

    public GraphName getDefaultNodeName() {
        return GraphName.of("android_15/visualization_view");
    }

    public boolean onTouchEvent(MotionEvent event) {
        for (T layer : Lists.reverse(this.layers)) {
            if (layer.onTouchEvent(this, event)) {
                return true;
            }
        }
        return super.onTouchEvent(event);
    }

    public XYOrthographicRenderer getRenderer() {
        return this.renderer;
    }

    public XYOrthographicCamera getCamera() {
        return this.camera;
    }

    public FrameTransformTree getFrameTransformTree() {
        return this.frameTransformTree;
    }

    public List<Layer> getLayers() {
        return Collections.unmodifiableList(this.layers);
    }

    public void onStart(ConnectedNode connectedNode2) {
        this.connectedNode = connectedNode2;
        startTransformListener();
        startLayers();
    }

    private void startTransformListener() {
        this.connectedNode.newSubscriber("tf", TFMessage._TYPE).addMessageListener(new MessageListener<TFMessage>() {
            public void onNewMessage(TFMessage message) {
                synchronized (VisualizationView.this.mutex) {
                    for (TransformStamped transform : message.getTransforms()) {
                        VisualizationView.this.frameTransformTree.update(transform);
                    }
                }
            }
        });
        this.connectedNode.newSubscriber("tf_static", TFMessage._TYPE).addMessageListener(new MessageListener<TFMessage>() {
            public void onNewMessage(TFMessage message) {
                synchronized (VisualizationView.this.mutex) {
                    for (TransformStamped transform : message.getTransforms()) {
                        VisualizationView.this.frameTransformTree.update(transform);
                    }
                }
            }
        });
    }

    private void startLayers() {
        for (Layer layer : this.layers) {
            layer.onStart(this, this.connectedNode);
        }
    }

    public void addLayer(Layer layer) {
        this.layers.add(layer);
    }

    public void onShutdown(Node node) {
        for (Layer layer : this.layers) {
            layer.onShutdown(this, node);
        }
        this.connectedNode = null;
    }

    public void onShutdownComplete(Node node) {
    }

    public void onError(Node node, Throwable throwable) {
    }
}
