package org.ros.android.android_tutorial_cv_bridge;

import android.os.Bundle;
import android.view.View;
import android.widget.TextView;
import org.apache.commons.logging.Log;
import org.ros.android.RosActivity;
import org.ros.android.tutorial_CompressedImage_cv_bridge.R;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMain;
import org.ros.node.NodeMainExecutor;

public class map1 extends RosActivity implements NodeMain {
    int area = 1;
    protected ConnectedNode node;
    NodeConfiguration nodeConfiguration;
    PaintView paintView;
    int status = 0;
    int x;
    int y;

    public map1() {
        super("map", "map");
    }

    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_map1);
        this.paintView = (PaintView) findViewById(R.id.paint_view);
        this.paintView.setTextView((TextView) findViewById(R.id.tv_coordinates));
        this.paintView.setOnTouchListener(this.paintView);
    }

    public void back_map(View v) {
        finish();
    }

    public void ok(View v) {
        this.x = (int) this.paintView.mX;
        this.y = (int) this.paintView.mY;
        this.status = 1;
    }

    public void area1(View v) {
        this.area = 1;
    }

    public void area2(View v) {
        this.area = 2;
    }

    /* access modifiers changed from: protected */
    public void init(NodeMainExecutor nodeMainExecutor) {
        points_node t1 = new points_node(this);
        this.nodeConfiguration = NodeConfiguration.newPublic(getRosHostname());
        this.nodeConfiguration.setMasterUri(getMasterUri());
        nodeMainExecutor.execute(this, this.nodeConfiguration);
        nodeMainExecutor.execute(t1, this.nodeConfiguration);
        onResume();
    }

    public void onStart(ConnectedNode connectedNode) {
        this.node = connectedNode;
        Log log = this.node.getLog();
    }

    public GraphName getDefaultNodeName() {
        return GraphName.of("android_tutorial_cv_bridge");
    }

    public void onResume() {
        super.onResume();
    }

    public void onShutdown(Node node2) {
    }

    public void onShutdownComplete(Node node2) {
    }

    public void onError(Node node2, Throwable throwable) {
    }
}
