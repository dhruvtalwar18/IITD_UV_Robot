package org.ros.android.android_tutorial_cv_bridge;

import android.content.Intent;
import android.os.Bundle;
import android.view.View;
import org.apache.commons.logging.Log;
import org.ros.android.RosActivity;
import org.ros.android.tutorial_CompressedImage_cv_bridge.R;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMain;
import org.ros.node.NodeMainExecutor;

public class HomePage extends RosActivity implements NodeMain {
    protected ConnectedNode node;
    NodeConfiguration nodeConfiguration;

    public HomePage() {
        super("home", "home");
    }

    /* access modifiers changed from: protected */
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_home_page);
    }

    /* access modifiers changed from: protected */
    public void init(NodeMainExecutor nodeMainExecutor) {
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

    public void manual(View v) {
        startActivity(new Intent(this, MainActivityCompressedJavacv.class));
    }

    public void autonomous(View v) {
        startActivity(new Intent(this, MainActivity.class));
    }

    public void map(View v) {
        startActivity(new Intent(this, map1.class));
    }
}
