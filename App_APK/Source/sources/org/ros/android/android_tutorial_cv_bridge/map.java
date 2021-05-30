package org.ros.android.android_tutorial_cv_bridge;

import android.content.Context;
import android.content.Intent;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Canvas;
import android.graphics.Paint;
import android.graphics.Point;
import android.os.Bundle;
import android.view.MotionEvent;
import android.view.View;
import android.widget.ImageView;
import android.widget.Toast;
import org.apache.commons.logging.Log;
import org.ros.android.RosActivity;
import org.ros.android.tutorial_CompressedImage_cv_bridge.R;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMain;
import org.ros.node.NodeMainExecutor;

public class map extends RosActivity implements NodeMain {
    static int i = 0;
    Bitmap bitmap;
    Canvas canvas;
    ImageView mainLayout;
    Bitmap mutableBitmap;
    BitmapFactory.Options myOptions;
    protected ConnectedNode node;
    NodeConfiguration nodeConfiguration;
    Paint paint;
    int status = 0;
    Bitmap workingBitmap;
    int x;
    int y;

    public map() {
        super("map", "map");
    }

    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_map);
        this.mainLayout = (ImageView) findViewById(R.id.imageView);
        this.myOptions = new BitmapFactory.Options();
        this.bitmap = BitmapFactory.decodeResource(getResources(), R.drawable.map, this.myOptions);
        this.paint = new Paint();
        this.paint.setColor(-16711936);
        this.workingBitmap = Bitmap.createBitmap(this.bitmap);
        this.mutableBitmap = this.workingBitmap.copy(Bitmap.Config.ARGB_4444, true);
        this.canvas = new Canvas(this.mutableBitmap);
        this.mainLayout.setOnTouchListener(new View.OnTouchListener() {
            public boolean onTouch(View v, MotionEvent event) {
                Point touchPlace = new Point();
                touchPlace.set((int) event.getX(), (int) event.getY());
                map.this.x = (int) event.getX();
                map.this.y = (int) event.getY();
                Context applicationContext = map.this.getApplicationContext();
                Toast.makeText(applicationContext, "hh" + touchPlace, 1).show();
                map.i = map.i + 1;
                map.this.drawpoint(map.this.mainLayout, (float) map.this.x, (float) map.this.y, 100);
                return true;
            }
        });
    }

    /* access modifiers changed from: private */
    public void drawpoint(ImageView imageView, float x2, float y2, int raduis) {
        Context applicationContext = getApplicationContext();
        Toast.makeText(applicationContext, "ch" + x2 + " " + y2, 0).show();
        this.myOptions.inPreferredConfig = Bitmap.Config.ARGB_4444;
        this.myOptions.inPurgeable = true;
        this.canvas.drawCircle(4.0f * x2, 3.0f * y2, (float) raduis, this.paint);
        ((ImageView) findViewById(R.id.imageView)).setImageBitmap(this.mutableBitmap);
    }

    public void back_map(View v) {
        startActivity(new Intent(this, MainActivityCompressedJavacv.class));
        Context applicationContext = getApplicationContext();
        Toast.makeText(applicationContext, "" + this.x + " " + this.y, 0).show();
        finish();
    }

    public void ok(View v) {
        this.status = 1;
    }

    /* access modifiers changed from: protected */
    public void init(NodeMainExecutor nodeMainExecutor) {
        this.nodeConfiguration = NodeConfiguration.newPublic(getRosHostname());
        this.nodeConfiguration.setMasterUri(getMasterUri());
        nodeMainExecutor.execute(this, this.nodeConfiguration);
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
