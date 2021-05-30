package org.ros.android.android_tutorial_cv_bridge;

import android.content.Intent;
import android.graphics.Bitmap;
import android.os.Bundle;
import android.view.View;
import android.widget.Button;
import android.widget.ImageView;
import android.widget.Toast;
import cv_bridge.CvImage;
import cv_bridge.Format;
import org.apache.commons.logging.Log;
import org.bytedeco.javacpp.opencv_core;
import org.bytedeco.javacpp.opencv_imgproc;
import org.bytedeco.javacpp.opencv_stitching;
import org.ros.android.RosActivity;
import org.ros.android.tutorial_CompressedImage_cv_bridge.R;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMain;
import org.ros.node.NodeMainExecutor;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import sensor_msgs.CompressedImage;
import sensor_msgs.ImageEncodings;

public class MainActivityCompressedJavacv extends RosActivity implements NodeMain {
    protected static final String TAG = "compressed Tutorial";
    Button back1;
    protected Bitmap bmp;
    protected Runnable displayImage;
    protected Publisher<CompressedImage> imagePublisher;
    protected Subscriber<CompressedImage> imageSubscriber;
    protected ImageView imageView;
    Button left1;
    Button mark1;
    public int mark_status = 0;
    protected ConnectedNode node;
    NodeConfiguration nodeConfiguration;
    Button right1;
    public int status = 0;
    Button stop1;
    Button throttle1;

    public MainActivityCompressedJavacv() {
        super("micron Tutorial", "micron Tutorial");
    }

    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        getWindow().addFlags(128);
        setContentView(R.layout.main);
        this.imageView = (ImageView) findViewById(R.id.imageView);
        this.throttle1 = (Button) findViewById(R.id.throttle);
        this.left1 = (Button) findViewById(R.id.left);
        this.right1 = (Button) findViewById(R.id.right);
        this.back1 = (Button) findViewById(R.id.back);
        this.stop1 = (Button) findViewById(R.id.stop);
        this.mark1 = (Button) findViewById(R.id.mark);
        this.displayImage = new Runnable() {
            public void run() {
                MainActivityCompressedJavacv.this.imageView.setImageBitmap(MainActivityCompressedJavacv.this.bmp);
            }
        };
    }

    /* access modifiers changed from: protected */
    public void init(NodeMainExecutor nodeMainExecutor) {
        throttle t1 = new throttle(this);
        mark m1 = new mark(this);
        Battery1 b = new Battery1(this);
        this.nodeConfiguration = NodeConfiguration.newPublic(getRosHostname());
        this.nodeConfiguration.setMasterUri(getMasterUri());
        nodeMainExecutor.execute(this, this.nodeConfiguration);
        nodeMainExecutor.execute(t1, this.nodeConfiguration);
        nodeMainExecutor.execute(m1, this.nodeConfiguration);
        nodeMainExecutor.execute(b, this.nodeConfiguration);
        this.throttle1.setOnClickListener(new View.OnClickListener() {
            public void onClick(View v) {
                MainActivityCompressedJavacv.this.status = 1;
                Toast.makeText(MainActivityCompressedJavacv.this.getApplicationContext(), "Forward", 0).show();
            }
        });
        this.stop1.setOnClickListener(new View.OnClickListener() {
            public void onClick(View v) {
                MainActivityCompressedJavacv.this.status = 0;
                Toast.makeText(MainActivityCompressedJavacv.this.getApplicationContext(), "Stop", 0).show();
            }
        });
        this.left1.setOnClickListener(new View.OnClickListener() {
            public void onClick(View v) {
                MainActivityCompressedJavacv.this.status = 2;
                Toast.makeText(MainActivityCompressedJavacv.this.getApplicationContext(), "Left", 0).show();
            }
        });
        this.right1.setOnClickListener(new View.OnClickListener() {
            public void onClick(View v) {
                MainActivityCompressedJavacv.this.status = 3;
                Toast.makeText(MainActivityCompressedJavacv.this.getApplicationContext(), "Right", 0).show();
            }
        });
        this.back1.setOnClickListener(new View.OnClickListener() {
            public void onClick(View v) {
                MainActivityCompressedJavacv.this.status = 4;
                Toast.makeText(MainActivityCompressedJavacv.this.getApplicationContext(), "Reverse", 0).show();
            }
        });
        this.mark1.setOnClickListener(new View.OnClickListener() {
            public void onClick(View v) {
                MainActivityCompressedJavacv.this.mark_status = 1;
            }
        });
        onResume();
    }

    public GraphName getDefaultNodeName() {
        return GraphName.of("android_tutorial_cv_bridge");
    }

    public void onStart(ConnectedNode connectedNode) {
        this.node = connectedNode;
        final Log log = this.node.getLog();
        this.imagePublisher = this.node.newPublisher("/image_converter/output_video/compressed", CompressedImage._TYPE);
        this.imageSubscriber = this.node.newSubscriber("/camera/image/compressed", CompressedImage._TYPE);
        this.imageSubscriber.addMessageListener(new MessageListener<CompressedImage>() {
            public void onNewMessage(CompressedImage message) {
                try {
                    CvImage cvImage = CvImage.toCvCopy(message, ImageEncodings.RGBA8);
                    if (cvImage.image.rows() > 110 && cvImage.image.cols() > 110) {
                        opencv_imgproc.circle(cvImage.image, new opencv_core.Point(cvImage.image.cols() / 2, cvImage.image.rows() / 2), 100, new opencv_core.Scalar(255.0d, opencv_stitching.Stitcher.ORIG_RESOL, opencv_stitching.Stitcher.ORIG_RESOL, opencv_stitching.Stitcher.ORIG_RESOL));
                    }
                    cvImage.image = cvImage.image.t().asMat();
                    opencv_core.flip(cvImage.image, cvImage.image, 1);
                    MainActivityCompressedJavacv.this.bmp = Bitmap.createBitmap(cvImage.image.cols(), cvImage.image.rows(), Bitmap.Config.ARGB_8888);
                    MainActivityCompressedJavacv.this.bmp.copyPixelsFromBuffer(cvImage.image.createBuffer());
                    MainActivityCompressedJavacv.this.runOnUiThread(MainActivityCompressedJavacv.this.displayImage);
                    opencv_core.flip(cvImage.image, cvImage.image, 1);
                    cvImage.image = cvImage.image.t().asMat();
                    try {
                        MainActivityCompressedJavacv.this.imagePublisher.publish(cvImage.toCompressedImageMsg(MainActivityCompressedJavacv.this.imagePublisher.newMessage(), Format.JPG));
                    } catch (Exception e) {
                        Log log = log;
                        log.error("cv_bridge exception: " + e.getMessage());
                    }
                } catch (Exception e2) {
                    Log log2 = log;
                    log2.error("cv_bridge exception: " + e2.getMessage());
                }
            }
        });
        android.util.Log.i(TAG, "called onStart");
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

    public void nodes(View v) {
        startActivity(new Intent(this, MainActivity.class));
        finish();
    }

    public void map(View v) {
        startActivity(new Intent(this, map1.class));
        finish();
    }

    public void back_nav(View v) {
        finish();
    }
}
