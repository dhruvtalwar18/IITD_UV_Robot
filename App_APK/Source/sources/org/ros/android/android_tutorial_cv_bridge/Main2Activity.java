package org.ros.android.android_tutorial_cv_bridge;

import android.content.Intent;
import android.os.Bundle;
import android.view.Menu;
import android.view.MenuItem;
import android.view.View;
import android.widget.Button;
import com.github.rosjava.android_remocons.common_tools.apps.RosAppActivity;
import java.io.IOException;
import java.net.InetAddress;
import java.net.Socket;
import org.ros.android.tutorial_CompressedImage_cv_bridge.R;
import org.ros.android.view.RosImageView;
import org.ros.android.view.VirtualJoystickView;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;
import sensor_msgs.CompressedImage;

public class Main2Activity extends RosAppActivity {
    private Button backButton;
    private RosImageView<CompressedImage> cameraView;
    private VirtualJoystickView virtualJoystickView;

    public Main2Activity() {
        super("android teleop", "android teleop");
    }

    public void onCreate(Bundle savedInstanceState) {
        setDashboardResource(R.id.top_bar);
        setMainWindowResource(R.layout.activity_main2);
        super.onCreate(savedInstanceState);
        this.cameraView = (RosImageView) findViewById(R.id.image);
        this.virtualJoystickView = (VirtualJoystickView) findViewById(R.id.virtual_joystick);
        this.backButton = (Button) findViewById(R.id.back_button);
        this.backButton.setOnClickListener(new View.OnClickListener() {
            public void onClick(View view) {
                Main2Activity.this.startActivity(new Intent(Main2Activity.this.getApplicationContext(), MainActivity.class));
                Main2Activity.this.finish();
            }
        });
    }

    /* access modifiers changed from: protected */
    public void init(NodeMainExecutor nodeMainExecutor) {
        super.init(nodeMainExecutor);
        try {
            Socket socket = new Socket(getMasterUri().getHost(), getMasterUri().getPort());
            InetAddress local_network_address = socket.getLocalAddress();
            socket.close();
            NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(local_network_address.getHostAddress(), getMasterUri());
            this.virtualJoystickView.setTopicName(getMasterNameSpace().resolve(this.remaps.get(getString(R.string.joystick_topic))).toString());
            nodeMainExecutor.execute(this.virtualJoystickView, nodeConfiguration.setNodeName("android/virtual_joystick"));
        } catch (IOException e) {
        }
    }

    public boolean onCreateOptionsMenu(Menu menu) {
        menu.add(0, 0, 0, R.string.stop_app);
        return super.onCreateOptionsMenu(menu);
    }

    public boolean onOptionsItemSelected(MenuItem item) {
        super.onOptionsItemSelected(item);
        if (item.getItemId() != 0) {
            return true;
        }
        onDestroy();
        return true;
    }
}
