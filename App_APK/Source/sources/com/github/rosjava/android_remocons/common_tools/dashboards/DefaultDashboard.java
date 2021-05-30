package com.github.rosjava.android_remocons.common_tools.dashboards;

import android.content.Context;
import android.util.AttributeSet;
import android.view.LayoutInflater;
import android.widget.LinearLayout;
import com.github.rosjava.android_extras.gingerbread.view.BatteryLevelView;
import com.github.rosjava.android_remocons.common_tools.R;
import com.github.rosjava.android_remocons.common_tools.dashboards.Dashboard;
import diagnostic_msgs.DiagnosticArray;
import diagnostic_msgs.DiagnosticStatus;
import diagnostic_msgs.KeyValue;
import java.util.HashMap;
import java.util.List;
import org.ros.exception.RosException;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.topic.Subscriber;

public class DefaultDashboard extends LinearLayout implements Dashboard.DashboardInterface {
    private ConnectedNode connectedNode;
    private Subscriber<DiagnosticArray> diagnosticSubscriber;
    private BatteryLevelView laptopBattery;
    private boolean powerOn = false;
    private BatteryLevelView robotBattery;

    public DefaultDashboard(Context context) {
        super(context);
        inflateSelf(context);
    }

    public DefaultDashboard(Context context, AttributeSet attrs) {
        super(context, attrs);
        inflateSelf(context);
    }

    private void inflateSelf(Context context) {
        ((LayoutInflater) context.getSystemService("layout_inflater")).inflate(R.layout.default_dashboard, this);
        this.robotBattery = (BatteryLevelView) findViewById(R.id.robot_battery);
        this.laptopBattery = (BatteryLevelView) findViewById(R.id.laptop_battery);
    }

    /* access modifiers changed from: private */
    public void handleDiagnosticArray(DiagnosticArray msg) {
        for (DiagnosticStatus status : msg.getStatus()) {
            if (status.getName().equals("/Power System/Battery")) {
                populateBatteryFromStatus(this.robotBattery, status);
            }
            if (status.getName().equals("/Power System/Laptop Battery")) {
                populateBatteryFromStatus(this.laptopBattery, status);
            }
        }
    }

    private void populateBatteryFromStatus(BatteryLevelView view, DiagnosticStatus status) {
        HashMap<String, String> values = keyValueArrayToMap(status.getValues());
        try {
            view.setBatteryPercent((float) ((int) ((Float.parseFloat(values.get("Charge (Ah)")) * 100.0f) / Float.parseFloat(values.get("Capacity (Ah)")))));
        } catch (ArithmeticException | NullPointerException | NumberFormatException e) {
        }
        try {
            view.setPluggedIn(Float.parseFloat(values.get("Current (A)")) > 0.0f);
        } catch (ArithmeticException | NullPointerException | NumberFormatException e2) {
        }
    }

    private HashMap<String, String> keyValueArrayToMap(List<KeyValue> list) {
        HashMap<String, String> map = new HashMap<>();
        for (KeyValue kv : list) {
            map.put(kv.getKey(), kv.getValue());
        }
        return map;
    }

    public void onShutdown(Node node) {
        if (this.diagnosticSubscriber != null) {
            this.diagnosticSubscriber.shutdown();
        }
        this.diagnosticSubscriber = null;
        this.connectedNode = null;
    }

    public void onStart(ConnectedNode connectedNode2) {
        this.connectedNode = connectedNode2;
        try {
            Subscriber<DiagnosticArray> subscriber = this.diagnosticSubscriber;
            this.diagnosticSubscriber = connectedNode2.newSubscriber("diagnostics_agg", DiagnosticArray._TYPE);
            this.diagnosticSubscriber.addMessageListener(new MessageListener<DiagnosticArray>() {
                public void onNewMessage(final DiagnosticArray message) {
                    DefaultDashboard.this.post(new Runnable() {
                        public void run() {
                            DefaultDashboard.this.handleDiagnosticArray(message);
                        }
                    });
                }
            });
            connectedNode2.getResolver().newChild(GraphName.of("/turtlebot_node"));
        } catch (Exception ex) {
            this.connectedNode = null;
            throw new RosException((Throwable) ex);
        } catch (RosException e) {
            e.printStackTrace();
        }
    }
}
