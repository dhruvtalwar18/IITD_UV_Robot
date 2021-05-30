package com.github.rosjava.android_remocons.common_tools.dashboards;

import android.app.Activity;
import android.content.Context;
import android.util.Log;
import android.view.View;
import android.view.ViewGroup;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeMain;

public class Dashboard implements NodeMain {
    private static String customDashboardPath = null;
    private static final String defaultDashboardPath = "com.github.rosjava.android_remocons.common_tools.dashboards.DefaultDashboard";
    private static final String pr2DashboardPath = "com.ros.pr2.apps.core_components.Pr2Dashboard";
    private static String robotName = null;
    private static final String turtlebotDashboardPath = "com.github.turtlebot.turtlebot_android.turtlebot_core.dashboards.TurtlebotDashboard";
    private Activity activity;
    /* access modifiers changed from: private */
    public DashboardInterface dashboard = null;
    /* access modifiers changed from: private */
    public ViewGroup.LayoutParams lparams;
    /* access modifiers changed from: private */
    public ViewGroup view;

    public interface DashboardInterface {
        void onShutdown(Node node);

        void onStart(ConnectedNode connectedNode);
    }

    public Dashboard(Activity activity2) {
        this.activity = activity2;
        this.view = null;
        this.lparams = null;
    }

    public void setView(ViewGroup view2, ViewGroup.LayoutParams lparams2) {
        if (view2 == null) {
            Log.e("Dashboard", "Null view for dashboard");
        }
        this.view = view2;
        this.lparams = lparams2;
    }

    public void setRobotName(String name) {
        robotName = name;
    }

    public void setCustomDashboardPath(String path) {
        customDashboardPath = path;
    }

    private static DashboardInterface createDashboard(Class dashClass, Context context) {
        ClassLoader classLoader = Dashboard.class.getClassLoader();
        try {
            return (DashboardInterface) dashClass.getConstructor(new Class[]{Class.forName("android.content.Context")}).newInstance(new Object[]{context});
        } catch (Exception ex) {
            Log.e("Dashboard", "Error during dashboard instantiation:", ex);
            return null;
        }
    }

    private static DashboardInterface createDashboard(String className, Context context) {
        try {
            return createDashboard(Class.forName(className), context);
        } catch (Exception ex) {
            Log.e("Dashboard", "Error during dashboard class loading:", ex);
            return null;
        }
    }

    private static DashboardInterface createDashboard(Context context) {
        if (customDashboardPath != null) {
            return createDashboard(customDashboardPath, context);
        }
        return createDashboard(defaultDashboardPath, context);
    }

    public void onError(Node arg0, Throwable arg1) {
    }

    public void onShutdown(final Node node) {
        this.activity.runOnUiThread(new Runnable() {
            public void run() {
                DashboardInterface dash = Dashboard.this.dashboard;
                if (dash != null) {
                    dash.onShutdown(node);
                    Dashboard.this.view.removeView((View) dash);
                }
                DashboardInterface unused = Dashboard.this.dashboard = null;
            }
        });
    }

    public void onShutdownComplete(Node node) {
    }

    public void onStart(ConnectedNode connectedNode) {
        if (this.dashboard == null) {
            this.dashboard = createDashboard(this.activity);
            if (this.dashboard != null) {
                this.activity.runOnUiThread(new Runnable() {
                    public void run() {
                        DashboardInterface dash = Dashboard.this.dashboard;
                        ViewGroup localView = Dashboard.this.view;
                        if (dash != null && localView != null) {
                            localView.addView((View) dash, Dashboard.this.lparams);
                        } else if (dash == null) {
                            Log.e("Dashboard", "Dashboard could not start: no dashboard");
                        } else if (Dashboard.this.view == null) {
                            Log.e("Dashboard", "Dashboard could not start: no view");
                        } else {
                            Log.e("Dashboard", "Dashboard could not start: no view or dashboard");
                        }
                    }
                });
                this.dashboard.onStart(connectedNode);
            }
        }
    }

    public GraphName getDefaultNodeName() {
        return null;
    }
}
