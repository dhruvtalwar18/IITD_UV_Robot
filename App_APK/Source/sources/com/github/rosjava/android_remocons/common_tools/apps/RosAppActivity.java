package com.github.rosjava.android_remocons.common_tools.apps;

import android.app.AlertDialog;
import android.content.DialogInterface;
import android.content.Intent;
import android.os.AsyncTask;
import android.os.Bundle;
import android.util.Log;
import android.widget.LinearLayout;
import com.github.robotics_in_concert.rocon_rosjava_core.rocon_interactions.InteractionMode;
import com.github.rosjava.android_remocons.common_tools.dashboards.Dashboard;
import com.github.rosjava.android_remocons.common_tools.master.MasterDescription;
import java.io.Serializable;
import java.net.URI;
import java.net.URISyntaxException;
import java.util.LinkedHashMap;
import org.ros.address.InetAddressFactory;
import org.ros.android.RosActivity;
import org.ros.exception.RosRuntimeException;
import org.ros.namespace.NameResolver;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;
import org.yaml.snakeyaml.Yaml;

public abstract class RosAppActivity extends RosActivity {
    private String androidApplicationName;
    private InteractionMode appMode = InteractionMode.STANDALONE;
    private Dashboard dashboard = null;
    private int dashboardResourceId = 0;
    private String defaultMasterAppName = null;
    private String defaultMasterName = "";
    private int mainWindowId = 0;
    private String masterAppName = null;
    protected MasterDescription masterDescription;
    protected MasterNameResolver masterNameResolver;
    private NodeConfiguration nodeConfiguration;
    private NodeMainExecutor nodeMainExecutor;
    protected AppParameters params = new AppParameters();
    protected AppRemappings remaps = new AppRemappings();
    private String remoconActivity = null;
    private Serializable remoconExtraData = null;

    /* access modifiers changed from: protected */
    public void setDashboardResource(int resource) {
        this.dashboardResourceId = resource;
    }

    /* access modifiers changed from: protected */
    public void setMainWindowResource(int resource) {
        this.mainWindowId = resource;
    }

    /* access modifiers changed from: protected */
    public void setDefaultMasterName(String name) {
        this.defaultMasterName = name;
    }

    /* access modifiers changed from: protected */
    public void setDefaultAppName(String name) {
        this.defaultMasterAppName = name;
    }

    /* access modifiers changed from: protected */
    public void setCustomDashboardPath(String path) {
        this.dashboard.setCustomDashboardPath(path);
    }

    protected RosAppActivity(String notificationTicker, String notificationTitle) {
        super(notificationTicker, notificationTitle);
        this.androidApplicationName = notificationTitle;
    }

    public void onCreate(Bundle savedInstanceState) {
        LinkedHashMap<String, String> remapsList;
        LinkedHashMap<String, Object> paramsList;
        super.onCreate(savedInstanceState);
        if (this.mainWindowId == 0) {
            Log.e("RosApp", "You must set the dashboard resource ID in your RosAppActivity");
        } else if (this.dashboardResourceId == 0) {
            Log.e("RosApp", "You must set the dashboard resource ID in your RosAppActivity");
        } else {
            requestWindowFeature(1);
            getWindow().setFlags(1024, 1024);
            setContentView(this.mainWindowId);
            this.masterNameResolver = new MasterNameResolver();
            if (this.defaultMasterName != null) {
                this.masterNameResolver.setMasterName(this.defaultMasterName);
            }
            InteractionMode[] values = InteractionMode.values();
            int length = values.length;
            int i = 0;
            while (true) {
                if (i >= length) {
                    break;
                }
                InteractionMode mode = values[i];
                Intent intent = getIntent();
                this.masterAppName = intent.getStringExtra("com.github.rosjava.android_remocons.common_tools.rocon.Constants." + mode + "_app_name");
                if (this.masterAppName != null) {
                    this.appMode = mode;
                    break;
                }
                i++;
            }
            if (this.masterAppName == null) {
                Log.e("RosApp", "We are running as standalone :(");
                this.masterAppName = this.defaultMasterAppName;
                this.appMode = InteractionMode.STANDALONE;
            } else {
                Yaml yaml = new Yaml();
                String paramsStr = getIntent().getStringExtra("Parameters");
                String remapsStr = getIntent().getStringExtra("Remappings");
                Log.d("RosApp", "Parameters: " + paramsStr);
                Log.d("RosApp", "Remappings: " + remapsStr);
                if (paramsStr != null) {
                    try {
                        if (!paramsStr.isEmpty() && (paramsList = (LinkedHashMap) yaml.load(paramsStr)) != null) {
                            this.params.putAll(paramsList);
                            Log.d("RosApp", "Parameters: " + paramsStr);
                        }
                    } catch (ClassCastException e) {
                        Log.e("RosApp", "Cannot cast parameters yaml string to a hash map (" + paramsStr + ")");
                        throw new RosRuntimeException("Cannot cast parameters yaml string to a hash map (" + paramsStr + ")");
                    }
                }
                if (remapsStr != null) {
                    try {
                        if (!remapsStr.isEmpty() && (remapsList = (LinkedHashMap) yaml.load(remapsStr)) != null) {
                            this.remaps.putAll(remapsList);
                            Log.d("RosApp", "Remappings: " + remapsStr);
                        }
                    } catch (ClassCastException e2) {
                        Log.e("RosApp", "Cannot cast parameters yaml string to a hash map (" + remapsStr + ")");
                        throw new RosRuntimeException("Cannot cast parameters yaml string to a hash map (" + remapsStr + ")");
                    }
                }
                this.remoconActivity = getIntent().getStringExtra("RemoconActivity");
                if (getIntent().hasExtra(MasterDescription.UNIQUE_KEY)) {
                    this.remoconExtraData = getIntent().getSerializableExtra(MasterDescription.UNIQUE_KEY);
                    try {
                        this.masterDescription = (MasterDescription) getIntent().getSerializableExtra(MasterDescription.UNIQUE_KEY);
                    } catch (ClassCastException e3) {
                        Log.e("RosApp", "Master description expected on intent on " + this.appMode + " mode");
                        throw new RosRuntimeException("Master description expected on intent on " + this.appMode + " mode");
                    }
                } else {
                    Log.e("RosApp", "Master description missing on intent on " + this.appMode + " mode");
                    throw new RosRuntimeException("Master description missing on intent on " + this.appMode + " mode");
                }
            }
            if (this.dashboard == null) {
                this.dashboard = new Dashboard(this);
                this.dashboard.setView((LinearLayout) findViewById(this.dashboardResourceId), new LinearLayout.LayoutParams(-2, -2));
            }
        }
    }

    /* access modifiers changed from: protected */
    public void init(NodeMainExecutor nodeMainExecutor2) {
        this.nodeMainExecutor = nodeMainExecutor2;
        this.nodeConfiguration = NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress(), getMasterUri());
        if (this.appMode == InteractionMode.STANDALONE) {
            this.dashboard.setRobotName(this.masterNameResolver.getMasterName());
        } else {
            this.masterNameResolver.setMaster(this.masterDescription);
            this.dashboard.setRobotName(this.masterDescription.getMasterName());
            if (this.appMode == InteractionMode.PAIRED) {
                this.dashboard.setRobotName(this.masterDescription.getMasterType());
            }
        }
        nodeMainExecutor2.execute(this.masterNameResolver, this.nodeConfiguration.setNodeName("masterNameResolver"));
        this.masterNameResolver.waitForResolver();
        nodeMainExecutor2.execute(this.dashboard, this.nodeConfiguration.setNodeName("dashboard"));
    }

    /* access modifiers changed from: protected */
    public NameResolver getMasterNameSpace() {
        return this.masterNameResolver.getMasterNameSpace();
    }

    /* access modifiers changed from: protected */
    public void onAppTerminate() {
        runOnUiThread(new Runnable() {
            public void run() {
                new AlertDialog.Builder(RosAppActivity.this).setTitle("App Termination").setMessage("The application has terminated on the server, so the client is exiting.").setCancelable(false).setNeutralButton("Exit", new DialogInterface.OnClickListener() {
                    public void onClick(DialogInterface dialog, int which) {
                        RosAppActivity.this.finish();
                    }
                }).create().show();
            }
        });
    }

    public void startMasterChooser() {
        if (this.appMode == InteractionMode.STANDALONE) {
            super.startMasterChooser();
            return;
        }
        try {
            this.nodeMainExecutorService.setMasterUri(new URI(this.masterDescription.getMasterUri()));
            new AsyncTask<Void, Void, Void>() {
                /* access modifiers changed from: protected */
                public Void doInBackground(Void... params) {
                    RosAppActivity.this.init(RosAppActivity.this.nodeMainExecutorService);
                    return null;
                }
            }.execute(new Void[0]);
        } catch (URISyntaxException e) {
            throw new RosRuntimeException((Throwable) e);
        }
    }

    /* access modifiers changed from: protected */
    public void releaseMasterNameResolver() {
        this.nodeMainExecutor.shutdownNodeMain(this.masterNameResolver);
    }

    /* access modifiers changed from: protected */
    public void releaseDashboardNode() {
        this.nodeMainExecutor.shutdownNodeMain(this.dashboard);
    }

    private boolean managePairedRobotApplication() {
        return this.appMode == InteractionMode.STANDALONE && this.masterAppName != null;
    }

    /* access modifiers changed from: protected */
    public void onDestroy() {
        super.onDestroy();
    }

    public void onBackPressed() {
        if (this.appMode != InteractionMode.STANDALONE) {
            Log.i("RosApp", "app terminating and returning control to the remocon.");
            Intent intent = new Intent();
            intent.putExtra("com.github.rosjava.android_remocons.common_tools.rocon.Constants." + this.appMode + "_app_name", "AppChooser");
            intent.putExtra(MasterDescription.UNIQUE_KEY, this.remoconExtraData);
            intent.setAction(this.remoconActivity);
            intent.addCategory("android.intent.category.DEFAULT");
            startActivity(intent);
            finish();
        } else {
            Log.i("RosApp", "backpress processing for RosAppActivity");
        }
        super.onBackPressed();
    }
}
