package org.ros.android;

import android.app.Activity;
import android.content.ComponentName;
import android.content.Intent;
import android.content.ServiceConnection;
import android.os.AsyncTask;
import android.os.IBinder;
import com.google.common.base.Preconditions;
import java.net.NetworkInterface;
import java.net.SocketException;
import java.net.URI;
import java.net.URISyntaxException;
import org.ros.EnvironmentVariables;
import org.ros.address.InetAddressFactory;
import org.ros.android.NodeMainExecutorService;
import org.ros.exception.RosRuntimeException;
import org.ros.node.NodeMainExecutor;

public abstract class RosActivity extends Activity {
    protected static final int MASTER_CHOOSER_REQUEST_CODE = 0;
    private Class<?> masterChooserActivity;
    private int masterChooserRequestCode;
    /* access modifiers changed from: protected */
    public NodeMainExecutorService nodeMainExecutorService;
    private final NodeMainExecutorServiceConnection nodeMainExecutorServiceConnection;
    private final String notificationTicker;
    private final String notificationTitle;
    private OnActivityResultCallback onActivityResultCallback;

    public interface OnActivityResultCallback {
        void execute(int i, int i2, Intent intent);
    }

    /* access modifiers changed from: protected */
    public abstract void init(NodeMainExecutor nodeMainExecutor);

    private final class NodeMainExecutorServiceConnection implements ServiceConnection {
        private URI customMasterUri;
        private NodeMainExecutorServiceListener serviceListener;

        public NodeMainExecutorServiceConnection(URI customUri) {
            this.customMasterUri = customUri;
        }

        public void onServiceConnected(ComponentName name, IBinder binder) {
            RosActivity.this.nodeMainExecutorService = ((NodeMainExecutorService.LocalBinder) binder).getService();
            if (this.customMasterUri != null) {
                RosActivity.this.nodeMainExecutorService.setMasterUri(this.customMasterUri);
                RosActivity.this.nodeMainExecutorService.setRosHostname(RosActivity.this.getDefaultHostAddress());
            }
            this.serviceListener = new NodeMainExecutorServiceListener() {
                public void onShutdown(NodeMainExecutorService nodeMainExecutorService) {
                    if (!RosActivity.this.isFinishing()) {
                        RosActivity.this.finish();
                    }
                }
            };
            RosActivity.this.nodeMainExecutorService.addListener(this.serviceListener);
            if (RosActivity.this.getMasterUri() == null) {
                RosActivity.this.startMasterChooser();
            } else {
                RosActivity.this.init();
            }
        }

        public void onServiceDisconnected(ComponentName name) {
            RosActivity.this.nodeMainExecutorService.removeListener(this.serviceListener);
            this.serviceListener = null;
        }

        public NodeMainExecutorServiceListener getServiceListener() {
            return this.serviceListener;
        }
    }

    protected RosActivity(String notificationTicker2, String notificationTitle2) {
        this(notificationTicker2, notificationTitle2, (URI) null);
    }

    protected RosActivity(String notificationTicker2, String notificationTitle2, URI customMasterUri) {
        this.masterChooserActivity = MasterChooser.class;
        this.masterChooserRequestCode = 0;
        this.onActivityResultCallback = new OnActivityResultCallback() {
            public void execute(int requestCode, int resultCode, Intent data) {
                String host;
                if (resultCode != -1) {
                    return;
                }
                if (requestCode == 0) {
                    String networkInterfaceName = data.getStringExtra("ROS_MASTER_NETWORK_INTERFACE");
                    if (networkInterfaceName == null || networkInterfaceName.equals("")) {
                        host = RosActivity.this.getDefaultHostAddress();
                    } else {
                        try {
                            host = InetAddressFactory.newNonLoopbackForNetworkInterface(NetworkInterface.getByName(networkInterfaceName)).getHostAddress();
                        } catch (SocketException e) {
                            throw new RosRuntimeException((Throwable) e);
                        }
                    }
                    RosActivity.this.nodeMainExecutorService.setRosHostname(host);
                    if (data.getBooleanExtra("ROS_MASTER_CREATE_NEW", false)) {
                        RosActivity.this.nodeMainExecutorService.startMaster(data.getBooleanExtra("ROS_MASTER_PRIVATE", true));
                    } else {
                        try {
                            RosActivity.this.nodeMainExecutorService.setMasterUri(new URI(data.getStringExtra(EnvironmentVariables.ROS_MASTER_URI)));
                        } catch (URISyntaxException e2) {
                            throw new RosRuntimeException((Throwable) e2);
                        }
                    }
                    new AsyncTask<Void, Void, Void>() {
                        /* access modifiers changed from: protected */
                        public Void doInBackground(Void... params) {
                            RosActivity.this.init(RosActivity.this.nodeMainExecutorService);
                            return null;
                        }
                    }.execute(new Void[0]);
                    return;
                }
                RosActivity.this.nodeMainExecutorService.forceShutdown();
            }
        };
        this.notificationTicker = notificationTicker2;
        this.notificationTitle = notificationTitle2;
        this.nodeMainExecutorServiceConnection = new NodeMainExecutorServiceConnection(customMasterUri);
    }

    protected RosActivity(String notificationTicker2, String notificationTitle2, Class<?> activity, int requestCode) {
        this(notificationTicker2, notificationTitle2);
        this.masterChooserActivity = activity;
        this.masterChooserRequestCode = requestCode;
    }

    /* access modifiers changed from: protected */
    public void onStart() {
        super.onStart();
        bindNodeMainExecutorService();
    }

    /* access modifiers changed from: protected */
    public void bindNodeMainExecutorService() {
        Intent intent = new Intent(this, NodeMainExecutorService.class);
        intent.setAction(NodeMainExecutorService.ACTION_START);
        intent.putExtra(NodeMainExecutorService.EXTRA_NOTIFICATION_TICKER, this.notificationTicker);
        intent.putExtra(NodeMainExecutorService.EXTRA_NOTIFICATION_TITLE, this.notificationTitle);
        startService(intent);
        Preconditions.checkState(bindService(intent, this.nodeMainExecutorServiceConnection, 1), "Failed to bind NodeMainExecutorService.");
    }

    /* access modifiers changed from: protected */
    public void onDestroy() {
        unbindService(this.nodeMainExecutorServiceConnection);
        this.nodeMainExecutorService.removeListener(this.nodeMainExecutorServiceConnection.getServiceListener());
        super.onDestroy();
    }

    /* access modifiers changed from: protected */
    public void init() {
        new AsyncTask<Void, Void, Void>() {
            /* access modifiers changed from: protected */
            public Void doInBackground(Void... params) {
                RosActivity.this.init(RosActivity.this.nodeMainExecutorService);
                return null;
            }
        }.execute(new Void[0]);
    }

    public void startMasterChooser() {
        Preconditions.checkState(getMasterUri() == null);
        super.startActivityForResult(new Intent(this, this.masterChooserActivity), this.masterChooserRequestCode);
    }

    public URI getMasterUri() {
        Preconditions.checkNotNull(this.nodeMainExecutorService);
        return this.nodeMainExecutorService.getMasterUri();
    }

    public String getRosHostname() {
        Preconditions.checkNotNull(this.nodeMainExecutorService);
        return this.nodeMainExecutorService.getRosHostname();
    }

    public void startActivityForResult(Intent intent, int requestCode) {
        Preconditions.checkArgument(requestCode != this.masterChooserRequestCode);
        super.startActivityForResult(intent, requestCode);
    }

    /* access modifiers changed from: protected */
    public void onActivityResult(int requestCode, int resultCode, Intent data) {
        super.onActivityResult(requestCode, resultCode, data);
        if (this.onActivityResultCallback != null) {
            this.onActivityResultCallback.execute(requestCode, resultCode, data);
        }
    }

    /* access modifiers changed from: private */
    public String getDefaultHostAddress() {
        return InetAddressFactory.newNonLoopback().getHostAddress();
    }

    public void setOnActivityResultCallback(OnActivityResultCallback callback) {
        this.onActivityResultCallback = callback;
    }
}
