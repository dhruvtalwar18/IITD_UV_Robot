package org.ros.android;

import android.app.AlertDialog;
import android.app.Notification;
import android.app.NotificationChannel;
import android.app.NotificationManager;
import android.app.PendingIntent;
import android.app.Service;
import android.content.DialogInterface;
import android.content.Intent;
import android.net.wifi.WifiManager;
import android.os.AsyncTask;
import android.os.Binder;
import android.os.Build;
import android.os.Handler;
import android.os.IBinder;
import android.os.PowerManager;
import android.util.Log;
import android.widget.Toast;
import com.google.common.base.Preconditions;
import java.net.URI;
import java.util.Collection;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ScheduledExecutorService;
import org.ros.RosCore;
import org.ros.android.android_core_components.R;
import org.ros.concurrent.ListenerGroup;
import org.ros.concurrent.SignalRunnable;
import org.ros.exception.RosRuntimeException;
import org.ros.node.DefaultNodeMainExecutor;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeListener;
import org.ros.node.NodeMain;
import org.ros.node.NodeMainExecutor;

public class NodeMainExecutorService extends Service implements NodeMainExecutor {
    static final /* synthetic */ boolean $assertionsDisabled = false;
    public static final String ACTION_SHUTDOWN = "org.ros.android.ACTION_SHUTDOWN_NODE_RUNNER_SERVICE";
    public static final String ACTION_START = "org.ros.android.ACTION_START_NODE_RUNNER_SERVICE";
    public static final String CHANNEL_NAME = "ROS Android background service";
    public static final String EXTRA_NOTIFICATION_TICKER = "org.ros.android.EXTRA_NOTIFICATION_TICKER";
    public static final String EXTRA_NOTIFICATION_TITLE = "org.ros.android.EXTRA_NOTIFICATION_TITLE";
    public static final String NOTIFICATION_CHANNEL_ID = "org.ros.android";
    private static final int ONGOING_NOTIFICATION = 1;
    private static final String TAG = "NodeMainExecutorService";
    private final IBinder binder = new LocalBinder();
    private Handler handler;
    private final ListenerGroup<NodeMainExecutorServiceListener> listeners = new ListenerGroup<>(this.nodeMainExecutor.getScheduledExecutorService());
    private URI masterUri;
    private final NodeMainExecutor nodeMainExecutor = DefaultNodeMainExecutor.newDefault();
    private RosCore rosCore;
    private String rosHostname = null;
    private PowerManager.WakeLock wakeLock;
    private WifiManager.WifiLock wifiLock;

    public class LocalBinder extends Binder {
        public LocalBinder() {
        }

        public NodeMainExecutorService getService() {
            return NodeMainExecutorService.this;
        }
    }

    public void onCreate() {
        this.handler = new Handler();
        this.wakeLock = ((PowerManager) getSystemService("power")).newWakeLock(1, TAG);
        this.wakeLock.acquire();
        int wifiLockType = 1;
        try {
            wifiLockType = WifiManager.class.getField("WIFI_MODE_FULL_HIGH_PERF").getInt((Object) null);
        } catch (Exception e) {
            Log.w(TAG, "Unable to acquire high performance wifi lock.");
        }
        this.wifiLock = WifiManager.class.cast(getApplicationContext().getSystemService("wifi")).createWifiLock(wifiLockType, TAG);
        this.wifiLock.acquire();
    }

    public void execute(NodeMain nodeMain, NodeConfiguration nodeConfiguration, Collection<NodeListener> nodeListeneners) {
        this.nodeMainExecutor.execute(nodeMain, nodeConfiguration, nodeListeneners);
    }

    public void execute(NodeMain nodeMain, NodeConfiguration nodeConfiguration) {
        execute(nodeMain, nodeConfiguration, (Collection<NodeListener>) null);
    }

    public ScheduledExecutorService getScheduledExecutorService() {
        return this.nodeMainExecutor.getScheduledExecutorService();
    }

    public void shutdownNodeMain(NodeMain nodeMain) {
        this.nodeMainExecutor.shutdownNodeMain(nodeMain);
    }

    public void shutdown() {
        this.handler.post(new Runnable() {
            public void run() {
                AlertDialog.Builder builder = new AlertDialog.Builder(NodeMainExecutorService.this);
                builder.setMessage("Continue shutting down?");
                builder.setPositiveButton("Shutdown", new DialogInterface.OnClickListener() {
                    public void onClick(DialogInterface dialog, int which) {
                        NodeMainExecutorService.this.forceShutdown();
                    }
                });
                builder.setNegativeButton("Cancel", new DialogInterface.OnClickListener() {
                    public void onClick(DialogInterface dialog, int which) {
                    }
                });
                AlertDialog alertDialog = builder.create();
                alertDialog.getWindow().setType(2005);
                alertDialog.show();
            }
        });
    }

    public void forceShutdown() {
        signalOnShutdown();
        stopForeground(true);
        stopSelf();
    }

    public void addListener(NodeMainExecutorServiceListener listener) {
        this.listeners.add(listener);
    }

    public void removeListener(NodeMainExecutorServiceListener listener) {
        this.listeners.remove(listener);
    }

    private void signalOnShutdown() {
        this.listeners.signal(new SignalRunnable<NodeMainExecutorServiceListener>() {
            public void run(NodeMainExecutorServiceListener nodeMainExecutorServiceListener) {
                nodeMainExecutorServiceListener.onShutdown(NodeMainExecutorService.this);
            }
        });
    }

    public void onDestroy() {
        toast("Shutting down...");
        this.nodeMainExecutor.shutdown();
        if (this.rosCore != null) {
            this.rosCore.shutdown();
        }
        if (this.wakeLock.isHeld()) {
            this.wakeLock.release();
        }
        if (this.wifiLock.isHeld()) {
            this.wifiLock.release();
        }
        super.onDestroy();
    }

    public int onStartCommand(Intent intent, int flags, int startId) {
        if (intent.getAction() == null) {
            return 2;
        }
        if (intent.getAction().equals(ACTION_START)) {
            Preconditions.checkArgument(intent.hasExtra(EXTRA_NOTIFICATION_TICKER));
            Preconditions.checkArgument(intent.hasExtra(EXTRA_NOTIFICATION_TITLE));
            Intent notificationIntent = new Intent(this, NodeMainExecutorService.class);
            notificationIntent.setAction(ACTION_SHUTDOWN);
            startForeground(1, buildNotification(intent, PendingIntent.getService(this, 0, notificationIntent, 0)));
        }
        if (intent.getAction().equals(ACTION_SHUTDOWN)) {
            shutdown();
        }
        return 2;
    }

    public IBinder onBind(Intent intent) {
        return this.binder;
    }

    public URI getMasterUri() {
        return this.masterUri;
    }

    public void setMasterUri(URI uri) {
        this.masterUri = uri;
    }

    public void setRosHostname(String hostname) {
        this.rosHostname = hostname;
    }

    public String getRosHostname() {
        return this.rosHostname;
    }

    @Deprecated
    public void startMaster() {
        startMaster(true);
    }

    public void startMaster(boolean isPrivate) {
        AsyncTask<Boolean, Void, URI> task = new AsyncTask<Boolean, Void, URI>() {
            /* access modifiers changed from: protected */
            public URI doInBackground(Boolean[] params) {
                NodeMainExecutorService.this.startMasterBlocking(params[0].booleanValue());
                return NodeMainExecutorService.this.getMasterUri();
            }
        };
        task.execute(new Boolean[]{Boolean.valueOf(isPrivate)});
        try {
            task.get();
        } catch (InterruptedException e) {
            throw new RosRuntimeException((Throwable) e);
        } catch (ExecutionException e2) {
            throw new RosRuntimeException((Throwable) e2);
        }
    }

    /* access modifiers changed from: private */
    public void startMasterBlocking(boolean isPrivate) {
        if (isPrivate) {
            this.rosCore = RosCore.newPrivate();
        } else if (this.rosHostname != null) {
            this.rosCore = RosCore.newPublic(this.rosHostname, 11311);
        } else {
            this.rosCore = RosCore.newPublic(11311);
        }
        this.rosCore.start();
        try {
            this.rosCore.awaitStart();
            this.masterUri = this.rosCore.getUri();
        } catch (Exception e) {
            throw new RosRuntimeException((Throwable) e);
        }
    }

    public void toast(final String text) {
        this.handler.post(new Runnable() {
            public void run() {
                Toast.makeText(NodeMainExecutorService.this, text, 0).show();
            }
        });
    }

    private Notification buildNotification(Intent intent, PendingIntent pendingIntent) {
        Notification.Builder builder;
        if (Build.VERSION.SDK_INT > 26) {
            NotificationChannel chan = new NotificationChannel(NOTIFICATION_CHANNEL_ID, CHANNEL_NAME, 0);
            chan.setLightColor(-16776961);
            chan.setLockscreenVisibility(0);
            ((NotificationManager) getSystemService("notification")).createNotificationChannel(chan);
            builder = new Notification.Builder(this, NOTIFICATION_CHANNEL_ID);
        } else {
            builder = new Notification.Builder(this);
        }
        return builder.setContentIntent(pendingIntent).setOngoing(true).setSmallIcon(R.mipmap.icon).setTicker(intent.getStringExtra(EXTRA_NOTIFICATION_TICKER)).setWhen(System.currentTimeMillis()).setContentTitle(intent.getStringExtra(EXTRA_NOTIFICATION_TITLE)).setAutoCancel(true).setContentText("Tap to shutdown.").build();
    }
}
