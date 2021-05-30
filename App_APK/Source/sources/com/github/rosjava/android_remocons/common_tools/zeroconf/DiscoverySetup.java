package com.github.rosjava.android_remocons.common_tools.zeroconf;

import android.app.ProgressDialog;
import android.content.Context;
import android.os.AsyncTask;
import android.util.Log;
import com.github.rosjava.zeroconf_jmdns_suite.jmdns.Zeroconf;

public class DiscoverySetup extends AsyncTask<Zeroconf, String, Void> {
    private ProgressDialog commencing_dialog;
    private final Context context;

    public DiscoverySetup(Context context2) {
        this.context = context2;
    }

    /* access modifiers changed from: protected */
    public Void doInBackground(Zeroconf... zeroconfs) {
        if (zeroconfs.length == 1) {
            Zeroconf zconf = zeroconfs[0];
            Log.i("zeroconf", "*********** Discovery Commencing **************");
            zconf.addListener("_ros-master._tcp", "local");
            zconf.addListener("_ros-master._udp", "local");
            zconf.addListener("_concert-master._tcp", "local");
            zconf.addListener("_concert-master._udp", "local");
            return null;
        }
        Log.i("zeroconf", "Error - DiscoveryTask::doInBackground received #zeroconfs != 1");
        return null;
    }
}
