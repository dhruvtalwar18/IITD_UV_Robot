package com.github.rosjava.android_remocons.common_tools.zeroconf;

import android.util.Log;
import com.github.rosjava.zeroconf_jmdns_suite.jmdns.ZeroconfLogger;

public class Logger implements ZeroconfLogger {
    public void println(String msg) {
        Log.i("zeroconf", msg);
    }
}
