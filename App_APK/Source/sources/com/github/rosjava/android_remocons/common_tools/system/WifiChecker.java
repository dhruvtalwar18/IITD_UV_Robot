package com.github.rosjava.android_remocons.common_tools.system;

import android.net.wifi.ScanResult;
import android.net.wifi.SupplicantState;
import android.net.wifi.WifiConfiguration;
import android.net.wifi.WifiInfo;
import android.net.wifi.WifiManager;
import android.util.Log;
import com.github.rosjava.android_remocons.common_tools.master.MasterId;
import java.lang.Thread;
import java.util.Iterator;
import java.util.List;

public class WifiChecker {
    private CheckerThread checkerThread;
    /* access modifiers changed from: private */
    public FailureHandler failureCallback;
    /* access modifiers changed from: private */
    public SuccessHandler foundWiFiCallback;
    /* access modifiers changed from: private */
    public ReconnectionHandler reconnectionCallback;

    public interface FailureHandler {
        void handleFailure(String str);
    }

    public interface ReconnectionHandler {
        boolean doReconnection(String str, String str2);
    }

    public interface SuccessHandler {
        void handleSuccess();
    }

    public WifiChecker(SuccessHandler foundWiFiCallback2, FailureHandler failureCallback2, ReconnectionHandler reconnectionCallback2) {
        this.foundWiFiCallback = foundWiFiCallback2;
        this.failureCallback = failureCallback2;
        this.reconnectionCallback = reconnectionCallback2;
    }

    public void beginChecking(MasterId masterId, WifiManager manager) {
        stopChecking();
        if (masterId.getWifi() == null) {
            this.foundWiFiCallback.handleSuccess();
            return;
        }
        this.checkerThread = new CheckerThread(masterId, manager);
        this.checkerThread.start();
    }

    public void stopChecking() {
        if (this.checkerThread != null && this.checkerThread.isAlive()) {
            this.checkerThread.interrupt();
        }
    }

    public static boolean wifiValid(MasterId masterId, WifiManager wifiManager) {
        WifiInfo wifiInfo = wifiManager.getConnectionInfo();
        if (masterId.getWifi() == null) {
            return true;
        }
        if (!wifiManager.isWifiEnabled() || wifiInfo == null) {
            return false;
        }
        Log.d("WiFiChecker", "WiFi Info: " + wifiInfo.toString() + " IP " + wifiInfo.getIpAddress());
        if (wifiInfo.getSSID() == null || wifiInfo.getIpAddress() == 0 || wifiInfo.getSupplicantState() != SupplicantState.COMPLETED) {
            return false;
        }
        if (wifiInfo.getSSID().equals("\"" + masterId.getWifi() + "\"")) {
            return true;
        }
        return false;
    }

    public String getScanResultSecurity(ScanResult scanResult) {
        String cap = scanResult.capabilities;
        String[] securityModes = {"WEP", "PSK", "EAP"};
        for (int i = securityModes.length - 1; i >= 0; i--) {
            if (cap.contains(securityModes[i])) {
                return securityModes[i];
            }
        }
        return "OPEN";
    }

    private class CheckerThread extends Thread {
        private MasterId masterId;
        private WifiManager wifiManager;

        public CheckerThread(MasterId masterId2, WifiManager wifi) {
            this.masterId = masterId2;
            this.wifiManager = wifi;
            setDaemon(true);
            setUncaughtExceptionHandler(new Thread.UncaughtExceptionHandler(WifiChecker.this) {
                public void uncaughtException(Thread thread, Throwable ex) {
                    FailureHandler access$000 = WifiChecker.this.failureCallback;
                    access$000.handleFailure("exception: " + ex.getMessage());
                }
            });
        }

        private boolean wifiValid() {
            return WifiChecker.wifiValid(this.masterId, this.wifiManager);
        }

        public void run() {
            try {
                if (wifiValid()) {
                    WifiChecker.this.foundWiFiCallback.handleSuccess();
                } else if (WifiChecker.this.reconnectionCallback.doReconnection(this.wifiManager.getConnectionInfo().getSSID(), this.masterId.getWifi())) {
                    Log.d("WiFiChecker", "Wait for networking");
                    this.wifiManager.setWifiEnabled(true);
                    for (int i = 0; i < 30 && !this.wifiManager.isWifiEnabled(); i++) {
                        Log.d("WiFiChecker", "Waiting for WiFi enable");
                        Thread.sleep(1000);
                    }
                    if (!this.wifiManager.isWifiEnabled()) {
                        WifiChecker.this.failureCallback.handleFailure("Un-able to enable to WiFi");
                        return;
                    }
                    int n = -1;
                    int priority = -1;
                    WifiConfiguration wc = null;
                    String SSID = "\"" + this.masterId.getWifi() + "\"";
                    for (WifiConfiguration test : this.wifiManager.getConfiguredNetworks()) {
                        Log.d("WiFiChecker", "WIFI " + test.toString());
                        if (test.priority > priority) {
                            priority = test.priority;
                        }
                        if (test.SSID.equals(SSID)) {
                            n = test.networkId;
                            wc = test;
                        }
                    }
                    if (wc != null) {
                        if (wc.priority != priority) {
                            wc.priority = priority + 1;
                        }
                        wc.status = 1;
                        this.wifiManager.updateNetwork(wc);
                    }
                    if (n == -1) {
                        Log.d("WiFiChecker", "WIFI Unknown");
                        Log.d("WiFiChecker", "WIFI Scan Start");
                        if (this.wifiManager.startScan()) {
                            Log.d("WiFiChecker", "WIFI Scan Success");
                        } else {
                            Log.d("WiFiChecker", "WIFI Scan Failure");
                            WifiChecker.this.failureCallback.handleFailure("wifi scan fail");
                        }
                        List<ScanResult> scanResultList = this.wifiManager.getScanResults();
                        for (int i2 = 0; i2 < 30 && scanResultList.size() == 0; i2++) {
                            scanResultList = this.wifiManager.getScanResults();
                            Log.d("WiFiChecker", "Waiting for getting wifi list");
                            Thread.sleep(1000);
                        }
                        WifiConfiguration wc2 = new WifiConfiguration();
                        Iterator<ScanResult> it = scanResultList.iterator();
                        while (true) {
                            if (!it.hasNext()) {
                                break;
                            }
                            ScanResult result = it.next();
                            if (result.SSID.equals(this.masterId.getWifi())) {
                                String securityMode = WifiChecker.this.getScanResultSecurity(result);
                                Log.d("WiFiChecker", "WIFI mode: " + securityMode);
                                wc2.SSID = "\"" + this.masterId.getWifi() + "\"";
                                if (securityMode.equalsIgnoreCase("OPEN")) {
                                    wc2.allowedKeyManagement.set(0);
                                } else if (securityMode.equalsIgnoreCase("WEP")) {
                                    wc2.wepKeys[0] = "\"" + this.masterId.getWifiPassword() + "\"";
                                    wc2.wepTxKeyIndex = 0;
                                    wc2.allowedKeyManagement.set(0);
                                    wc2.allowedGroupCiphers.set(0);
                                } else {
                                    wc2.preSharedKey = "\"" + this.masterId.getWifiPassword() + "\"";
                                    wc2.hiddenSSID = true;
                                    wc2.status = 2;
                                    wc2.allowedGroupCiphers.set(2);
                                    wc2.allowedGroupCiphers.set(3);
                                    wc2.allowedKeyManagement.set(1);
                                    wc2.allowedPairwiseCiphers.set(1);
                                    wc2.allowedPairwiseCiphers.set(2);
                                    wc2.allowedProtocols.set(1);
                                    wc2.allowedProtocols.set(0);
                                }
                                n = this.wifiManager.addNetwork(wc2);
                            }
                        }
                    }
                    Log.d("WiFiChecker", "add Network returned " + n);
                    if (n == -1) {
                        WifiChecker.this.failureCallback.handleFailure("Failed to add the WiFi configure");
                    }
                    boolean b = this.wifiManager.enableNetwork(n, true);
                    Log.d("WiFiChecker", "enableNetwork returned " + b);
                    if (b) {
                        this.wifiManager.reconnect();
                        Log.d("WiFiChecker", "Wait for wifi network");
                        for (int i3 = 0; i3 < 3 && !wifiValid(); i3++) {
                            Log.d("WiFiChecker", "Waiting for network: " + i3 + " " + this.wifiManager.getWifiState());
                            Thread.sleep(3000);
                        }
                        if (wifiValid()) {
                            WifiChecker.this.foundWiFiCallback.handleSuccess();
                        } else {
                            WifiChecker.this.failureCallback.handleFailure("WiFi connection timed out");
                        }
                    }
                } else {
                    WifiChecker.this.failureCallback.handleFailure("Wrong WiFi network");
                }
            } catch (Throwable ex) {
                Log.e("RosAndroid", "Exception while searching for WiFi for " + this.masterId.getWifi(), ex);
                WifiChecker.this.failureCallback.handleFailure(ex.toString());
            }
        }
    }
}
