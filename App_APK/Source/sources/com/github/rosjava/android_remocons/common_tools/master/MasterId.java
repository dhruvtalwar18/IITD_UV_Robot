package com.github.rosjava.android_remocons.common_tools.master;

import java.io.Serializable;
import java.util.Map;

public class MasterId implements Serializable {
    private static final long serialVersionUID = -1185642483404745956L;
    private String masterUri;
    private String wifi;
    private String wifiEncryption;
    private String wifiPassword;

    public MasterId() {
    }

    public MasterId(String masterUri2, String wifi2, String wifiEncryption2, String wifiPassword2) {
        this.masterUri = masterUri2;
        this.wifi = wifi2;
        this.wifiEncryption = wifiEncryption2;
        this.wifiPassword = wifiPassword2;
    }

    public MasterId(Map<String, Object> map) {
        if (map.containsKey("URL")) {
            this.masterUri = map.get("URL").toString();
        }
        if (map.containsKey("WIFI")) {
            this.wifi = map.get("WIFI").toString();
        }
        if (map.containsKey("WIFIENC")) {
            this.wifiEncryption = map.get("WIFIENC").toString();
        }
        if (map.containsKey("WIFIPW")) {
            this.wifiPassword = map.get("WIFIPW").toString();
        }
    }

    public MasterId(String masterUri2) {
        this.masterUri = masterUri2;
    }

    public String getMasterUri() {
        return this.masterUri;
    }

    public String getWifi() {
        return this.wifi;
    }

    public String getWifiEncryption() {
        return this.wifiEncryption;
    }

    public String getWifiPassword() {
        return this.wifiPassword;
    }

    public String toString() {
        String str = getMasterUri() == null ? "" : getMasterUri();
        if (getWifi() == null) {
            return str;
        }
        return str + " On Wifi: " + getWifi();
    }

    private boolean nullSafeEquals(Object a, Object b) {
        if (a == b) {
            return true;
        }
        if (a == null || b == null) {
            return false;
        }
        return a.equals(b);
    }

    public boolean equals(Object o) {
        if (this == o) {
            return true;
        }
        if (!(o instanceof MasterId)) {
            return false;
        }
        MasterId lhs = (MasterId) o;
        if (!nullSafeEquals(this.masterUri, lhs.masterUri) || !nullSafeEquals(this.wifi, lhs.wifi) || !nullSafeEquals(this.wifiEncryption, lhs.wifiEncryption) || !nullSafeEquals(this.wifiPassword, lhs.wifiPassword)) {
            return false;
        }
        return true;
    }

    public int hashCode() {
        int i = 0;
        int result = ((((((17 * 31) + (this.masterUri == null ? 0 : this.masterUri.hashCode())) * 31) + (this.wifi == null ? 0 : this.wifi.hashCode())) * 31) + (this.wifiEncryption == null ? 0 : this.wifiEncryption.hashCode())) * 31;
        if (this.wifiPassword != null) {
            i = this.wifiPassword.hashCode();
        }
        return result + i;
    }
}
