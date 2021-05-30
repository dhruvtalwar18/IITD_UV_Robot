package com.github.rosjava.android_remocons.common_tools.master;

import java.io.Serializable;
import java.util.Date;
import java.util.regex.Pattern;
import org.jboss.netty.buffer.ChannelBuffer;
import org.jboss.netty.buffer.ChannelBuffers;
import rocon_std_msgs.Icon;

public class MasterDescription implements Serializable {
    public static final String CONNECTING = "connecting...";
    public static final String CONTROL = "not started";
    public static final String ERROR = "exception";
    public static final String NAME_UNKNOWN = "Unknown";
    public static final String OK = "ok";
    public static final String TYPE_UNKNOWN = "Unknown";
    public static final String UNAVAILABLE = "unavailable";
    public static final String UNIQUE_KEY = "com.github.rosjava.android_remocons.master.MasterDescription";
    public static final String WIFI = "invalid wifi";
    private static final long serialVersionUID = 1;
    private String appsNameSpace;
    private String connectionStatus;
    private byte[] masterIconData;
    private int masterIconDataLength;
    private int masterIconDataOffset;
    private String masterIconFormat;
    private MasterId masterId;
    private String masterName;
    private String masterType;
    private Date timeLastSeen;

    public MasterDescription() {
    }

    public MasterDescription(MasterId masterId2, String masterName2, String masterType2, Icon masterIcon, String appsNameSpace2, Date timeLastSeen2) {
        setMasterName(masterName2);
        setMasterId(masterId2);
        this.masterName = masterName2;
        this.masterType = masterType2;
        this.appsNameSpace = appsNameSpace2;
        if (masterIcon != null) {
            this.masterIconFormat = masterIcon.getFormat();
            this.masterIconData = masterIcon.getData().array();
            this.masterIconDataOffset = masterIcon.getData().arrayOffset();
            this.masterIconDataLength = masterIcon.getData().readableBytes();
        }
        this.timeLastSeen = timeLastSeen2;
    }

    public void copyFrom(MasterDescription other) {
        this.masterId = other.masterId;
        this.masterName = other.masterName;
        this.masterType = other.masterType;
        this.appsNameSpace = other.appsNameSpace;
        this.masterIconFormat = other.masterIconFormat;
        this.masterIconData = other.masterIconData;
        this.masterIconDataOffset = other.masterIconDataOffset;
        this.masterIconDataLength = other.masterIconDataLength;
        this.connectionStatus = other.connectionStatus;
        this.timeLastSeen = other.timeLastSeen;
    }

    public MasterId getMasterId() {
        return this.masterId;
    }

    public String getAppsNameSpace() {
        return this.appsNameSpace;
    }

    public String getMasterUri() {
        return this.masterId.getMasterUri();
    }

    public void setMasterId(MasterId masterId2) {
        this.masterId = masterId2;
    }

    public String getMasterName() {
        return this.masterName;
    }

    public String getMasterFriendlyName() {
        String friendlyName = this.masterName;
        if (this.masterName.length() > 32) {
            if (!Pattern.compile("[^a-f0-9]").matcher(this.masterName.substring(this.masterName.length() - 32)).find()) {
                friendlyName = this.masterName.substring(0, this.masterName.length() - 32);
            }
        }
        String friendlyName2 = friendlyName.replace('_', ' ');
        StringBuilder result = new StringBuilder(friendlyName2.length());
        String[] words = friendlyName2.split("\\s");
        int l = words.length;
        for (int i = 0; i < l; i++) {
            if (i > 0) {
                result.append(" ");
            }
            result.append(Character.toUpperCase(words[i].charAt(0)));
            result.append(words[i].substring(1));
        }
        return result.toString();
    }

    public void setMasterName(String masterName2) {
        this.masterName = masterName2;
    }

    public String getMasterType() {
        return this.masterType;
    }

    public void setMasterType(String masterType2) {
        this.masterType = masterType2;
    }

    public String getMasterIconFormat() {
        return this.masterIconFormat;
    }

    public ChannelBuffer getMasterIconData() {
        if (this.masterIconData == null) {
            return null;
        }
        return ChannelBuffers.copiedBuffer(this.masterIconData, this.masterIconDataOffset, this.masterIconDataLength);
    }

    public void setMasterIconFormat(String iconFormat) {
        this.masterIconFormat = iconFormat;
    }

    public void setMasterIconData(ChannelBuffer iconData) {
        this.masterIconData = iconData.array();
    }

    public void setMasterIcon(Icon masterIcon) {
        this.masterIconFormat = masterIcon.getFormat();
        this.masterIconData = masterIcon.getData().array();
    }

    public String getConnectionStatus() {
        return this.connectionStatus;
    }

    public void setConnectionStatus(String connectionStatus2) {
        this.connectionStatus = connectionStatus2;
    }

    public Date getTimeLastSeen() {
        return this.timeLastSeen;
    }

    public void setTimeLastSeen(Date timeLastSeen2) {
        this.timeLastSeen = timeLastSeen2;
    }

    public boolean isUnknown() {
        return this.masterName.equals("Unknown");
    }

    public boolean equals(Object o) {
        if (this == o) {
            return true;
        }
        if (!(o instanceof MasterDescription)) {
            return false;
        }
        MasterDescription lhs = (MasterDescription) o;
        if (this.masterId != null) {
            return this.masterId.equals(lhs.masterId);
        }
        if (lhs.masterId == null) {
            return true;
        }
        return false;
    }

    public int hashCode() {
        return (17 * 31) + (this.masterId == null ? 0 : this.masterId.hashCode());
    }
}
