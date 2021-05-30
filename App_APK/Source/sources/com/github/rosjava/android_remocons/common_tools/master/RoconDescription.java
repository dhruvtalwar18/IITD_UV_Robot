package com.github.rosjava.android_remocons.common_tools.master;

import java.io.Serializable;
import java.util.Date;
import java.util.List;
import rocon_std_msgs.Icon;

public class RoconDescription extends MasterDescription implements Serializable {
    private static final long serialVersionUID = -4705526306056241179L;
    private int currentRole = -1;
    private String description;
    private String interactionsNamespace;
    private String[] userRoles;

    public static RoconDescription create(MasterDescription master) {
        RoconDescription cd = new RoconDescription(master.getMasterId(), master.getMasterName(), (String) null, (Icon) null, (String) null, new Date());
        cd.setMasterIconFormat(master.getMasterIconFormat());
        cd.setMasterIconData(master.getMasterIconData());
        return cd;
    }

    public static RoconDescription createUnknown(MasterId masterId) {
        return new RoconDescription(masterId, "Unknown", (String) null, (Icon) null, (String) null, new Date());
    }

    public RoconDescription() {
    }

    public RoconDescription(MasterId masterId, String concertName, String description2, Icon concertIcon, String interactionsNamespace2, Date timeLastSeen) {
        super(masterId, concertName, "Rocon concert", concertIcon, "", timeLastSeen);
        this.description = description2;
        this.interactionsNamespace = interactionsNamespace2;
    }

    public void copyFrom(RoconDescription other) {
        super.copyFrom(other);
        this.userRoles = (String[]) other.userRoles.clone();
        this.description = other.description;
        this.interactionsNamespace = other.interactionsNamespace;
    }

    public String getInteractionsNamespace() {
        return this.interactionsNamespace;
    }

    public String[] getUserRoles() {
        return this.userRoles;
    }

    public String getCurrentRole() {
        if (this.userRoles == null || this.currentRole < 0 || this.currentRole >= this.userRoles.length) {
            return null;
        }
        return this.userRoles[this.currentRole];
    }

    public void setInteractionsNamespace(String namespace) {
        this.interactionsNamespace = namespace;
    }

    public void setUserRoles(List<String> roles) {
        this.userRoles = new String[roles.size()];
        roles.toArray(this.userRoles);
    }

    public void setCurrentRole(int role) {
        this.currentRole = role;
    }
}
