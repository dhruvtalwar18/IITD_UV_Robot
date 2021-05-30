package org.apache.commons.net.telnet;

public abstract class TelnetOptionHandler {
    private boolean acceptLocal = false;
    private boolean acceptRemote = false;
    private boolean doFlag = false;
    private boolean initialLocal = false;
    private boolean initialRemote = false;
    private int optionCode = -1;
    private boolean willFlag = false;

    public abstract int[] answerSubnegotiation(int[] iArr, int i);

    public abstract int[] startSubnegotiationLocal();

    public abstract int[] startSubnegotiationRemote();

    public TelnetOptionHandler(int optcode, boolean initlocal, boolean initremote, boolean acceptlocal, boolean acceptremote) {
        this.optionCode = optcode;
        this.initialLocal = initlocal;
        this.initialRemote = initremote;
        this.acceptLocal = acceptlocal;
        this.acceptRemote = acceptremote;
    }

    public int getOptionCode() {
        return this.optionCode;
    }

    public boolean getAcceptLocal() {
        return this.acceptLocal;
    }

    public boolean getAcceptRemote() {
        return this.acceptRemote;
    }

    public void setAcceptLocal(boolean accept) {
        this.acceptLocal = accept;
    }

    public void setAcceptRemote(boolean accept) {
        this.acceptRemote = accept;
    }

    public boolean getInitLocal() {
        return this.initialLocal;
    }

    public boolean getInitRemote() {
        return this.initialRemote;
    }

    public void setInitLocal(boolean init) {
        this.initialLocal = init;
    }

    public void setInitRemote(boolean init) {
        this.initialRemote = init;
    }

    /* access modifiers changed from: package-private */
    public boolean getWill() {
        return this.willFlag;
    }

    /* access modifiers changed from: package-private */
    public void setWill(boolean state) {
        this.willFlag = state;
    }

    /* access modifiers changed from: package-private */
    public boolean getDo() {
        return this.doFlag;
    }

    /* access modifiers changed from: package-private */
    public void setDo(boolean state) {
        this.doFlag = state;
    }
}
