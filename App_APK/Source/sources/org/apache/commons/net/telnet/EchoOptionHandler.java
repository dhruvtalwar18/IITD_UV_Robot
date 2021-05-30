package org.apache.commons.net.telnet;

public class EchoOptionHandler extends TelnetOptionHandler {
    public EchoOptionHandler(boolean initlocal, boolean initremote, boolean acceptlocal, boolean acceptremote) {
        super(1, initlocal, initremote, acceptlocal, acceptremote);
    }

    public EchoOptionHandler() {
        super(1, false, false, false, false);
    }

    public int[] answerSubnegotiation(int[] suboptionData, int suboptionLength) {
        return null;
    }

    public int[] startSubnegotiationLocal() {
        return null;
    }

    public int[] startSubnegotiationRemote() {
        return null;
    }
}
