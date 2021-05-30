package org.apache.commons.net.telnet;

public class TerminalTypeOptionHandler extends TelnetOptionHandler {
    protected static final int TERMINAL_TYPE = 24;
    protected static final int TERMINAL_TYPE_IS = 0;
    protected static final int TERMINAL_TYPE_SEND = 1;
    private String termType = null;

    public TerminalTypeOptionHandler(String termtype, boolean initlocal, boolean initremote, boolean acceptlocal, boolean acceptremote) {
        super(24, initlocal, initremote, acceptlocal, acceptremote);
        this.termType = termtype;
    }

    public TerminalTypeOptionHandler(String termtype) {
        super(24, false, false, false, false);
        this.termType = termtype;
    }

    public int[] answerSubnegotiation(int[] suboptionData, int suboptionLength) {
        if (suboptionData == null || suboptionLength <= 1 || this.termType == null) {
            return null;
        }
        int ii = 0;
        if (suboptionData[0] != 24 || suboptionData[1] != 1) {
            return null;
        }
        int[] response = new int[(this.termType.length() + 2)];
        response[0] = 24;
        response[1] = 0;
        while (true) {
            int ii2 = ii;
            if (ii2 >= this.termType.length()) {
                return response;
            }
            response[ii2 + 2] = this.termType.charAt(ii2);
            ii = ii2 + 1;
        }
    }

    public int[] startSubnegotiationLocal() {
        return null;
    }

    public int[] startSubnegotiationRemote() {
        return null;
    }
}
