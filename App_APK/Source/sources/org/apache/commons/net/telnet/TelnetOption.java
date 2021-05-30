package org.apache.commons.net.telnet;

public class TelnetOption {
    public static final int APPROXIMATE_MESSAGE_SIZE = 4;
    public static final int AUTHENTICATION = 37;
    public static final int BINARY = 0;
    public static final int BYTE_MACRO = 19;
    public static final int DATA_ENTRY_TERMINAL = 20;
    public static final int ECHO = 1;
    public static final int ENCRYPTION = 38;
    public static final int END_OF_RECORD = 25;
    public static final int EXTENDED_ASCII = 17;
    public static final int EXTENDED_OPTIONS_LIST = 255;
    public static final int FORCE_LOGOUT = 18;
    public static final int LINEMODE = 34;
    public static final int MAX_OPTION_VALUE = 255;
    public static final int NEGOTIATE_CARRIAGE_RETURN = 10;
    public static final int NEGOTIATE_FORMFEED = 13;
    public static final int NEGOTIATE_HORIZONTAL_TAB = 12;
    public static final int NEGOTIATE_HORIZONTAL_TAB_STOP = 11;
    public static final int NEGOTIATE_LINEFEED = 16;
    public static final int NEGOTIATE_OUTPUT_LINE_WIDTH = 8;
    public static final int NEGOTIATE_OUTPUT_PAGE_SIZE = 9;
    public static final int NEGOTIATE_VERTICAL_TAB = 15;
    public static final int NEGOTIATE_VERTICAL_TAB_STOP = 14;
    public static final int NEW_ENVIRONMENT_VARIABLES = 39;
    public static final int OLD_ENVIRONMENT_VARIABLES = 36;
    public static final int OUTPUT_MARKING = 27;
    public static final int PREPARE_TO_RECONNECT = 2;
    public static final int REGIME_3270 = 29;
    public static final int REMOTE_CONTROLLED_TRANSMISSION = 7;
    public static final int REMOTE_FLOW_CONTROL = 33;
    public static final int SEND_LOCATION = 23;
    public static final int STATUS = 5;
    public static final int SUPDUP = 21;
    public static final int SUPDUP_OUTPUT = 22;
    public static final int SUPPRESS_GO_AHEAD = 3;
    public static final int TACACS_USER_IDENTIFICATION = 26;
    public static final int TERMINAL_LOCATION_NUMBER = 28;
    public static final int TERMINAL_SPEED = 32;
    public static final int TERMINAL_TYPE = 24;
    public static final int TIMING_MARK = 6;
    public static final int WINDOW_SIZE = 31;
    public static final int X3_PAD = 30;
    public static final int X_DISPLAY_LOCATION = 35;
    private static final int __FIRST_OPTION = 0;
    private static final int __LAST_OPTION = 255;
    private static final String[] __optionString = {"BINARY", "ECHO", "RCP", "SUPPRESS GO AHEAD", "NAME", "STATUS", "TIMING MARK", "RCTE", "NAOL", "NAOP", "NAOCRD", "NAOHTS", "NAOHTD", "NAOFFD", "NAOVTS", "NAOVTD", "NAOLFD", "EXTEND ASCII", "LOGOUT", "BYTE MACRO", "DATA ENTRY TERMINAL", "SUPDUP", "SUPDUP OUTPUT", "SEND LOCATION", "TERMINAL TYPE", "END OF RECORD", "TACACS UID", "OUTPUT MARKING", "TTYLOC", "3270 REGIME", "X.3 PAD", "NAWS", "TSPEED", "LFLOW", "LINEMODE", "XDISPLOC", "OLD-ENVIRON", "AUTHENTICATION", "ENCRYPT", "NEW-ENVIRON", "TN3270E", "XAUTH", "CHARSET", "RSP", "Com Port Control", "Suppress Local Echo", "Start TLS", "KERMIT", "SEND-URL", "FORWARD_X", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "TELOPT PRAGMA LOGON", "TELOPT SSPI LOGON", "TELOPT PRAGMA HEARTBEAT", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "Extended-Options-List"};

    public static final String getOption(int code) {
        if (__optionString[code].length() == 0) {
            return "UNASSIGNED";
        }
        return __optionString[code];
    }

    public static final boolean isValidOption(int code) {
        return code <= 255;
    }

    private TelnetOption() {
    }
}
