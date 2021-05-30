package org.apache.commons.net.pop3;

public final class POP3Command {
    public static final int APOP = 9;
    public static final int DELE = 6;
    public static final int LIST = 4;
    public static final int NOOP = 7;
    public static final int PASS = 1;
    public static final int QUIT = 2;
    public static final int RETR = 5;
    public static final int RSET = 8;
    public static final int STAT = 3;
    public static final int TOP = 10;
    public static final int UIDL = 11;
    public static final int USER = 0;
    static final String[] _commands = {"USER", "PASS", "QUIT", "STAT", "LIST", "RETR", "DELE", "NOOP", "RSET", "APOP", "TOP", "UIDL"};

    private POP3Command() {
    }

    public static final String getCommand(int command) {
        return _commands[command];
    }
}
