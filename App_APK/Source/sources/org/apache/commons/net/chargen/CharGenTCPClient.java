package org.apache.commons.net.chargen;

import java.io.InputStream;
import org.apache.commons.net.SocketClient;

public final class CharGenTCPClient extends SocketClient {
    public static final int CHARGEN_PORT = 19;
    public static final int DEFAULT_PORT = 19;
    public static final int NETSTAT_PORT = 15;
    public static final int QUOTE_OF_DAY_PORT = 17;
    public static final int SYSTAT_PORT = 11;

    public CharGenTCPClient() {
        setDefaultPort(19);
    }

    public InputStream getInputStream() {
        return this._input_;
    }
}
