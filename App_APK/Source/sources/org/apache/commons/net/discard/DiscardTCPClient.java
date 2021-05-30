package org.apache.commons.net.discard;

import java.io.OutputStream;
import org.apache.commons.net.SocketClient;

public class DiscardTCPClient extends SocketClient {
    public static final int DEFAULT_PORT = 9;

    public DiscardTCPClient() {
        setDefaultPort(9);
    }

    public OutputStream getOutputStream() {
        return this._output_;
    }
}
