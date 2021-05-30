package org.apache.commons.net;

import java.io.PrintWriter;

public class PrintCommandListener implements ProtocolCommandListener {
    private PrintWriter __writer;

    public PrintCommandListener(PrintWriter writer) {
        this.__writer = writer;
    }

    public void protocolCommandSent(ProtocolCommandEvent event) {
        this.__writer.print(event.getMessage());
        this.__writer.flush();
    }

    public void protocolReplyReceived(ProtocolCommandEvent event) {
        this.__writer.print(event.getMessage());
        this.__writer.flush();
    }
}
