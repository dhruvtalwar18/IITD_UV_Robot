package org.apache.commons.net.pop3;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.EOFException;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import java.util.Enumeration;
import java.util.Vector;
import org.apache.commons.net.MalformedServerReplyException;
import org.apache.commons.net.ProtocolCommandListener;
import org.apache.commons.net.ProtocolCommandSupport;
import org.apache.commons.net.SocketClient;

public class POP3 extends SocketClient {
    public static final int AUTHORIZATION_STATE = 0;
    public static final int DEFAULT_PORT = 110;
    public static final int DISCONNECTED_STATE = -1;
    public static final int TRANSACTION_STATE = 1;
    public static final int UPDATE_STATE = 2;
    static final String _ERROR = "-ERR";
    static final String _OK = "+OK";
    private static final String __DEFAULT_ENCODING = "ISO-8859-1";
    private StringBuffer __commandBuffer = new StringBuffer();
    private int __popState = -1;
    private BufferedWriter __writer = null;
    protected ProtocolCommandSupport _commandSupport_ = new ProtocolCommandSupport(this);
    String _lastReplyLine;
    BufferedReader _reader = null;
    int _replyCode;
    Vector<String> _replyLines = new Vector<>();

    public POP3() {
        setDefaultPort(110);
    }

    private void __getReply() throws IOException {
        this._replyLines.setSize(0);
        String line = this._reader.readLine();
        if (line != null) {
            if (line.startsWith(_OK)) {
                this._replyCode = 0;
            } else if (line.startsWith(_ERROR)) {
                this._replyCode = 1;
            } else {
                throw new MalformedServerReplyException("Received invalid POP3 protocol response from server.");
            }
            this._replyLines.addElement(line);
            this._lastReplyLine = line;
            if (this._commandSupport_.getListenerCount() > 0) {
                this._commandSupport_.fireReplyReceived(this._replyCode, getReplyString());
                return;
            }
            return;
        }
        throw new EOFException("Connection closed without indication.");
    }

    /* access modifiers changed from: protected */
    public void _connectAction_() throws IOException {
        super._connectAction_();
        this._reader = new BufferedReader(new InputStreamReader(this._input_, "ISO-8859-1"));
        this.__writer = new BufferedWriter(new OutputStreamWriter(this._output_, "ISO-8859-1"));
        __getReply();
        setState(0);
    }

    public void addProtocolCommandListener(ProtocolCommandListener listener) {
        this._commandSupport_.addProtocolCommandListener(listener);
    }

    public void removeProtocolCommandistener(ProtocolCommandListener listener) {
        this._commandSupport_.removeProtocolCommandListener(listener);
    }

    public void setState(int state) {
        this.__popState = state;
    }

    public int getState() {
        return this.__popState;
    }

    public void getAdditionalReply() throws IOException {
        String line = this._reader.readLine();
        while (line != null) {
            this._replyLines.addElement(line);
            if (!line.equals(".")) {
                line = this._reader.readLine();
            } else {
                return;
            }
        }
    }

    public void disconnect() throws IOException {
        super.disconnect();
        this._reader = null;
        this.__writer = null;
        this._lastReplyLine = null;
        this._replyLines.setSize(0);
        setState(-1);
    }

    public int sendCommand(String command, String args) throws IOException {
        this.__commandBuffer.setLength(0);
        this.__commandBuffer.append(command);
        if (args != null) {
            this.__commandBuffer.append(' ');
            this.__commandBuffer.append(args);
        }
        this.__commandBuffer.append("\r\n");
        BufferedWriter bufferedWriter = this.__writer;
        String stringBuffer = this.__commandBuffer.toString();
        String message = stringBuffer;
        bufferedWriter.write(stringBuffer);
        this.__writer.flush();
        if (this._commandSupport_.getListenerCount() > 0) {
            this._commandSupport_.fireCommandSent(command, message);
        }
        __getReply();
        return this._replyCode;
    }

    public int sendCommand(String command) throws IOException {
        return sendCommand(command, (String) null);
    }

    public int sendCommand(int command, String args) throws IOException {
        return sendCommand(POP3Command._commands[command], args);
    }

    public int sendCommand(int command) throws IOException {
        return sendCommand(POP3Command._commands[command], (String) null);
    }

    public String[] getReplyStrings() {
        String[] lines = new String[this._replyLines.size()];
        this._replyLines.copyInto(lines);
        return lines;
    }

    public String getReplyString() {
        StringBuffer buffer = new StringBuffer(256);
        Enumeration<String> en = this._replyLines.elements();
        while (en.hasMoreElements()) {
            buffer.append(en.nextElement());
            buffer.append("\r\n");
        }
        return buffer.toString();
    }
}
