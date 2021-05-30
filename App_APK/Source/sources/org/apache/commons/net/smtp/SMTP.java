package org.apache.commons.net.smtp;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Iterator;
import org.apache.commons.net.MalformedServerReplyException;
import org.apache.commons.net.ProtocolCommandListener;
import org.apache.commons.net.ProtocolCommandSupport;
import org.apache.commons.net.SocketClient;

public class SMTP extends SocketClient {
    public static final int DEFAULT_PORT = 25;
    private static final String __DEFAULT_ENCODING = "ISO-8859-1";
    private StringBuffer __commandBuffer;
    protected ProtocolCommandSupport _commandSupport_;
    boolean _newReplyString;
    BufferedReader _reader;
    int _replyCode;
    ArrayList<String> _replyLines;
    String _replyString;
    BufferedWriter _writer;
    private String encoding;

    public SMTP() {
        this.encoding = "ISO-8859-1";
        setDefaultPort(25);
        this.__commandBuffer = new StringBuffer();
        this._replyLines = new ArrayList<>();
        this._newReplyString = false;
        this._replyString = null;
        this._commandSupport_ = new ProtocolCommandSupport(this);
    }

    public SMTP(String encoding2) {
        this();
        this.encoding = encoding2;
    }

    private int __sendCommand(String command, String args, boolean includeSpace) throws IOException {
        this.__commandBuffer.setLength(0);
        this.__commandBuffer.append(command);
        if (args != null) {
            if (includeSpace) {
                this.__commandBuffer.append(' ');
            }
            this.__commandBuffer.append(args);
        }
        this.__commandBuffer.append("\r\n");
        BufferedWriter bufferedWriter = this._writer;
        String stringBuffer = this.__commandBuffer.toString();
        String message = stringBuffer;
        bufferedWriter.write(stringBuffer);
        this._writer.flush();
        if (this._commandSupport_.getListenerCount() > 0) {
            this._commandSupport_.fireCommandSent(command, message);
        }
        __getReply();
        return this._replyCode;
    }

    private int __sendCommand(int command, String args, boolean includeSpace) throws IOException {
        return __sendCommand(SMTPCommand._commands[command], args, includeSpace);
    }

    private void __getReply() throws IOException {
        this._newReplyString = true;
        this._replyLines.clear();
        String line = this._reader.readLine();
        if (line != null) {
            int length = line.length();
            if (length >= 3) {
                try {
                    this._replyCode = Integer.parseInt(line.substring(0, 3));
                    this._replyLines.add(line);
                    if (length > 3 && line.charAt(3) == '-') {
                        while (true) {
                            String line2 = this._reader.readLine();
                            if (line2 != null) {
                                this._replyLines.add(line2);
                                if (line2.length() >= 4 && line2.charAt(3) != '-' && Character.isDigit(line2.charAt(0))) {
                                    break;
                                }
                            } else {
                                throw new SMTPConnectionClosedException("Connection closed without indication.");
                            }
                        }
                    }
                    if (this._commandSupport_.getListenerCount() > 0) {
                        this._commandSupport_.fireReplyReceived(this._replyCode, getReplyString());
                    }
                    if (this._replyCode == 421) {
                        throw new SMTPConnectionClosedException("SMTP response 421 received.  Server closed connection.");
                    }
                } catch (NumberFormatException e) {
                    throw new MalformedServerReplyException("Could not parse response code.\nServer Reply: " + line);
                }
            } else {
                throw new MalformedServerReplyException("Truncated server reply: " + line);
            }
        } else {
            throw new SMTPConnectionClosedException("Connection closed without indication.");
        }
    }

    /* access modifiers changed from: protected */
    public void _connectAction_() throws IOException {
        super._connectAction_();
        this._reader = new BufferedReader(new InputStreamReader(this._input_, this.encoding));
        this._writer = new BufferedWriter(new OutputStreamWriter(this._output_, this.encoding));
        __getReply();
    }

    public void addProtocolCommandListener(ProtocolCommandListener listener) {
        this._commandSupport_.addProtocolCommandListener(listener);
    }

    public void removeProtocolCommandistener(ProtocolCommandListener listener) {
        this._commandSupport_.removeProtocolCommandListener(listener);
    }

    public void disconnect() throws IOException {
        super.disconnect();
        this._reader = null;
        this._writer = null;
        this._replyString = null;
        this._replyLines.clear();
        this._newReplyString = false;
    }

    public int sendCommand(String command, String args) throws IOException {
        return __sendCommand(command, args, true);
    }

    public int sendCommand(int command, String args) throws IOException {
        return sendCommand(SMTPCommand._commands[command], args);
    }

    public int sendCommand(String command) throws IOException {
        return sendCommand(command, (String) null);
    }

    public int sendCommand(int command) throws IOException {
        return sendCommand(command, (String) null);
    }

    public int getReplyCode() {
        return this._replyCode;
    }

    public int getReply() throws IOException {
        __getReply();
        return this._replyCode;
    }

    public String[] getReplyStrings() {
        String[] lines = new String[this._replyLines.size()];
        this._replyLines.addAll(Arrays.asList(lines));
        return lines;
    }

    public String getReplyString() {
        if (!this._newReplyString) {
            return this._replyString;
        }
        StringBuilder buffer = new StringBuilder();
        Iterator i$ = this._replyLines.iterator();
        while (i$.hasNext()) {
            buffer.append(i$.next());
            buffer.append("\r\n");
        }
        this._newReplyString = false;
        String sb = buffer.toString();
        this._replyString = sb;
        return sb;
    }

    public int helo(String hostname) throws IOException {
        return sendCommand(0, hostname);
    }

    public int mail(String reversePath) throws IOException {
        return __sendCommand(1, reversePath, false);
    }

    public int rcpt(String forwardPath) throws IOException {
        return __sendCommand(2, forwardPath, false);
    }

    public int data() throws IOException {
        return sendCommand(3);
    }

    public int send(String reversePath) throws IOException {
        return sendCommand(4, reversePath);
    }

    public int soml(String reversePath) throws IOException {
        return sendCommand(5, reversePath);
    }

    public int saml(String reversePath) throws IOException {
        return sendCommand(6, reversePath);
    }

    public int rset() throws IOException {
        return sendCommand(7);
    }

    public int vrfy(String user) throws IOException {
        return sendCommand(8, user);
    }

    public int expn(String name) throws IOException {
        return sendCommand(9, name);
    }

    public int help() throws IOException {
        return sendCommand(10);
    }

    public int help(String command) throws IOException {
        return sendCommand(10, command);
    }

    public int noop() throws IOException {
        return sendCommand(11);
    }

    public int turn() throws IOException {
        return sendCommand(12);
    }

    public int quit() throws IOException {
        return sendCommand(13);
    }
}
