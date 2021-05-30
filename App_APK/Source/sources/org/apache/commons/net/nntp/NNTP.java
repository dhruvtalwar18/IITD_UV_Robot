package org.apache.commons.net.nntp;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import org.apache.commons.net.MalformedServerReplyException;
import org.apache.commons.net.ProtocolCommandListener;
import org.apache.commons.net.ProtocolCommandSupport;
import org.apache.commons.net.SocketClient;

public class NNTP extends SocketClient {
    public static final int DEFAULT_PORT = 119;
    private static final String __DEFAULT_ENCODING = "ISO-8859-1";
    private StringBuffer __commandBuffer = new StringBuffer();
    protected ProtocolCommandSupport _commandSupport_ = new ProtocolCommandSupport(this);
    boolean _isAllowedToPost = false;
    protected BufferedReader _reader_ = null;
    int _replyCode;
    String _replyString = null;
    protected BufferedWriter _writer_ = null;

    public NNTP() {
        setDefaultPort(119);
    }

    private void __getReply() throws IOException {
        this._replyString = this._reader_.readLine();
        if (this._replyString == null) {
            throw new NNTPConnectionClosedException("Connection closed without indication.");
        } else if (this._replyString.length() >= 3) {
            try {
                this._replyCode = Integer.parseInt(this._replyString.substring(0, 3));
                if (this._commandSupport_.getListenerCount() > 0) {
                    ProtocolCommandSupport protocolCommandSupport = this._commandSupport_;
                    int i = this._replyCode;
                    protocolCommandSupport.fireReplyReceived(i, this._replyString + "\r\n");
                }
                if (this._replyCode == 400) {
                    throw new NNTPConnectionClosedException("NNTP response 400 received.  Server closed connection.");
                }
            } catch (NumberFormatException e) {
                throw new MalformedServerReplyException("Could not parse response code.\nServer Reply: " + this._replyString);
            }
        } else {
            throw new MalformedServerReplyException("Truncated server reply: " + this._replyString);
        }
    }

    /* access modifiers changed from: protected */
    public void _connectAction_() throws IOException {
        super._connectAction_();
        this._reader_ = new BufferedReader(new InputStreamReader(this._input_, "ISO-8859-1"));
        this._writer_ = new BufferedWriter(new OutputStreamWriter(this._output_, "ISO-8859-1"));
        __getReply();
        this._isAllowedToPost = this._replyCode == 200;
    }

    public void addProtocolCommandListener(ProtocolCommandListener listener) {
        this._commandSupport_.addProtocolCommandListener(listener);
    }

    public void removeProtocolCommandListener(ProtocolCommandListener listener) {
        this._commandSupport_.removeProtocolCommandListener(listener);
    }

    public void disconnect() throws IOException {
        super.disconnect();
        this._reader_ = null;
        this._writer_ = null;
        this._replyString = null;
        this._isAllowedToPost = false;
    }

    public boolean isAllowedToPost() {
        return this._isAllowedToPost;
    }

    public int sendCommand(String command, String args) throws IOException {
        this.__commandBuffer.setLength(0);
        this.__commandBuffer.append(command);
        if (args != null) {
            this.__commandBuffer.append(' ');
            this.__commandBuffer.append(args);
        }
        this.__commandBuffer.append("\r\n");
        BufferedWriter bufferedWriter = this._writer_;
        String stringBuffer = this.__commandBuffer.toString();
        String message = stringBuffer;
        bufferedWriter.write(stringBuffer);
        this._writer_.flush();
        if (this._commandSupport_.getListenerCount() > 0) {
            this._commandSupport_.fireCommandSent(command, message);
        }
        __getReply();
        return this._replyCode;
    }

    public int sendCommand(int command, String args) throws IOException {
        return sendCommand(NNTPCommand._commands[command], args);
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

    public String getReplyString() {
        return this._replyString;
    }

    public int article(String messageId) throws IOException {
        return sendCommand(0, messageId);
    }

    public int article(int articleNumber) throws IOException {
        return sendCommand(0, Integer.toString(articleNumber));
    }

    public int article() throws IOException {
        return sendCommand(0);
    }

    public int body(String messageId) throws IOException {
        return sendCommand(1, messageId);
    }

    public int body(int articleNumber) throws IOException {
        return sendCommand(1, Integer.toString(articleNumber));
    }

    public int body() throws IOException {
        return sendCommand(1);
    }

    public int head(String messageId) throws IOException {
        return sendCommand(3, messageId);
    }

    public int head(int articleNumber) throws IOException {
        return sendCommand(3, Integer.toString(articleNumber));
    }

    public int head() throws IOException {
        return sendCommand(3);
    }

    public int stat(String messageId) throws IOException {
        return sendCommand(14, messageId);
    }

    public int stat(int articleNumber) throws IOException {
        return sendCommand(14, Integer.toString(articleNumber));
    }

    public int stat() throws IOException {
        return sendCommand(14);
    }

    public int group(String newsgroup) throws IOException {
        return sendCommand(2, newsgroup);
    }

    public int help() throws IOException {
        return sendCommand(4);
    }

    public int ihave(String messageId) throws IOException {
        return sendCommand(5, messageId);
    }

    public int last() throws IOException {
        return sendCommand(6);
    }

    public int list() throws IOException {
        return sendCommand(7);
    }

    public int next() throws IOException {
        return sendCommand(10);
    }

    public int newgroups(String date, String time, boolean GMT, String distributions) throws IOException {
        StringBuffer buffer = new StringBuffer();
        buffer.append(date);
        buffer.append(' ');
        buffer.append(time);
        if (GMT) {
            buffer.append(' ');
            buffer.append("GMT");
        }
        if (distributions != null) {
            buffer.append(" <");
            buffer.append(distributions);
            buffer.append('>');
        }
        return sendCommand(8, buffer.toString());
    }

    public int newnews(String newsgroups, String date, String time, boolean GMT, String distributions) throws IOException {
        StringBuffer buffer = new StringBuffer();
        buffer.append(newsgroups);
        buffer.append(' ');
        buffer.append(date);
        buffer.append(' ');
        buffer.append(time);
        if (GMT) {
            buffer.append(' ');
            buffer.append("GMT");
        }
        if (distributions != null) {
            buffer.append(" <");
            buffer.append(distributions);
            buffer.append('>');
        }
        return sendCommand(9, buffer.toString());
    }

    public int post() throws IOException {
        return sendCommand(11);
    }

    public int quit() throws IOException {
        return sendCommand(12);
    }

    public int authinfoUser(String username) throws IOException {
        return sendCommand(15, "USER " + username);
    }

    public int authinfoPass(String password) throws IOException {
        return sendCommand(15, "PASS " + password);
    }

    public int xover(String selectedArticles) throws IOException {
        return sendCommand(16, selectedArticles);
    }

    public int xhdr(String header, String selectedArticles) throws IOException {
        StringBuffer command = new StringBuffer(header);
        command.append(" ");
        command.append(selectedArticles);
        return sendCommand(17, command.toString());
    }

    public int listActive(String wildmat) throws IOException {
        StringBuffer command = new StringBuffer("ACTIVE ");
        command.append(wildmat);
        return sendCommand(7, command.toString());
    }
}
