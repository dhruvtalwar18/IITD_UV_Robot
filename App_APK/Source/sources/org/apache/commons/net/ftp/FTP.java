package org.apache.commons.net.ftp;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import java.net.InetAddress;
import java.net.Socket;
import java.net.SocketException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Iterator;
import org.apache.commons.net.MalformedServerReplyException;
import org.apache.commons.net.ProtocolCommandListener;
import org.apache.commons.net.ProtocolCommandSupport;
import org.apache.commons.net.SocketClient;

public class FTP extends SocketClient {
    public static final int ASCII_FILE_TYPE = 0;
    public static final int BINARY_FILE_TYPE = 2;
    public static final int BLOCK_TRANSFER_MODE = 11;
    public static final int CARRIAGE_CONTROL_TEXT_FORMAT = 6;
    public static final int COMPRESSED_TRANSFER_MODE = 12;
    public static final String DEFAULT_CONTROL_ENCODING = "ISO-8859-1";
    public static final int DEFAULT_DATA_PORT = 20;
    public static final int DEFAULT_PORT = 21;
    public static final int EBCDIC_FILE_TYPE = 1;
    public static final int FILE_STRUCTURE = 7;
    public static final int LOCAL_FILE_TYPE = 3;
    public static final int NON_PRINT_TEXT_FORMAT = 4;
    public static final int PAGE_STRUCTURE = 9;
    public static final int RECORD_STRUCTURE = 8;
    public static final int STREAM_TRANSFER_MODE = 10;
    public static final int TELNET_TEXT_FORMAT = 5;
    private static final String __modes = "AEILNTCFRPSBC";
    private StringBuilder __commandBuffer = new StringBuilder();
    protected ProtocolCommandSupport _commandSupport_;
    protected String _controlEncoding;
    protected BufferedReader _controlInput_;
    protected BufferedWriter _controlOutput_;
    protected boolean _newReplyString;
    protected int _replyCode;
    protected ArrayList<String> _replyLines;
    protected String _replyString;
    protected boolean strictMultilineParsing = false;

    public FTP() {
        setDefaultPort(21);
        this._replyLines = new ArrayList<>();
        this._newReplyString = false;
        this._replyString = null;
        this._commandSupport_ = new ProtocolCommandSupport(this);
        this._controlEncoding = "ISO-8859-1";
    }

    private boolean __strictCheck(String line, String code) {
        return !line.startsWith(code) || line.charAt(3) != ' ';
    }

    private boolean __lenientCheck(String line) {
        return line.length() < 4 || line.charAt(3) == '-' || !Character.isDigit(line.charAt(0));
    }

    private void __getReply() throws IOException {
        this._newReplyString = true;
        this._replyLines.clear();
        String line = this._controlInput_.readLine();
        if (line != null) {
            int length = line.length();
            if (length >= 3) {
                try {
                    String code = line.substring(0, 3);
                    this._replyCode = Integer.parseInt(code);
                    this._replyLines.add(line);
                    if (length > 3 && line.charAt(3) == '-') {
                        while (true) {
                            String line2 = this._controlInput_.readLine();
                            if (line2 != null) {
                                this._replyLines.add(line2);
                                if (isStrictMultilineParsing()) {
                                    if (!__strictCheck(line2, code)) {
                                        break;
                                    }
                                } else if (!__lenientCheck(line2)) {
                                    break;
                                }
                            } else {
                                throw new FTPConnectionClosedException("Connection closed without indication.");
                            }
                        }
                    }
                    if (this._commandSupport_.getListenerCount() > 0) {
                        this._commandSupport_.fireReplyReceived(this._replyCode, getReplyString());
                    }
                    if (this._replyCode == 421) {
                        throw new FTPConnectionClosedException("FTP response 421 received.  Server closed connection.");
                    }
                } catch (NumberFormatException e) {
                    throw new MalformedServerReplyException("Could not parse response code.\nServer Reply: " + line);
                }
            } else {
                throw new MalformedServerReplyException("Truncated server reply: " + line);
            }
        } else {
            throw new FTPConnectionClosedException("Connection closed without indication.");
        }
    }

    /* access modifiers changed from: protected */
    public void _connectAction_() throws IOException {
        super._connectAction_();
        this._controlInput_ = new BufferedReader(new InputStreamReader(this._socket_.getInputStream(), getControlEncoding()));
        this._controlOutput_ = new BufferedWriter(new OutputStreamWriter(this._socket_.getOutputStream(), getControlEncoding()));
        __getReply();
        if (FTPReply.isPositivePreliminary(this._replyCode)) {
            __getReply();
        }
    }

    public void setControlEncoding(String encoding) {
        this._controlEncoding = encoding;
    }

    public String getControlEncoding() {
        return this._controlEncoding;
    }

    public void addProtocolCommandListener(ProtocolCommandListener listener) {
        this._commandSupport_.addProtocolCommandListener(listener);
    }

    public void removeProtocolCommandListener(ProtocolCommandListener listener) {
        this._commandSupport_.removeProtocolCommandListener(listener);
    }

    public void disconnect() throws IOException {
        super.disconnect();
        this._controlInput_ = null;
        this._controlOutput_ = null;
        this._newReplyString = false;
        this._replyString = null;
    }

    public int sendCommand(String command, String args) throws IOException {
        this.__commandBuffer.setLength(0);
        this.__commandBuffer.append(command);
        if (args != null) {
            this.__commandBuffer.append(' ');
            this.__commandBuffer.append(args);
        }
        this.__commandBuffer.append("\r\n");
        try {
            BufferedWriter bufferedWriter = this._controlOutput_;
            String sb = this.__commandBuffer.toString();
            String message = sb;
            bufferedWriter.write(sb);
            this._controlOutput_.flush();
            if (this._commandSupport_.getListenerCount() > 0) {
                this._commandSupport_.fireCommandSent(command, message);
            }
            __getReply();
            return this._replyCode;
        } catch (SocketException e) {
            if (!isConnected() || !socketIsConnected(this._socket_)) {
                throw new FTPConnectionClosedException("Connection unexpectedly closed.");
            }
            throw e;
        }
    }

    private boolean socketIsConnected(Socket socket) {
        if (socket == null) {
            return false;
        }
        return socket.isConnected();
    }

    public int sendCommand(int command, String args) throws IOException {
        return sendCommand(FTPCommand._commands[command], args);
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
        StringBuilder buffer = new StringBuilder(256);
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

    public int user(String username) throws IOException {
        return sendCommand(0, username);
    }

    public int pass(String password) throws IOException {
        return sendCommand(1, password);
    }

    public int acct(String account) throws IOException {
        return sendCommand(2, account);
    }

    public int abor() throws IOException {
        return sendCommand(21);
    }

    public int cwd(String directory) throws IOException {
        return sendCommand(3, directory);
    }

    public int cdup() throws IOException {
        return sendCommand(4);
    }

    public int quit() throws IOException {
        return sendCommand(7);
    }

    public int rein() throws IOException {
        return sendCommand(6);
    }

    public int smnt(String dir) throws IOException {
        return sendCommand(5, dir);
    }

    public int port(InetAddress host, int port) throws IOException {
        StringBuffer info = new StringBuffer(24);
        info.append(host.getHostAddress().replace('.', ','));
        info.append(',');
        info.append(port >>> 8);
        info.append(',');
        info.append(port & 255);
        return sendCommand(8, info.toString());
    }

    public int pasv() throws IOException {
        return sendCommand(9);
    }

    public int type(int fileType, int formatOrByteSize) throws IOException {
        StringBuffer arg = new StringBuffer();
        arg.append(__modes.charAt(fileType));
        arg.append(' ');
        if (fileType == 3) {
            arg.append(formatOrByteSize);
        } else {
            arg.append(__modes.charAt(formatOrByteSize));
        }
        return sendCommand(10, arg.toString());
    }

    public int type(int fileType) throws IOException {
        return sendCommand(10, __modes.substring(fileType, fileType + 1));
    }

    public int stru(int structure) throws IOException {
        return sendCommand(11, __modes.substring(structure, structure + 1));
    }

    public int mode(int mode) throws IOException {
        return sendCommand(12, __modes.substring(mode, mode + 1));
    }

    public int retr(String pathname) throws IOException {
        return sendCommand(13, pathname);
    }

    public int stor(String pathname) throws IOException {
        return sendCommand(14, pathname);
    }

    public int stou() throws IOException {
        return sendCommand(15);
    }

    public int stou(String pathname) throws IOException {
        return sendCommand(15, pathname);
    }

    public int appe(String pathname) throws IOException {
        return sendCommand(16, pathname);
    }

    public int allo(int bytes) throws IOException {
        return sendCommand(17, Integer.toString(bytes));
    }

    public int allo(int bytes, int recordSize) throws IOException {
        return sendCommand(17, Integer.toString(bytes) + " R " + Integer.toString(recordSize));
    }

    public int rest(String marker) throws IOException {
        return sendCommand(18, marker);
    }

    public int mdtm(String file) throws IOException {
        return sendCommand(33, file);
    }

    public int rnfr(String pathname) throws IOException {
        return sendCommand(19, pathname);
    }

    public int rnto(String pathname) throws IOException {
        return sendCommand(20, pathname);
    }

    public int dele(String pathname) throws IOException {
        return sendCommand(22, pathname);
    }

    public int rmd(String pathname) throws IOException {
        return sendCommand(23, pathname);
    }

    public int mkd(String pathname) throws IOException {
        return sendCommand(24, pathname);
    }

    public int pwd() throws IOException {
        return sendCommand(25);
    }

    public int list() throws IOException {
        return sendCommand(26);
    }

    public int list(String pathname) throws IOException {
        return sendCommand(26, pathname);
    }

    public int nlst() throws IOException {
        return sendCommand(27);
    }

    public int nlst(String pathname) throws IOException {
        return sendCommand(27, pathname);
    }

    public int site(String parameters) throws IOException {
        return sendCommand(28, parameters);
    }

    public int syst() throws IOException {
        return sendCommand(29);
    }

    public int stat() throws IOException {
        return sendCommand(30);
    }

    public int stat(String pathname) throws IOException {
        return sendCommand(30, pathname);
    }

    public int help() throws IOException {
        return sendCommand(31);
    }

    public int help(String command) throws IOException {
        return sendCommand(31, command);
    }

    public int noop() throws IOException {
        return sendCommand(32);
    }

    public boolean isStrictMultilineParsing() {
        return this.strictMultilineParsing;
    }

    public void setStrictMultilineParsing(boolean strictMultilineParsing2) {
        this.strictMultilineParsing = strictMultilineParsing2;
    }
}
