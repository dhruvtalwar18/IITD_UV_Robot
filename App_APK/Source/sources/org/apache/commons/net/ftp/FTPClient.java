package org.apache.commons.net.ftp;

import java.io.BufferedInputStream;
import java.io.BufferedOutputStream;
import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.net.InetAddress;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.ArrayList;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import org.apache.commons.net.MalformedServerReplyException;
import org.apache.commons.net.ftp.parser.DefaultFTPFileEntryParserFactory;
import org.apache.commons.net.ftp.parser.FTPFileEntryParserFactory;
import org.apache.commons.net.io.FromNetASCIIInputStream;
import org.apache.commons.net.io.SocketInputStream;
import org.apache.commons.net.io.SocketOutputStream;
import org.apache.commons.net.io.ToNetASCIIOutputStream;

public class FTPClient extends FTP implements Configurable {
    public static final int ACTIVE_LOCAL_DATA_CONNECTION_MODE = 0;
    public static final int ACTIVE_REMOTE_DATA_CONNECTION_MODE = 1;
    public static final int PASSIVE_LOCAL_DATA_CONNECTION_MODE = 2;
    public static final int PASSIVE_REMOTE_DATA_CONNECTION_MODE = 3;
    private static String __parms = "\\d{1,3},\\d{1,3},\\d{1,3},\\d{1,3},\\d{1,3},\\d{1,3}";
    private static Pattern __parms_pat = Pattern.compile(__parms);
    private int __bufferSize;
    private FTPClientConfig __configuration = null;
    private int __dataConnectionMode;
    private int __dataTimeout = -1;
    private FTPFileEntryParser __entryParser;
    private int __fileFormat;
    private int __fileStructure;
    private int __fileTransferMode;
    private int __fileType;
    private boolean __listHiddenFiles = false;
    private FTPFileEntryParserFactory __parserFactory = new DefaultFTPFileEntryParserFactory();
    private String __passiveHost;
    private int __passivePort;
    private boolean __remoteVerificationEnabled = true;
    private long __restartOffset;
    private String __systemName;

    public FTPClient() {
        __initDefaults();
    }

    private void __initDefaults() {
        this.__dataConnectionMode = 0;
        this.__passiveHost = null;
        this.__passivePort = -1;
        this.__fileType = 0;
        this.__fileStructure = 7;
        this.__fileFormat = 4;
        this.__fileTransferMode = 10;
        this.__restartOffset = 0;
        this.__systemName = null;
        this.__entryParser = null;
        this.__bufferSize = 1024;
    }

    private String __parsePathname(String reply) {
        int begin = reply.indexOf(34) + 1;
        return reply.substring(begin, reply.indexOf(34, begin));
    }

    private void __parsePassiveModeReply(String reply) throws MalformedServerReplyException {
        Matcher m = __parms_pat.matcher(reply);
        if (m.find()) {
            String reply2 = m.group();
            String[] parts = m.group().split(",");
            this.__passiveHost = parts[0] + '.' + parts[1] + '.' + parts[2] + '.' + parts[3];
            try {
                this.__passivePort = (Integer.parseInt(parts[4]) << 8) | Integer.parseInt(parts[5]);
            } catch (NumberFormatException e) {
                throw new MalformedServerReplyException("Could not parse passive host information.\nServer Reply: " + reply2);
            }
        } else {
            throw new MalformedServerReplyException("Could not parse passive host information.\nServer Reply: " + reply);
        }
    }

    /* JADX WARNING: Multi-variable type inference failed */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    private boolean __storeFile(int r10, java.lang.String r11, java.io.InputStream r12) throws java.io.IOException {
        /*
            r9 = this;
            java.net.Socket r0 = r9._openDataConnection_(r10, r11)
            r1 = r0
            if (r0 != 0) goto L_0x0009
            r0 = 0
            return r0
        L_0x0009:
            java.io.BufferedOutputStream r0 = new java.io.BufferedOutputStream
            java.io.OutputStream r2 = r1.getOutputStream()
            int r3 = r9.getBufferSize()
            r0.<init>(r2, r3)
            int r2 = r9.__fileType
            if (r2 != 0) goto L_0x0020
            org.apache.commons.net.io.ToNetASCIIOutputStream r2 = new org.apache.commons.net.io.ToNetASCIIOutputStream
            r2.<init>(r0)
            r0 = r2
        L_0x0020:
            int r4 = r9.getBufferSize()     // Catch:{ IOException -> 0x0039 }
            r5 = -1
            r7 = 0
            r8 = 0
            r2 = r12
            r3 = r0
            org.apache.commons.net.io.Util.copyStream(r2, r3, r4, r5, r7, r8)     // Catch:{ IOException -> 0x0039 }
            r0.close()
            r1.close()
            boolean r2 = r9.completePendingCommand()
            return r2
        L_0x0039:
            r2 = move-exception
            r1.close()     // Catch:{ IOException -> 0x003e }
            goto L_0x003f
        L_0x003e:
            r3 = move-exception
        L_0x003f:
            throw r2
        */
        throw new UnsupportedOperationException("Method not decompiled: org.apache.commons.net.ftp.FTPClient.__storeFile(int, java.lang.String, java.io.InputStream):boolean");
    }

    private OutputStream __storeFileStream(int command, String remote) throws IOException {
        Socket _openDataConnection_ = _openDataConnection_(command, remote);
        Socket socket = _openDataConnection_;
        if (_openDataConnection_ == null) {
            return null;
        }
        OutputStream output = socket.getOutputStream();
        if (this.__fileType == 0) {
            output = new ToNetASCIIOutputStream(new BufferedOutputStream(output, getBufferSize()));
        }
        return new SocketOutputStream(socket, output);
    }

    /* JADX INFO: finally extract failed */
    /* access modifiers changed from: protected */
    public Socket _openDataConnection_(int command, String arg) throws IOException {
        Socket socket;
        if (this.__dataConnectionMode != 0 && this.__dataConnectionMode != 2) {
            return null;
        }
        if (this.__dataConnectionMode == 0) {
            ServerSocket server = this._serverSocketFactory_.createServerSocket(0, 1, getLocalAddress());
            if (!FTPReply.isPositiveCompletion(port(getLocalAddress(), server.getLocalPort()))) {
                server.close();
                return null;
            } else if (this.__restartOffset > 0 && !restart(this.__restartOffset)) {
                server.close();
                return null;
            } else if (!FTPReply.isPositivePreliminary(sendCommand(command, arg))) {
                server.close();
                return null;
            } else {
                if (this.__dataTimeout >= 0) {
                    server.setSoTimeout(this.__dataTimeout);
                }
                try {
                    Socket socket2 = server.accept();
                    server.close();
                    socket = socket2;
                } catch (Throwable th) {
                    server.close();
                    throw th;
                }
            }
        } else if (pasv() != 227) {
            return null;
        } else {
            __parsePassiveModeReply((String) this._replyLines.get(this._replyLines.size() - 1));
            socket = this._socketFactory_.createSocket(this.__passiveHost, this.__passivePort);
            if (this.__restartOffset > 0 && !restart(this.__restartOffset)) {
                socket.close();
                return null;
            } else if (!FTPReply.isPositivePreliminary(sendCommand(command, arg))) {
                socket.close();
                return null;
            }
        }
        if (!this.__remoteVerificationEnabled || verifyRemote(socket)) {
            if (this.__dataTimeout >= 0) {
                socket.setSoTimeout(this.__dataTimeout);
            }
            return socket;
        }
        InetAddress host1 = socket.getInetAddress();
        InetAddress host2 = getRemoteAddress();
        socket.close();
        throw new IOException("Host attempting data connection " + host1.getHostAddress() + " is not same as server " + host2.getHostAddress());
    }

    /* access modifiers changed from: protected */
    public void _connectAction_() throws IOException {
        super._connectAction_();
        __initDefaults();
    }

    public void setDataTimeout(int timeout) {
        this.__dataTimeout = timeout;
    }

    public void setParserFactory(FTPFileEntryParserFactory parserFactory) {
        this.__parserFactory = parserFactory;
    }

    public void disconnect() throws IOException {
        super.disconnect();
        __initDefaults();
    }

    public void setRemoteVerificationEnabled(boolean enable) {
        this.__remoteVerificationEnabled = enable;
    }

    public boolean isRemoteVerificationEnabled() {
        return this.__remoteVerificationEnabled;
    }

    public boolean login(String username, String password) throws IOException {
        user(username);
        if (FTPReply.isPositiveCompletion(this._replyCode)) {
            return true;
        }
        if (!FTPReply.isPositiveIntermediate(this._replyCode)) {
            return false;
        }
        return FTPReply.isPositiveCompletion(pass(password));
    }

    public boolean login(String username, String password, String account) throws IOException {
        user(username);
        if (FTPReply.isPositiveCompletion(this._replyCode)) {
            return true;
        }
        if (!FTPReply.isPositiveIntermediate(this._replyCode)) {
            return false;
        }
        pass(password);
        if (FTPReply.isPositiveCompletion(this._replyCode)) {
            return true;
        }
        if (!FTPReply.isPositiveIntermediate(this._replyCode)) {
            return false;
        }
        return FTPReply.isPositiveCompletion(acct(account));
    }

    public boolean logout() throws IOException {
        return FTPReply.isPositiveCompletion(quit());
    }

    public boolean changeWorkingDirectory(String pathname) throws IOException {
        return FTPReply.isPositiveCompletion(cwd(pathname));
    }

    public boolean changeToParentDirectory() throws IOException {
        return FTPReply.isPositiveCompletion(cdup());
    }

    public boolean structureMount(String pathname) throws IOException {
        return FTPReply.isPositiveCompletion(smnt(pathname));
    }

    /* access modifiers changed from: package-private */
    public boolean reinitialize() throws IOException {
        rein();
        if (!FTPReply.isPositiveCompletion(this._replyCode) && (!FTPReply.isPositivePreliminary(this._replyCode) || !FTPReply.isPositiveCompletion(getReply()))) {
            return false;
        }
        __initDefaults();
        return true;
    }

    public void enterLocalActiveMode() {
        this.__dataConnectionMode = 0;
        this.__passiveHost = null;
        this.__passivePort = -1;
    }

    public void enterLocalPassiveMode() {
        this.__dataConnectionMode = 2;
        this.__passiveHost = null;
        this.__passivePort = -1;
    }

    public boolean enterRemoteActiveMode(InetAddress host, int port) throws IOException {
        if (!FTPReply.isPositiveCompletion(port(host, port))) {
            return false;
        }
        this.__dataConnectionMode = 1;
        this.__passiveHost = null;
        this.__passivePort = -1;
        return true;
    }

    public boolean enterRemotePassiveMode() throws IOException {
        if (pasv() != 227) {
            return false;
        }
        this.__dataConnectionMode = 3;
        __parsePassiveModeReply((String) this._replyLines.get(0));
        return true;
    }

    public String getPassiveHost() {
        return this.__passiveHost;
    }

    public int getPassivePort() {
        return this.__passivePort;
    }

    public int getDataConnectionMode() {
        return this.__dataConnectionMode;
    }

    public boolean setFileType(int fileType) throws IOException {
        if (!FTPReply.isPositiveCompletion(type(fileType))) {
            return false;
        }
        this.__fileType = fileType;
        this.__fileFormat = 4;
        return true;
    }

    public boolean setFileType(int fileType, int formatOrByteSize) throws IOException {
        if (!FTPReply.isPositiveCompletion(type(fileType, formatOrByteSize))) {
            return false;
        }
        this.__fileType = fileType;
        this.__fileFormat = formatOrByteSize;
        return true;
    }

    public boolean setFileStructure(int structure) throws IOException {
        if (!FTPReply.isPositiveCompletion(stru(structure))) {
            return false;
        }
        this.__fileStructure = structure;
        return true;
    }

    public boolean setFileTransferMode(int mode) throws IOException {
        if (!FTPReply.isPositiveCompletion(mode(mode))) {
            return false;
        }
        this.__fileTransferMode = mode;
        return true;
    }

    public boolean remoteRetrieve(String filename) throws IOException {
        if (this.__dataConnectionMode == 1 || this.__dataConnectionMode == 3) {
            return FTPReply.isPositivePreliminary(retr(filename));
        }
        return false;
    }

    public boolean remoteStore(String filename) throws IOException {
        if (this.__dataConnectionMode == 1 || this.__dataConnectionMode == 3) {
            return FTPReply.isPositivePreliminary(stor(filename));
        }
        return false;
    }

    public boolean remoteStoreUnique(String filename) throws IOException {
        if (this.__dataConnectionMode == 1 || this.__dataConnectionMode == 3) {
            return FTPReply.isPositivePreliminary(stou(filename));
        }
        return false;
    }

    public boolean remoteStoreUnique() throws IOException {
        if (this.__dataConnectionMode == 1 || this.__dataConnectionMode == 3) {
            return FTPReply.isPositivePreliminary(stou());
        }
        return false;
    }

    public boolean remoteAppend(String filename) throws IOException {
        if (this.__dataConnectionMode == 1 || this.__dataConnectionMode == 3) {
            return FTPReply.isPositivePreliminary(stor(filename));
        }
        return false;
    }

    public boolean completePendingCommand() throws IOException {
        return FTPReply.isPositiveCompletion(getReply());
    }

    /* JADX WARNING: Multi-variable type inference failed */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public boolean retrieveFile(java.lang.String r10, java.io.OutputStream r11) throws java.io.IOException {
        /*
            r9 = this;
            r0 = 13
            java.net.Socket r0 = r9._openDataConnection_(r0, r10)
            r1 = r0
            if (r0 != 0) goto L_0x000b
            r0 = 0
            return r0
        L_0x000b:
            java.io.BufferedInputStream r0 = new java.io.BufferedInputStream
            java.io.InputStream r2 = r1.getInputStream()
            int r3 = r9.getBufferSize()
            r0.<init>(r2, r3)
            int r2 = r9.__fileType
            if (r2 != 0) goto L_0x0022
            org.apache.commons.net.io.FromNetASCIIInputStream r2 = new org.apache.commons.net.io.FromNetASCIIInputStream
            r2.<init>(r0)
            r0 = r2
        L_0x0022:
            int r4 = r9.getBufferSize()     // Catch:{ IOException -> 0x0038 }
            r5 = -1
            r7 = 0
            r8 = 0
            r2 = r0
            r3 = r11
            org.apache.commons.net.io.Util.copyStream(r2, r3, r4, r5, r7, r8)     // Catch:{ IOException -> 0x0038 }
            r1.close()
            boolean r2 = r9.completePendingCommand()
            return r2
        L_0x0038:
            r2 = move-exception
            r1.close()     // Catch:{ IOException -> 0x003d }
            goto L_0x003e
        L_0x003d:
            r3 = move-exception
        L_0x003e:
            throw r2
        */
        throw new UnsupportedOperationException("Method not decompiled: org.apache.commons.net.ftp.FTPClient.retrieveFile(java.lang.String, java.io.OutputStream):boolean");
    }

    public InputStream retrieveFileStream(String remote) throws IOException {
        Socket _openDataConnection_ = _openDataConnection_(13, remote);
        Socket socket = _openDataConnection_;
        if (_openDataConnection_ == null) {
            return null;
        }
        InputStream input = socket.getInputStream();
        if (this.__fileType == 0) {
            input = new FromNetASCIIInputStream(new BufferedInputStream(input, getBufferSize()));
        }
        return new SocketInputStream(socket, input);
    }

    public boolean storeFile(String remote, InputStream local) throws IOException {
        return __storeFile(14, remote, local);
    }

    public OutputStream storeFileStream(String remote) throws IOException {
        return __storeFileStream(14, remote);
    }

    public boolean appendFile(String remote, InputStream local) throws IOException {
        return __storeFile(16, remote, local);
    }

    public OutputStream appendFileStream(String remote) throws IOException {
        return __storeFileStream(16, remote);
    }

    public boolean storeUniqueFile(String remote, InputStream local) throws IOException {
        return __storeFile(15, remote, local);
    }

    public OutputStream storeUniqueFileStream(String remote) throws IOException {
        return __storeFileStream(15, remote);
    }

    public boolean storeUniqueFile(InputStream local) throws IOException {
        return __storeFile(15, (String) null, local);
    }

    public OutputStream storeUniqueFileStream() throws IOException {
        return __storeFileStream(15, (String) null);
    }

    public boolean allocate(int bytes) throws IOException {
        return FTPReply.isPositiveCompletion(allo(bytes));
    }

    public boolean allocate(int bytes, int recordSize) throws IOException {
        return FTPReply.isPositiveCompletion(allo(bytes, recordSize));
    }

    private boolean restart(long offset) throws IOException {
        this.__restartOffset = 0;
        return FTPReply.isPositiveIntermediate(rest(Long.toString(offset)));
    }

    public void setRestartOffset(long offset) {
        if (offset >= 0) {
            this.__restartOffset = offset;
        }
    }

    public long getRestartOffset() {
        return this.__restartOffset;
    }

    public boolean rename(String from, String to) throws IOException {
        if (!FTPReply.isPositiveIntermediate(rnfr(from))) {
            return false;
        }
        return FTPReply.isPositiveCompletion(rnto(to));
    }

    public boolean abort() throws IOException {
        return FTPReply.isPositiveCompletion(abor());
    }

    public boolean deleteFile(String pathname) throws IOException {
        return FTPReply.isPositiveCompletion(dele(pathname));
    }

    public boolean removeDirectory(String pathname) throws IOException {
        return FTPReply.isPositiveCompletion(rmd(pathname));
    }

    public boolean makeDirectory(String pathname) throws IOException {
        return FTPReply.isPositiveCompletion(mkd(pathname));
    }

    public String printWorkingDirectory() throws IOException {
        if (pwd() != 257) {
            return null;
        }
        return __parsePathname((String) this._replyLines.get(this._replyLines.size() - 1));
    }

    public boolean sendSiteCommand(String arguments) throws IOException {
        return FTPReply.isPositiveCompletion(site(arguments));
    }

    public String getSystemName() throws IOException {
        if (this.__systemName == null && FTPReply.isPositiveCompletion(syst())) {
            this.__systemName = ((String) this._replyLines.get(this._replyLines.size() - 1)).substring(4);
        }
        return this.__systemName;
    }

    public String listHelp() throws IOException {
        if (FTPReply.isPositiveCompletion(help())) {
            return getReplyString();
        }
        return null;
    }

    public String listHelp(String command) throws IOException {
        if (FTPReply.isPositiveCompletion(help(command))) {
            return getReplyString();
        }
        return null;
    }

    public boolean sendNoOp() throws IOException {
        return FTPReply.isPositiveCompletion(noop());
    }

    public String[] listNames(String pathname) throws IOException {
        Socket _openDataConnection_ = _openDataConnection_(27, pathname);
        Socket socket = _openDataConnection_;
        if (_openDataConnection_ == null) {
            return null;
        }
        BufferedReader reader = new BufferedReader(new InputStreamReader(socket.getInputStream(), getControlEncoding()));
        ArrayList<String> results = new ArrayList<>();
        while (true) {
            String readLine = reader.readLine();
            String line = readLine;
            if (readLine == null) {
                break;
            }
            results.add(line);
        }
        reader.close();
        socket.close();
        if (completePendingCommand()) {
            return (String[]) results.toArray(new String[results.size()]);
        }
        return null;
    }

    public String[] listNames() throws IOException {
        return listNames((String) null);
    }

    public FTPFile[] listFiles(String pathname) throws IOException {
        return initiateListParsing((String) null, pathname).getFiles();
    }

    public FTPFile[] listFiles() throws IOException {
        return listFiles((String) null);
    }

    public FTPListParseEngine initiateListParsing() throws IOException {
        return initiateListParsing((String) null);
    }

    public FTPListParseEngine initiateListParsing(String pathname) throws IOException {
        return initiateListParsing((String) null, pathname);
    }

    public FTPListParseEngine initiateListParsing(String parserKey, String pathname) throws IOException {
        if (this.__entryParser == null) {
            if (parserKey != null) {
                this.__entryParser = this.__parserFactory.createFileEntryParser(parserKey);
            } else if (this.__configuration != null) {
                this.__entryParser = this.__parserFactory.createFileEntryParser(this.__configuration);
            } else {
                this.__entryParser = this.__parserFactory.createFileEntryParser(getSystemName());
            }
        }
        return initiateListParsing(this.__entryParser, pathname);
    }

    /* JADX INFO: finally extract failed */
    private FTPListParseEngine initiateListParsing(FTPFileEntryParser parser, String pathname) throws IOException {
        FTPListParseEngine engine = new FTPListParseEngine(parser);
        Socket _openDataConnection_ = _openDataConnection_(26, getListArguments(pathname));
        Socket socket = _openDataConnection_;
        if (_openDataConnection_ == null) {
            return engine;
        }
        try {
            engine.readServerList(socket.getInputStream(), getControlEncoding());
            socket.close();
            completePendingCommand();
            return engine;
        } catch (Throwable th) {
            socket.close();
            throw th;
        }
    }

    /* access modifiers changed from: protected */
    public String getListArguments(String pathname) {
        if (!getListHiddenFiles()) {
            return pathname;
        }
        StringBuffer sb = new StringBuffer(pathname.length() + 3);
        sb.append("-a ");
        sb.append(pathname);
        return sb.toString();
    }

    public String getStatus() throws IOException {
        if (FTPReply.isPositiveCompletion(stat())) {
            return getReplyString();
        }
        return null;
    }

    public String getStatus(String pathname) throws IOException {
        if (FTPReply.isPositiveCompletion(stat(pathname))) {
            return getReplyString();
        }
        return null;
    }

    public String getModificationTime(String pathname) throws IOException {
        if (FTPReply.isPositiveCompletion(mdtm(pathname))) {
            return getReplyString();
        }
        return null;
    }

    public void setBufferSize(int bufSize) {
        this.__bufferSize = bufSize;
    }

    public int getBufferSize() {
        return this.__bufferSize;
    }

    public void configure(FTPClientConfig config) {
        this.__configuration = config;
    }

    public void setListHiddenFiles(boolean listHiddenFiles) {
        this.__listHiddenFiles = listHiddenFiles;
    }

    public boolean getListHiddenFiles() {
        return this.__listHiddenFiles;
    }
}
