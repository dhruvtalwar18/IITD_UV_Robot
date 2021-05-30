package org.apache.commons.net.ftp;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import java.net.Socket;
import java.security.KeyManagementException;
import java.security.NoSuchAlgorithmException;
import java.security.SecureRandom;
import javax.net.ServerSocketFactory;
import javax.net.SocketFactory;
import javax.net.ssl.KeyManager;
import javax.net.ssl.SSLContext;
import javax.net.ssl.SSLException;
import javax.net.ssl.SSLSocket;
import javax.net.ssl.TrustManager;

public class FTPSClient extends FTPClient {
    private static final String DEFAULT_PROT = "C";
    private static final String DEFAULT_PROTOCOL = "TLS";
    public static String KEYSTORE_ALGORITHM;
    private static final String[] PROT_COMMAND_VALUE = {DEFAULT_PROT, "E", "S", "P"};
    public static String PROVIDER;
    public static String STORE_TYPE;
    public static String TRUSTSTORE_ALGORITHM;
    private String auth;
    private SSLContext context;
    private boolean isClientMode;
    private boolean isCreation;
    private boolean isImplicit;
    private boolean isNeedClientAuth;
    private boolean isWantClientAuth;
    private KeyManager keyManager;
    private Socket planeSocket;
    private String protocol;
    private String[] protocols;
    private String[] suites;
    private TrustManager trustManager;

    public FTPSClient() throws NoSuchAlgorithmException {
        this.protocol = DEFAULT_PROTOCOL;
        this.auth = DEFAULT_PROTOCOL;
        this.isCreation = true;
        this.isClientMode = true;
        this.isNeedClientAuth = false;
        this.isWantClientAuth = false;
        this.suites = null;
        this.protocols = null;
        this.trustManager = new FTPSTrustManager();
        this.protocol = DEFAULT_PROTOCOL;
        this.isImplicit = false;
    }

    public FTPSClient(boolean isImplicit2) throws NoSuchAlgorithmException {
        this.protocol = DEFAULT_PROTOCOL;
        this.auth = DEFAULT_PROTOCOL;
        this.isCreation = true;
        this.isClientMode = true;
        this.isNeedClientAuth = false;
        this.isWantClientAuth = false;
        this.suites = null;
        this.protocols = null;
        this.trustManager = new FTPSTrustManager();
        this.protocol = DEFAULT_PROTOCOL;
        this.isImplicit = isImplicit2;
    }

    public FTPSClient(String protocol2) throws NoSuchAlgorithmException {
        this.protocol = DEFAULT_PROTOCOL;
        this.auth = DEFAULT_PROTOCOL;
        this.isCreation = true;
        this.isClientMode = true;
        this.isNeedClientAuth = false;
        this.isWantClientAuth = false;
        this.suites = null;
        this.protocols = null;
        this.trustManager = new FTPSTrustManager();
        this.protocol = protocol2;
        this.isImplicit = false;
    }

    public FTPSClient(String protocol2, boolean isImplicit2) throws NoSuchAlgorithmException {
        this.protocol = DEFAULT_PROTOCOL;
        this.auth = DEFAULT_PROTOCOL;
        this.isCreation = true;
        this.isClientMode = true;
        this.isNeedClientAuth = false;
        this.isWantClientAuth = false;
        this.suites = null;
        this.protocols = null;
        this.trustManager = new FTPSTrustManager();
        this.protocol = protocol2;
        this.isImplicit = isImplicit2;
    }

    public void setAuthValue(String auth2) {
        this.auth = auth2;
    }

    public String getAuthValue() {
        return this.auth;
    }

    /* access modifiers changed from: protected */
    public void _connectAction_() throws IOException {
        if (this.isImplicit) {
            sslNegotiation();
        }
        super._connectAction_();
        if (!this.isImplicit) {
            execAUTH();
            sslNegotiation();
        }
    }

    private void execAUTH() throws SSLException, IOException {
        int replyCode = sendCommand(FTPSCommand._commands[0], this.auth);
        if (334 != replyCode && 234 != replyCode) {
            throw new SSLException(getReplyString());
        }
    }

    private void initSslContext() throws IOException {
        if (this.context == null) {
            try {
                this.context = SSLContext.getInstance(this.protocol);
                this.context.init(new KeyManager[]{getKeyManager()}, new TrustManager[]{getTrustManager()}, (SecureRandom) null);
            } catch (KeyManagementException e) {
                IOException ioe = new IOException("Could not initialize SSL context");
                ioe.initCause(e);
                throw ioe;
            } catch (NoSuchAlgorithmException e2) {
                IOException ioe2 = new IOException("Could not initialize SSL context");
                ioe2.initCause(e2);
                throw ioe2;
            }
        }
    }

    private void sslNegotiation() throws IOException {
        this.planeSocket = this._socket_;
        initSslContext();
        SSLSocket socket = (SSLSocket) this.context.getSocketFactory().createSocket(this._socket_, this._socket_.getInetAddress().getHostAddress(), this._socket_.getPort(), true);
        socket.setEnableSessionCreation(this.isCreation);
        socket.setUseClientMode(this.isClientMode);
        if (!this.isClientMode) {
            socket.setNeedClientAuth(this.isNeedClientAuth);
            socket.setWantClientAuth(this.isWantClientAuth);
        }
        if (this.protocols != null) {
            socket.setEnabledProtocols(this.protocols);
        }
        if (this.suites != null) {
            socket.setEnabledCipherSuites(this.suites);
        }
        socket.startHandshake();
        this._socket_ = socket;
        this._controlInput_ = new BufferedReader(new InputStreamReader(socket.getInputStream(), getControlEncoding()));
        this._controlOutput_ = new BufferedWriter(new OutputStreamWriter(socket.getOutputStream(), getControlEncoding()));
    }

    private KeyManager getKeyManager() {
        return this.keyManager;
    }

    public void setKeyManager(KeyManager keyManager2) {
        this.keyManager = keyManager2;
    }

    public void setEnabledSessionCreation(boolean isCreation2) {
        this.isCreation = isCreation2;
    }

    public boolean getEnableSessionCreation() {
        if (this._socket_ instanceof SSLSocket) {
            return ((SSLSocket) this._socket_).getEnableSessionCreation();
        }
        return false;
    }

    public void setNeedClientAuth(boolean isNeedClientAuth2) {
        this.isNeedClientAuth = isNeedClientAuth2;
    }

    public boolean getNeedClientAuth() {
        if (this._socket_ instanceof SSLSocket) {
            return ((SSLSocket) this._socket_).getNeedClientAuth();
        }
        return false;
    }

    public void setWantClientAuth(boolean isWantClientAuth2) {
        this.isWantClientAuth = isWantClientAuth2;
    }

    public boolean getWantClientAuth() {
        if (this._socket_ instanceof SSLSocket) {
            return ((SSLSocket) this._socket_).getWantClientAuth();
        }
        return false;
    }

    public void setUseClientMode(boolean isClientMode2) {
        this.isClientMode = isClientMode2;
    }

    public boolean getUseClientMode() {
        if (this._socket_ instanceof SSLSocket) {
            return ((SSLSocket) this._socket_).getUseClientMode();
        }
        return false;
    }

    public void setEnabledCipherSuites(String[] cipherSuites) {
        this.suites = new String[cipherSuites.length];
        System.arraycopy(cipherSuites, 0, this.suites, 0, cipherSuites.length);
    }

    public String[] getEnabledCipherSuites() {
        if (this._socket_ instanceof SSLSocket) {
            return ((SSLSocket) this._socket_).getEnabledCipherSuites();
        }
        return null;
    }

    public void setEnabledProtocols(String[] protocolVersions) {
        this.protocols = new String[protocolVersions.length];
        System.arraycopy(protocolVersions, 0, this.protocols, 0, protocolVersions.length);
    }

    public String[] getEnabledProtocols() {
        if (this._socket_ instanceof SSLSocket) {
            return ((SSLSocket) this._socket_).getEnabledProtocols();
        }
        return null;
    }

    public void execPBSZ(long pbsz) throws SSLException, IOException {
        if (pbsz < 0 || 4294967295L < pbsz) {
            throw new IllegalArgumentException();
        } else if (200 != sendCommand(FTPSCommand._commands[2], String.valueOf(pbsz))) {
            throw new SSLException(getReplyString());
        }
    }

    public void execPROT(String prot) throws SSLException, IOException {
        if (prot == null) {
            prot = DEFAULT_PROT;
        }
        if (!checkPROTValue(prot)) {
            throw new IllegalArgumentException();
        } else if (200 != sendCommand(FTPSCommand._commands[3], prot)) {
            throw new SSLException(getReplyString());
        } else if (DEFAULT_PROT.equals(prot)) {
            setSocketFactory((SocketFactory) null);
            setServerSocketFactory((ServerSocketFactory) null);
        } else {
            setSocketFactory(new FTPSSocketFactory(this.context));
            initSslContext();
            setServerSocketFactory(this.context.getServerSocketFactory());
        }
    }

    private boolean checkPROTValue(String prot) {
        for (String equals : PROT_COMMAND_VALUE) {
            if (equals.equals(prot)) {
                return true;
            }
        }
        return false;
    }

    public int sendCommand(String command, String args) throws IOException {
        int repCode = super.sendCommand(command, args);
        if (FTPSCommand._commands[4].equals(command)) {
            if (200 == repCode) {
                this._socket_ = this.planeSocket;
                setSocketFactory((SocketFactory) null);
            } else {
                throw new SSLException(getReplyString());
            }
        }
        return repCode;
    }

    /* access modifiers changed from: protected */
    public Socket _openDataConnection_(int command, String arg) throws IOException {
        Socket socket = super._openDataConnection_(command, arg);
        if (socket != null && (socket instanceof SSLSocket)) {
            SSLSocket sslSocket = (SSLSocket) socket;
            sslSocket.setUseClientMode(this.isClientMode);
            sslSocket.setEnableSessionCreation(this.isCreation);
            if (!this.isClientMode) {
                sslSocket.setNeedClientAuth(this.isNeedClientAuth);
                sslSocket.setWantClientAuth(this.isWantClientAuth);
            }
            if (this.suites != null) {
                sslSocket.setEnabledCipherSuites(this.suites);
            }
            if (this.protocols != null) {
                sslSocket.setEnabledProtocols(this.protocols);
            }
            sslSocket.startHandshake();
        }
        return socket;
    }

    public TrustManager getTrustManager() {
        return this.trustManager;
    }

    public void setTrustManager(TrustManager trustManager2) {
        this.trustManager = trustManager2;
    }
}
