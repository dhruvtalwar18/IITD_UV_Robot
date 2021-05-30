package org.apache.commons.net;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.net.Socket;
import java.net.SocketException;
import javax.net.ServerSocketFactory;
import javax.net.SocketFactory;

public abstract class SocketClient {
    private static final int DEFAULT_CONNECT_TIMEOUT = 0;
    public static final String NETASCII_EOL = "\r\n";
    private static final ServerSocketFactory __DEFAULT_SERVER_SOCKET_FACTORY = ServerSocketFactory.getDefault();
    private static final SocketFactory __DEFAULT_SOCKET_FACTORY = SocketFactory.getDefault();
    protected int _defaultPort_ = 0;
    protected InputStream _input_ = null;
    protected OutputStream _output_ = null;
    protected ServerSocketFactory _serverSocketFactory_ = __DEFAULT_SERVER_SOCKET_FACTORY;
    protected SocketFactory _socketFactory_ = __DEFAULT_SOCKET_FACTORY;
    protected Socket _socket_ = null;
    protected int _timeout_ = 0;
    protected int connectTimeout = 0;

    /* access modifiers changed from: protected */
    public void _connectAction_() throws IOException {
        this._socket_.setSoTimeout(this._timeout_);
        this._input_ = this._socket_.getInputStream();
        this._output_ = this._socket_.getOutputStream();
    }

    public void connect(InetAddress host, int port) throws SocketException, IOException {
        this._socket_ = this._socketFactory_.createSocket();
        this._socket_.connect(new InetSocketAddress(host, port), this.connectTimeout);
        _connectAction_();
    }

    public void connect(String hostname, int port) throws SocketException, IOException {
        this._socket_ = this._socketFactory_.createSocket();
        this._socket_.connect(new InetSocketAddress(hostname, port), this.connectTimeout);
        _connectAction_();
    }

    public void connect(InetAddress host, int port, InetAddress localAddr, int localPort) throws SocketException, IOException {
        this._socket_ = this._socketFactory_.createSocket();
        this._socket_.bind(new InetSocketAddress(localAddr, localPort));
        this._socket_.connect(new InetSocketAddress(host, port), this.connectTimeout);
        _connectAction_();
    }

    public void connect(String hostname, int port, InetAddress localAddr, int localPort) throws SocketException, IOException {
        this._socket_ = this._socketFactory_.createSocket(hostname, port, localAddr, localPort);
        _connectAction_();
    }

    public void connect(InetAddress host) throws SocketException, IOException {
        connect(host, this._defaultPort_);
    }

    public void connect(String hostname) throws SocketException, IOException {
        connect(hostname, this._defaultPort_);
    }

    public void disconnect() throws IOException {
        if (this._socket_ != null) {
            this._socket_.close();
        }
        if (this._input_ != null) {
            this._input_.close();
        }
        if (this._output_ != null) {
            this._output_.close();
        }
        if (this._socket_ != null) {
            this._socket_ = null;
        }
        this._input_ = null;
        this._output_ = null;
    }

    public boolean isConnected() {
        if (this._socket_ == null) {
            return false;
        }
        return this._socket_.isConnected();
    }

    public void setDefaultPort(int port) {
        this._defaultPort_ = port;
    }

    public int getDefaultPort() {
        return this._defaultPort_;
    }

    public void setDefaultTimeout(int timeout) {
        this._timeout_ = timeout;
    }

    public int getDefaultTimeout() {
        return this._timeout_;
    }

    public void setSoTimeout(int timeout) throws SocketException {
        this._socket_.setSoTimeout(timeout);
    }

    public void setSendBufferSize(int size) throws SocketException {
        this._socket_.setSendBufferSize(size);
    }

    public void setReceiveBufferSize(int size) throws SocketException {
        this._socket_.setReceiveBufferSize(size);
    }

    public int getSoTimeout() throws SocketException {
        return this._socket_.getSoTimeout();
    }

    public void setTcpNoDelay(boolean on) throws SocketException {
        this._socket_.setTcpNoDelay(on);
    }

    public boolean getTcpNoDelay() throws SocketException {
        return this._socket_.getTcpNoDelay();
    }

    public void setSoLinger(boolean on, int val) throws SocketException {
        this._socket_.setSoLinger(on, val);
    }

    public int getSoLinger() throws SocketException {
        return this._socket_.getSoLinger();
    }

    public int getLocalPort() {
        return this._socket_.getLocalPort();
    }

    public InetAddress getLocalAddress() {
        return this._socket_.getLocalAddress();
    }

    public int getRemotePort() {
        return this._socket_.getPort();
    }

    public InetAddress getRemoteAddress() {
        return this._socket_.getInetAddress();
    }

    public boolean verifyRemote(Socket socket) {
        return socket.getInetAddress().equals(getRemoteAddress());
    }

    public void setSocketFactory(SocketFactory factory) {
        if (factory == null) {
            this._socketFactory_ = __DEFAULT_SOCKET_FACTORY;
        } else {
            this._socketFactory_ = factory;
        }
    }

    public void setServerSocketFactory(ServerSocketFactory factory) {
        if (factory == null) {
            this._serverSocketFactory_ = __DEFAULT_SERVER_SOCKET_FACTORY;
        } else {
            this._serverSocketFactory_ = factory;
        }
    }

    public void setConnectTimeout(int connectTimeout2) {
        this.connectTimeout = connectTimeout2;
    }

    public int getConnectTimeout() {
        return this.connectTimeout;
    }
}
