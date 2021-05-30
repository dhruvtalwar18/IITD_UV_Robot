package org.apache.commons.net.bsd;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.ServerSocket;
import java.net.Socket;
import org.apache.commons.net.SocketClient;
import org.apache.commons.net.io.SocketInputStream;

public class RExecClient extends SocketClient {
    public static final int DEFAULT_PORT = 512;
    private boolean __remoteVerificationEnabled;
    protected InputStream _errorStream_ = null;

    /* access modifiers changed from: package-private */
    public InputStream _createErrorStream() throws IOException {
        ServerSocket server = this._serverSocketFactory_.createServerSocket(0, 1, getLocalAddress());
        this._output_.write(Integer.toString(server.getLocalPort()).getBytes());
        this._output_.write(0);
        this._output_.flush();
        Socket socket = server.accept();
        server.close();
        if (!this.__remoteVerificationEnabled || verifyRemote(socket)) {
            return new SocketInputStream(socket, socket.getInputStream());
        }
        socket.close();
        throw new IOException("Security violation: unexpected connection attempt by " + socket.getInetAddress().getHostAddress());
    }

    public RExecClient() {
        setDefaultPort(512);
    }

    public InputStream getInputStream() {
        return this._input_;
    }

    public OutputStream getOutputStream() {
        return this._output_;
    }

    public InputStream getErrorStream() {
        return this._errorStream_;
    }

    public void rexec(String username, String password, String command, boolean separateErrorStream) throws IOException {
        if (separateErrorStream) {
            this._errorStream_ = _createErrorStream();
        } else {
            this._output_.write(0);
        }
        this._output_.write(username.getBytes());
        this._output_.write(0);
        this._output_.write(password.getBytes());
        this._output_.write(0);
        this._output_.write(command.getBytes());
        this._output_.write(0);
        this._output_.flush();
        int ch = this._input_.read();
        if (ch > 0) {
            StringBuffer buffer = new StringBuffer();
            while (true) {
                int read = this._input_.read();
                int ch2 = read;
                if (read != -1 && ch2 != 10) {
                    buffer.append((char) ch2);
                }
            }
            throw new IOException(buffer.toString());
        } else if (ch < 0) {
            throw new IOException("Server closed connection.");
        }
    }

    public void rexec(String username, String password, String command) throws IOException {
        rexec(username, password, command, false);
    }

    public void disconnect() throws IOException {
        if (this._errorStream_ != null) {
            this._errorStream_.close();
        }
        this._errorStream_ = null;
        super.disconnect();
    }

    public final void setRemoteVerificationEnabled(boolean enable) {
        this.__remoteVerificationEnabled = enable;
    }

    public final boolean isRemoteVerificationEnabled() {
        return this.__remoteVerificationEnabled;
    }
}
