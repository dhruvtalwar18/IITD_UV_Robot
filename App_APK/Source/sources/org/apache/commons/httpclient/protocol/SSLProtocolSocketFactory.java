package org.apache.commons.httpclient.protocol;

import java.io.IOException;
import java.net.InetAddress;
import java.net.Socket;
import java.net.UnknownHostException;
import javax.net.ssl.SSLSocketFactory;
import org.apache.commons.httpclient.ConnectTimeoutException;
import org.apache.commons.httpclient.params.HttpConnectionParams;

public class SSLProtocolSocketFactory implements SecureProtocolSocketFactory {
    private static final SSLProtocolSocketFactory factory = new SSLProtocolSocketFactory();

    static SSLProtocolSocketFactory getSocketFactory() {
        return factory;
    }

    public Socket createSocket(String host, int port, InetAddress clientHost, int clientPort) throws IOException, UnknownHostException {
        return SSLSocketFactory.getDefault().createSocket(host, port, clientHost, clientPort);
    }

    public Socket createSocket(String host, int port, InetAddress localAddress, int localPort, HttpConnectionParams params) throws IOException, UnknownHostException, ConnectTimeoutException {
        if (params != null) {
            int timeout = params.getConnectionTimeout();
            if (timeout == 0) {
                return createSocket(host, port, localAddress, localPort);
            }
            Socket socket = ReflectionSocketFactory.createSocket("javax.net.ssl.SSLSocketFactory", host, port, localAddress, localPort, timeout);
            if (socket == null) {
                return ControllerThreadSocketFactory.createSocket(this, host, port, localAddress, localPort, timeout);
            }
            return socket;
        }
        throw new IllegalArgumentException("Parameters may not be null");
    }

    public Socket createSocket(String host, int port) throws IOException, UnknownHostException {
        return SSLSocketFactory.getDefault().createSocket(host, port);
    }

    public Socket createSocket(Socket socket, String host, int port, boolean autoClose) throws IOException, UnknownHostException {
        return ((SSLSocketFactory) SSLSocketFactory.getDefault()).createSocket(socket, host, port, autoClose);
    }

    public boolean equals(Object obj) {
        return obj != null && obj.getClass().equals(getClass());
    }

    public int hashCode() {
        return getClass().hashCode();
    }
}
