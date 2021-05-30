package org.apache.commons.httpclient.protocol;

import java.io.IOException;
import java.net.InetAddress;
import java.net.Socket;
import java.net.UnknownHostException;
import org.apache.commons.httpclient.ConnectTimeoutException;
import org.apache.commons.httpclient.util.TimeoutController;

public final class ControllerThreadSocketFactory {
    private ControllerThreadSocketFactory() {
    }

    public static Socket createSocket(ProtocolSocketFactory socketfactory, String host, int port, InetAddress localAddress, int localPort, int timeout) throws IOException, UnknownHostException, ConnectTimeoutException {
        final ProtocolSocketFactory protocolSocketFactory = socketfactory;
        final String str = host;
        final int i = port;
        final InetAddress inetAddress = localAddress;
        final int i2 = localPort;
        SocketTask task = new SocketTask() {
            public void doit() throws IOException {
                setSocket(ProtocolSocketFactory.this.createSocket(str, i, inetAddress, i2));
            }
        };
        try {
            TimeoutController.execute((Runnable) task, (long) timeout);
            Socket socket = task.getSocket();
            if (task.exception == null) {
                return socket;
            }
            throw task.exception;
        } catch (TimeoutController.TimeoutException e) {
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append("The host did not accept the connection within timeout of ");
            stringBuffer.append(timeout);
            stringBuffer.append(" ms");
            throw new ConnectTimeoutException(stringBuffer.toString());
        }
    }

    public static Socket createSocket(SocketTask task, int timeout) throws IOException, UnknownHostException, ConnectTimeoutException {
        try {
            TimeoutController.execute((Runnable) task, (long) timeout);
            Socket socket = task.getSocket();
            if (task.exception == null) {
                return socket;
            }
            throw task.exception;
        } catch (TimeoutController.TimeoutException e) {
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append("The host did not accept the connection within timeout of ");
            stringBuffer.append(timeout);
            stringBuffer.append(" ms");
            throw new ConnectTimeoutException(stringBuffer.toString());
        }
    }

    public static abstract class SocketTask implements Runnable {
        /* access modifiers changed from: private */
        public IOException exception;
        private Socket socket;

        public abstract void doit() throws IOException;

        /* access modifiers changed from: protected */
        public void setSocket(Socket newSocket) {
            this.socket = newSocket;
        }

        /* access modifiers changed from: protected */
        public Socket getSocket() {
            return this.socket;
        }

        public void run() {
            try {
                doit();
            } catch (IOException e) {
                this.exception = e;
            }
        }
    }
}
