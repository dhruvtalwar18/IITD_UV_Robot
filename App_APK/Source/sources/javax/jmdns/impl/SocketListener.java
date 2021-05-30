package javax.jmdns.impl;

import java.io.IOException;
import java.net.DatagramPacket;
import java.util.logging.Level;
import java.util.logging.Logger;
import javax.jmdns.impl.constants.DNSConstants;

class SocketListener extends Thread {
    static Logger logger = Logger.getLogger(SocketListener.class.getName());
    private final JmDNSImpl _jmDNSImpl;

    /* JADX WARNING: Illegal instructions before constructor call */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    SocketListener(javax.jmdns.impl.JmDNSImpl r3) {
        /*
            r2 = this;
            java.lang.StringBuilder r0 = new java.lang.StringBuilder
            r0.<init>()
            java.lang.String r1 = "SocketListener("
            r0.append(r1)
            if (r3 == 0) goto L_0x0011
            java.lang.String r1 = r3.getName()
            goto L_0x0013
        L_0x0011:
            java.lang.String r1 = ""
        L_0x0013:
            r0.append(r1)
            java.lang.String r1 = ")"
            r0.append(r1)
            java.lang.String r0 = r0.toString()
            r2.<init>(r0)
            r0 = 1
            r2.setDaemon(r0)
            r2._jmDNSImpl = r3
            return
        */
        throw new UnsupportedOperationException("Method not decompiled: javax.jmdns.impl.SocketListener.<init>(javax.jmdns.impl.JmDNSImpl):void");
    }

    public void run() {
        try {
            byte[] buf = new byte[DNSConstants.MAX_MSG_ABSOLUTE];
            DatagramPacket packet = new DatagramPacket(buf, buf.length);
            while (!this._jmDNSImpl.isCanceling() && !this._jmDNSImpl.isCanceled()) {
                packet.setLength(buf.length);
                this._jmDNSImpl.getSocket().receive(packet);
                if (this._jmDNSImpl.isCanceling() || this._jmDNSImpl.isCanceled() || this._jmDNSImpl.isClosing() || this._jmDNSImpl.isClosed()) {
                    break;
                }
                try {
                    if (!this._jmDNSImpl.getLocalHost().shouldIgnorePacket(packet)) {
                        DNSIncoming msg = new DNSIncoming(packet);
                        if (logger.isLoggable(Level.FINEST)) {
                            Logger logger2 = logger;
                            logger2.finest(getName() + ".run() JmDNS in:" + msg.print(true));
                        }
                        if (msg.isQuery()) {
                            if (packet.getPort() != DNSConstants.MDNS_PORT) {
                                this._jmDNSImpl.handleQuery(msg, packet.getAddress(), packet.getPort());
                            }
                            this._jmDNSImpl.handleQuery(msg, this._jmDNSImpl.getGroup(), DNSConstants.MDNS_PORT);
                        } else {
                            this._jmDNSImpl.handleResponse(msg);
                        }
                    }
                } catch (IOException e) {
                    Logger logger3 = logger;
                    Level level = Level.WARNING;
                    logger3.log(level, getName() + ".run() exception ", e);
                }
            }
        } catch (IOException e2) {
            if (!this._jmDNSImpl.isCanceling() && !this._jmDNSImpl.isCanceled() && !this._jmDNSImpl.isClosing() && !this._jmDNSImpl.isClosed()) {
                Logger logger4 = logger;
                Level level2 = Level.WARNING;
                logger4.log(level2, getName() + ".run() exception ", e2);
                this._jmDNSImpl.recover();
            }
        }
        if (logger.isLoggable(Level.FINEST)) {
            Logger logger5 = logger;
            logger5.finest(getName() + ".run() exiting.");
        }
    }

    public JmDNSImpl getDns() {
        return this._jmDNSImpl;
    }
}
