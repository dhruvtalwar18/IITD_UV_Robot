package org.apache.commons.net.telnet;

import java.io.IOException;
import java.io.OutputStream;

final class TelnetOutputStream extends OutputStream {
    private TelnetClient __client;
    private boolean __convertCRtoCRLF = true;
    private boolean __lastWasCR = false;

    TelnetOutputStream(TelnetClient client) {
        this.__client = client;
    }

    /* JADX WARNING: Code restructure failed: missing block: B:27:0x0060, code lost:
        return;
     */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public void write(int r6) throws java.io.IOException {
        /*
            r5 = this;
            org.apache.commons.net.telnet.TelnetClient r0 = r5.__client
            monitor-enter(r0)
            r1 = 255(0xff, float:3.57E-43)
            r6 = r6 & r1
            org.apache.commons.net.telnet.TelnetClient r2 = r5.__client     // Catch:{ all -> 0x0061 }
            r3 = 0
            boolean r2 = r2._requestedWont(r3)     // Catch:{ all -> 0x0061 }
            if (r2 == 0) goto L_0x004d
            boolean r2 = r5.__lastWasCR     // Catch:{ all -> 0x0061 }
            if (r2 == 0) goto L_0x002b
            boolean r2 = r5.__convertCRtoCRLF     // Catch:{ all -> 0x0061 }
            r4 = 10
            if (r2 == 0) goto L_0x0024
            org.apache.commons.net.telnet.TelnetClient r2 = r5.__client     // Catch:{ all -> 0x0061 }
            r2._sendByte(r4)     // Catch:{ all -> 0x0061 }
            if (r6 != r4) goto L_0x002b
            r5.__lastWasCR = r3     // Catch:{ all -> 0x0061 }
            monitor-exit(r0)     // Catch:{ all -> 0x0061 }
            return
        L_0x0024:
            if (r6 == r4) goto L_0x002b
            org.apache.commons.net.telnet.TelnetClient r2 = r5.__client     // Catch:{ all -> 0x0061 }
            r2._sendByte(r3)     // Catch:{ all -> 0x0061 }
        L_0x002b:
            r5.__lastWasCR = r3     // Catch:{ all -> 0x0061 }
            r2 = 13
            if (r6 == r2) goto L_0x0044
            if (r6 == r1) goto L_0x0039
            org.apache.commons.net.telnet.TelnetClient r1 = r5.__client     // Catch:{ all -> 0x0061 }
            r1._sendByte(r6)     // Catch:{ all -> 0x0061 }
            goto L_0x005f
        L_0x0039:
            org.apache.commons.net.telnet.TelnetClient r2 = r5.__client     // Catch:{ all -> 0x0061 }
            r2._sendByte(r1)     // Catch:{ all -> 0x0061 }
            org.apache.commons.net.telnet.TelnetClient r2 = r5.__client     // Catch:{ all -> 0x0061 }
            r2._sendByte(r1)     // Catch:{ all -> 0x0061 }
            goto L_0x005f
        L_0x0044:
            org.apache.commons.net.telnet.TelnetClient r1 = r5.__client     // Catch:{ all -> 0x0061 }
            r1._sendByte(r2)     // Catch:{ all -> 0x0061 }
            r1 = 1
            r5.__lastWasCR = r1     // Catch:{ all -> 0x0061 }
            goto L_0x005f
        L_0x004d:
            if (r6 != r1) goto L_0x005a
            org.apache.commons.net.telnet.TelnetClient r2 = r5.__client     // Catch:{ all -> 0x0061 }
            r2._sendByte(r6)     // Catch:{ all -> 0x0061 }
            org.apache.commons.net.telnet.TelnetClient r2 = r5.__client     // Catch:{ all -> 0x0061 }
            r2._sendByte(r1)     // Catch:{ all -> 0x0061 }
            goto L_0x005f
        L_0x005a:
            org.apache.commons.net.telnet.TelnetClient r1 = r5.__client     // Catch:{ all -> 0x0061 }
            r1._sendByte(r6)     // Catch:{ all -> 0x0061 }
        L_0x005f:
            monitor-exit(r0)     // Catch:{ all -> 0x0061 }
            return
        L_0x0061:
            r1 = move-exception
            monitor-exit(r0)     // Catch:{ all -> 0x0061 }
            throw r1
        */
        throw new UnsupportedOperationException("Method not decompiled: org.apache.commons.net.telnet.TelnetOutputStream.write(int):void");
    }

    public void write(byte[] buffer) throws IOException {
        write(buffer, 0, buffer.length);
    }

    public void write(byte[] buffer, int offset, int offset2) throws IOException {
        Throwable th;
        synchronized (this.__client) {
            while (true) {
                int length = offset2 - 1;
                if (offset2 > 0) {
                    int offset3 = offset + 1;
                    try {
                        write((int) buffer[offset]);
                        offset = offset3;
                        offset2 = length;
                    } catch (Throwable th2) {
                        th = th2;
                        throw th;
                    }
                } else {
                    try {
                        return;
                    } catch (Throwable th3) {
                        Throwable th4 = th3;
                        int i = offset;
                        th = th4;
                        throw th;
                    }
                }
            }
        }
    }

    public void flush() throws IOException {
        this.__client._flushOutputStream();
    }

    public void close() throws IOException {
        this.__client._closeOutputStream();
    }
}
