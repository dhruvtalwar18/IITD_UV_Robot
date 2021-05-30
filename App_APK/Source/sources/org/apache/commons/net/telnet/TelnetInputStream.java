package org.apache.commons.net.telnet;

import java.io.BufferedInputStream;
import java.io.IOException;
import java.io.InputStream;

final class TelnetInputStream extends BufferedInputStream implements Runnable {
    static final int _STATE_CR = 8;
    static final int _STATE_DATA = 0;
    static final int _STATE_DO = 4;
    static final int _STATE_DONT = 5;
    static final int _STATE_IAC = 1;
    static final int _STATE_IAC_SB = 9;
    static final int _STATE_SB = 6;
    static final int _STATE_SE = 7;
    static final int _STATE_WILL = 2;
    static final int _STATE_WONT = 3;
    private int __bytesAvailable;
    private TelnetClient __client;
    private boolean __hasReachedEOF;
    private IOException __ioException;
    private boolean __isClosed;
    private int[] __queue;
    private int __queueHead;
    private int __queueTail;
    private boolean __readIsWaiting;
    private int __receiveState;
    private int[] __suboption;
    private int __suboption_count;
    private Thread __thread;
    private boolean __threaded;

    TelnetInputStream(InputStream input, TelnetClient client, boolean readerThread) {
        super(input);
        this.__suboption = new int[256];
        this.__suboption_count = 0;
        this.__client = client;
        this.__receiveState = 0;
        this.__isClosed = true;
        this.__hasReachedEOF = false;
        this.__queue = new int[2049];
        this.__queueHead = 0;
        this.__queueTail = 0;
        this.__bytesAvailable = 0;
        this.__ioException = null;
        this.__readIsWaiting = false;
        this.__threaded = false;
        if (readerThread) {
            this.__thread = new Thread(this);
        } else {
            this.__thread = null;
        }
    }

    TelnetInputStream(InputStream input, TelnetClient client) {
        this(input, client, true);
    }

    /* access modifiers changed from: package-private */
    public void _start() {
        if (this.__thread != null) {
            this.__isClosed = false;
            int priority = Thread.currentThread().getPriority() + 1;
            if (priority > 10) {
                priority = 10;
            }
            this.__thread.setPriority(priority);
            this.__thread.setDaemon(true);
            this.__thread.start();
            this.__threaded = true;
        }
    }

    /* JADX WARNING: Can't fix incorrect switch cases order */
    /* JADX WARNING: Code restructure failed: missing block: B:96:0x0104, code lost:
        return r0;
     */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    private int __read(boolean r7) throws java.io.IOException {
        /*
            r6 = this;
        L_0x0000:
            if (r7 != 0) goto L_0x000a
            int r0 = super.available()
            if (r0 != 0) goto L_0x000a
            r0 = -2
            return r0
        L_0x000a:
            int r0 = super.read()
            r1 = r0
            if (r0 >= 0) goto L_0x0013
            r0 = -1
            return r0
        L_0x0013:
            r0 = r1 & 255(0xff, float:3.57E-43)
            org.apache.commons.net.telnet.TelnetClient r1 = r6.__client
            monitor-enter(r1)
            org.apache.commons.net.telnet.TelnetClient r2 = r6.__client     // Catch:{ all -> 0x0105 }
            r2._processAYTResponse()     // Catch:{ all -> 0x0105 }
            monitor-exit(r1)     // Catch:{ all -> 0x0105 }
            org.apache.commons.net.telnet.TelnetClient r1 = r6.__client
            r1._spyRead(r0)
            int r1 = r6.__receiveState
            r2 = 6
            r3 = 255(0xff, float:3.57E-43)
            r4 = 0
            switch(r1) {
                case 0: goto L_0x00de;
                case 1: goto L_0x00b9;
                case 2: goto L_0x00a4;
                case 3: goto L_0x008f;
                case 4: goto L_0x007a;
                case 5: goto L_0x0066;
                case 6: goto L_0x0052;
                case 7: goto L_0x002c;
                case 8: goto L_0x004f;
                case 9: goto L_0x002e;
                default: goto L_0x002c;
            }
        L_0x002c:
            goto L_0x0103
        L_0x002e:
            r1 = 240(0xf0, float:3.36E-43)
            if (r0 == r1) goto L_0x0037
            r6.__receiveState = r2
            r6.__receiveState = r4
            goto L_0x0000
        L_0x0037:
            org.apache.commons.net.telnet.TelnetClient r1 = r6.__client
            monitor-enter(r1)
            org.apache.commons.net.telnet.TelnetClient r2 = r6.__client     // Catch:{ all -> 0x004c }
            int[] r3 = r6.__suboption     // Catch:{ all -> 0x004c }
            int r5 = r6.__suboption_count     // Catch:{ all -> 0x004c }
            r2._processSuboption(r3, r5)     // Catch:{ all -> 0x004c }
            org.apache.commons.net.telnet.TelnetClient r2 = r6.__client     // Catch:{ all -> 0x004c }
            r2._flushOutputStream()     // Catch:{ all -> 0x004c }
            monitor-exit(r1)     // Catch:{ all -> 0x004c }
            r6.__receiveState = r4
            goto L_0x0000
        L_0x004c:
            r2 = move-exception
            monitor-exit(r1)     // Catch:{ all -> 0x004c }
            throw r2
        L_0x004f:
            if (r0 != 0) goto L_0x00de
            goto L_0x0000
        L_0x0052:
            if (r0 == r3) goto L_0x0061
            int[] r1 = r6.__suboption
            int r3 = r6.__suboption_count
            int r4 = r3 + 1
            r6.__suboption_count = r4
            r1[r3] = r0
            r6.__receiveState = r2
            goto L_0x0000
        L_0x0061:
            r1 = 9
            r6.__receiveState = r1
            goto L_0x0000
        L_0x0066:
            org.apache.commons.net.telnet.TelnetClient r1 = r6.__client
            monitor-enter(r1)
            org.apache.commons.net.telnet.TelnetClient r2 = r6.__client     // Catch:{ all -> 0x0077 }
            r2._processDont(r0)     // Catch:{ all -> 0x0077 }
            org.apache.commons.net.telnet.TelnetClient r2 = r6.__client     // Catch:{ all -> 0x0077 }
            r2._flushOutputStream()     // Catch:{ all -> 0x0077 }
            monitor-exit(r1)     // Catch:{ all -> 0x0077 }
            r6.__receiveState = r4
            goto L_0x0000
        L_0x0077:
            r2 = move-exception
            monitor-exit(r1)     // Catch:{ all -> 0x0077 }
            throw r2
        L_0x007a:
            org.apache.commons.net.telnet.TelnetClient r1 = r6.__client
            monitor-enter(r1)
            org.apache.commons.net.telnet.TelnetClient r2 = r6.__client     // Catch:{ all -> 0x008c }
            r2._processDo(r0)     // Catch:{ all -> 0x008c }
            org.apache.commons.net.telnet.TelnetClient r2 = r6.__client     // Catch:{ all -> 0x008c }
            r2._flushOutputStream()     // Catch:{ all -> 0x008c }
            monitor-exit(r1)     // Catch:{ all -> 0x008c }
            r6.__receiveState = r4
            goto L_0x0000
        L_0x008c:
            r2 = move-exception
            monitor-exit(r1)     // Catch:{ all -> 0x008c }
            throw r2
        L_0x008f:
            org.apache.commons.net.telnet.TelnetClient r1 = r6.__client
            monitor-enter(r1)
            org.apache.commons.net.telnet.TelnetClient r2 = r6.__client     // Catch:{ all -> 0x00a1 }
            r2._processWont(r0)     // Catch:{ all -> 0x00a1 }
            org.apache.commons.net.telnet.TelnetClient r2 = r6.__client     // Catch:{ all -> 0x00a1 }
            r2._flushOutputStream()     // Catch:{ all -> 0x00a1 }
            monitor-exit(r1)     // Catch:{ all -> 0x00a1 }
            r6.__receiveState = r4
            goto L_0x0000
        L_0x00a1:
            r2 = move-exception
            monitor-exit(r1)     // Catch:{ all -> 0x00a1 }
            throw r2
        L_0x00a4:
            org.apache.commons.net.telnet.TelnetClient r1 = r6.__client
            monitor-enter(r1)
            org.apache.commons.net.telnet.TelnetClient r2 = r6.__client     // Catch:{ all -> 0x00b6 }
            r2._processWill(r0)     // Catch:{ all -> 0x00b6 }
            org.apache.commons.net.telnet.TelnetClient r2 = r6.__client     // Catch:{ all -> 0x00b6 }
            r2._flushOutputStream()     // Catch:{ all -> 0x00b6 }
            monitor-exit(r1)     // Catch:{ all -> 0x00b6 }
            r6.__receiveState = r4
            goto L_0x0000
        L_0x00b6:
            r2 = move-exception
            monitor-exit(r1)     // Catch:{ all -> 0x00b6 }
            throw r2
        L_0x00b9:
            switch(r0) {
                case 250: goto L_0x00d4;
                case 251: goto L_0x00cf;
                case 252: goto L_0x00ca;
                case 253: goto L_0x00c5;
                case 254: goto L_0x00c0;
                case 255: goto L_0x00bd;
                default: goto L_0x00bc;
            }
        L_0x00bc:
            goto L_0x00da
        L_0x00bd:
            r6.__receiveState = r4
            goto L_0x00da
        L_0x00c0:
            r1 = 5
            r6.__receiveState = r1
            goto L_0x0000
        L_0x00c5:
            r1 = 4
            r6.__receiveState = r1
            goto L_0x0000
        L_0x00ca:
            r1 = 3
            r6.__receiveState = r1
            goto L_0x0000
        L_0x00cf:
            r1 = 2
            r6.__receiveState = r1
            goto L_0x0000
        L_0x00d4:
            r6.__suboption_count = r4
            r6.__receiveState = r2
            goto L_0x0000
        L_0x00da:
            r6.__receiveState = r4
            goto L_0x0000
        L_0x00de:
            if (r0 != r3) goto L_0x00e5
            r1 = 1
            r6.__receiveState = r1
            goto L_0x0000
        L_0x00e5:
            r1 = 13
            if (r0 != r1) goto L_0x0100
            org.apache.commons.net.telnet.TelnetClient r1 = r6.__client
            monitor-enter(r1)
            org.apache.commons.net.telnet.TelnetClient r2 = r6.__client     // Catch:{ all -> 0x00fd }
            boolean r2 = r2._requestedDont(r4)     // Catch:{ all -> 0x00fd }
            if (r2 == 0) goto L_0x00f9
            r2 = 8
            r6.__receiveState = r2     // Catch:{ all -> 0x00fd }
            goto L_0x00fb
        L_0x00f9:
            r6.__receiveState = r4     // Catch:{ all -> 0x00fd }
        L_0x00fb:
            monitor-exit(r1)     // Catch:{ all -> 0x00fd }
            goto L_0x0103
        L_0x00fd:
            r2 = move-exception
            monitor-exit(r1)     // Catch:{ all -> 0x00fd }
            throw r2
        L_0x0100:
            r6.__receiveState = r4
        L_0x0103:
            return r0
        L_0x0105:
            r2 = move-exception
            monitor-exit(r1)     // Catch:{ all -> 0x0105 }
            throw r2
        */
        throw new UnsupportedOperationException("Method not decompiled: org.apache.commons.net.telnet.TelnetInputStream.__read(boolean):int");
    }

    private void __processChar(int ch) throws InterruptedException {
        synchronized (this.__queue) {
            while (this.__bytesAvailable >= this.__queue.length - 1) {
                if (this.__threaded) {
                    this.__queue.notify();
                    try {
                        this.__queue.wait();
                    } catch (InterruptedException e) {
                        throw e;
                    }
                } else {
                    throw new IllegalStateException("Queue is full! Cannot process another character.");
                }
            }
            if (this.__readIsWaiting && this.__threaded) {
                this.__queue.notify();
            }
            this.__queue[this.__queueTail] = ch;
            this.__bytesAvailable++;
            int i = this.__queueTail + 1;
            this.__queueTail = i;
            if (i >= this.__queue.length) {
                this.__queueTail = 0;
            }
        }
    }

    /* JADX WARNING: Code restructure failed: missing block: B:72:0x00a3, code lost:
        return r2;
     */
    /* JADX WARNING: Removed duplicated region for block: B:32:0x0044 A[SYNTHETIC, Splitter:B:32:0x0044] */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public int read() throws java.io.IOException {
        /*
            r9 = this;
            int[] r0 = r9.__queue
            monitor-enter(r0)
            r1 = 0
            r2 = 0
        L_0x0005:
            java.io.IOException r3 = r9.__ioException     // Catch:{ all -> 0x00aa }
            if (r3 != 0) goto L_0x00a4
            int r3 = r9.__bytesAvailable     // Catch:{ all -> 0x00aa }
            r4 = 1
            if (r3 != 0) goto L_0x007e
            boolean r3 = r9.__hasReachedEOF     // Catch:{ all -> 0x00aa }
            r5 = -1
            if (r3 == 0) goto L_0x0015
            monitor-exit(r0)     // Catch:{ all -> 0x00aa }
            return r5
        L_0x0015:
            boolean r3 = r9.__threaded     // Catch:{ all -> 0x00aa }
            if (r3 == 0) goto L_0x0031
            int[] r3 = r9.__queue     // Catch:{ all -> 0x00aa }
            r3.notify()     // Catch:{ all -> 0x00aa }
            r9.__readIsWaiting = r4     // Catch:{ InterruptedException -> 0x0028 }
            int[] r3 = r9.__queue     // Catch:{ InterruptedException -> 0x0028 }
            r3.wait()     // Catch:{ InterruptedException -> 0x0028 }
            r9.__readIsWaiting = r1     // Catch:{ InterruptedException -> 0x0028 }
            goto L_0x0005
        L_0x0028:
            r1 = move-exception
            java.io.InterruptedIOException r2 = new java.io.InterruptedIOException     // Catch:{ all -> 0x00aa }
            java.lang.String r3 = "Fatal thread interruption during read."
            r2.<init>(r3)     // Catch:{ all -> 0x00aa }
            throw r2     // Catch:{ all -> 0x00aa }
        L_0x0031:
            r9.__readIsWaiting = r4     // Catch:{ all -> 0x00aa }
            r3 = r2
            r2 = 1
        L_0x0035:
            int r6 = r9.__read(r2)     // Catch:{ InterruptedIOException -> 0x0064 }
            r3 = r6
            r7 = -2
            if (r6 >= 0) goto L_0x0041
            if (r3 == r7) goto L_0x0041
            monitor-exit(r0)     // Catch:{ all -> 0x00aa }
            return r3
        L_0x0041:
            if (r3 == r7) goto L_0x004f
            r9.__processChar(r3)     // Catch:{ InterruptedException -> 0x0048 }
            goto L_0x004f
        L_0x0048:
            r6 = move-exception
            boolean r7 = r9.__isClosed     // Catch:{ all -> 0x00aa }
            if (r7 == 0) goto L_0x0050
            monitor-exit(r0)     // Catch:{ all -> 0x00aa }
            return r5
        L_0x004f:
        L_0x0050:
            r2 = 0
            int r6 = super.available()     // Catch:{ all -> 0x00aa }
            if (r6 <= 0) goto L_0x005f
            int r6 = r9.__bytesAvailable     // Catch:{ all -> 0x00aa }
            int[] r7 = r9.__queue     // Catch:{ all -> 0x00aa }
            int r7 = r7.length     // Catch:{ all -> 0x00aa }
            int r7 = r7 - r4
            if (r6 < r7) goto L_0x0035
        L_0x005f:
            r9.__readIsWaiting = r1     // Catch:{ all -> 0x00aa }
            r2 = r3
            goto L_0x0005
        L_0x0064:
            r1 = move-exception
            int[] r4 = r9.__queue     // Catch:{ all -> 0x00aa }
            monitor-enter(r4)     // Catch:{ all -> 0x00aa }
            r9.__ioException = r1     // Catch:{ all -> 0x007b }
            int[] r6 = r9.__queue     // Catch:{ all -> 0x007b }
            r6.notifyAll()     // Catch:{ all -> 0x007b }
            int[] r6 = r9.__queue     // Catch:{ InterruptedException -> 0x0077 }
            r7 = 100
            r6.wait(r7)     // Catch:{ InterruptedException -> 0x0077 }
            goto L_0x0078
        L_0x0077:
            r6 = move-exception
        L_0x0078:
            monitor-exit(r4)     // Catch:{ all -> 0x007b }
            monitor-exit(r0)     // Catch:{ all -> 0x00aa }
            return r5
        L_0x007b:
            r5 = move-exception
            monitor-exit(r4)     // Catch:{ all -> 0x007b }
            throw r5     // Catch:{ all -> 0x00aa }
        L_0x007e:
            int[] r2 = r9.__queue     // Catch:{ all -> 0x00aa }
            int r3 = r9.__queueHead     // Catch:{ all -> 0x00aa }
            r2 = r2[r3]     // Catch:{ all -> 0x00aa }
            int r3 = r9.__queueHead     // Catch:{ all -> 0x00aa }
            int r3 = r3 + r4
            r9.__queueHead = r3     // Catch:{ all -> 0x00aa }
            int[] r5 = r9.__queue     // Catch:{ all -> 0x00aa }
            int r5 = r5.length     // Catch:{ all -> 0x00aa }
            if (r3 < r5) goto L_0x0090
            r9.__queueHead = r1     // Catch:{ all -> 0x00aa }
        L_0x0090:
            int r1 = r9.__bytesAvailable     // Catch:{ all -> 0x00aa }
            int r1 = r1 - r4
            r9.__bytesAvailable = r1     // Catch:{ all -> 0x00aa }
            int r1 = r9.__bytesAvailable     // Catch:{ all -> 0x00aa }
            if (r1 != 0) goto L_0x00a2
            boolean r1 = r9.__threaded     // Catch:{ all -> 0x00aa }
            if (r1 == 0) goto L_0x00a2
            int[] r1 = r9.__queue     // Catch:{ all -> 0x00aa }
            r1.notify()     // Catch:{ all -> 0x00aa }
        L_0x00a2:
            monitor-exit(r0)     // Catch:{ all -> 0x00aa }
            return r2
        L_0x00a4:
            java.io.IOException r1 = r9.__ioException     // Catch:{ all -> 0x00aa }
            r2 = 0
            r9.__ioException = r2     // Catch:{ all -> 0x00aa }
            throw r1     // Catch:{ all -> 0x00aa }
        L_0x00aa:
            r1 = move-exception
            monitor-exit(r0)     // Catch:{ all -> 0x00aa }
            throw r1
        */
        throw new UnsupportedOperationException("Method not decompiled: org.apache.commons.net.telnet.TelnetInputStream.read():int");
    }

    public int read(byte[] buffer) throws IOException {
        return read(buffer, 0, buffer.length);
    }

    public int read(byte[] buffer, int offset, int length) throws IOException {
        int offset2;
        if (length < 1) {
            return 0;
        }
        synchronized (this.__queue) {
            if (length > this.__bytesAvailable) {
                length = this.__bytesAvailable;
            }
        }
        int read = read();
        int ch = read;
        if (read == -1) {
            return -1;
        }
        int length2 = length;
        int length3 = offset;
        while (true) {
            offset2 = length3 + 1;
            buffer[length3] = (byte) ch;
            length2--;
            if (length2 <= 0) {
                break;
            }
            int read2 = read();
            ch = read2;
            if (read2 == -1) {
                break;
            }
            length3 = offset2;
        }
        return offset2 - offset;
    }

    public boolean markSupported() {
        return false;
    }

    public int available() throws IOException {
        int i;
        synchronized (this.__queue) {
            i = this.__bytesAvailable;
        }
        return i;
    }

    public void close() throws IOException {
        super.close();
        synchronized (this.__queue) {
            this.__hasReachedEOF = true;
            this.__isClosed = true;
            if (this.__thread != null && this.__thread.isAlive()) {
                this.__thread.interrupt();
            }
            this.__queue.notifyAll();
        }
        this.__threaded = false;
    }

    /*  JADX ERROR: IndexOutOfBoundsException in pass: RegionMakerVisitor
        java.lang.IndexOutOfBoundsException: Index: 0, Size: 0
        	at java.util.ArrayList.rangeCheck(ArrayList.java:659)
        	at java.util.ArrayList.get(ArrayList.java:435)
        	at jadx.core.dex.nodes.InsnNode.getArg(InsnNode.java:101)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverseMonitorExits(RegionMaker.java:611)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverseMonitorExits(RegionMaker.java:619)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverseMonitorExits(RegionMaker.java:619)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverseMonitorExits(RegionMaker.java:619)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverseMonitorExits(RegionMaker.java:619)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverseMonitorExits(RegionMaker.java:619)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverseMonitorExits(RegionMaker.java:619)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverseMonitorExits(RegionMaker.java:619)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverseMonitorExits(RegionMaker.java:619)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverseMonitorExits(RegionMaker.java:619)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverseMonitorExits(RegionMaker.java:619)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverseMonitorExits(RegionMaker.java:619)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverseMonitorExits(RegionMaker.java:619)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverseMonitorExits(RegionMaker.java:619)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverseMonitorExits(RegionMaker.java:619)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverseMonitorExits(RegionMaker.java:619)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverseMonitorExits(RegionMaker.java:619)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverseMonitorExits(RegionMaker.java:619)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverseMonitorExits(RegionMaker.java:619)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverseMonitorExits(RegionMaker.java:619)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverseMonitorExits(RegionMaker.java:619)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverseMonitorExits(RegionMaker.java:619)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverseMonitorExits(RegionMaker.java:619)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverseMonitorExits(RegionMaker.java:619)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverseMonitorExits(RegionMaker.java:619)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverseMonitorExits(RegionMaker.java:619)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverseMonitorExits(RegionMaker.java:619)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverseMonitorExits(RegionMaker.java:619)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverseMonitorExits(RegionMaker.java:619)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverseMonitorExits(RegionMaker.java:619)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverseMonitorExits(RegionMaker.java:619)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverseMonitorExits(RegionMaker.java:619)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverseMonitorExits(RegionMaker.java:619)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverseMonitorExits(RegionMaker.java:619)
        	at jadx.core.dex.visitors.regions.RegionMaker.processMonitorEnter(RegionMaker.java:561)
        	at jadx.core.dex.visitors.regions.RegionMaker.traverse(RegionMaker.java:133)
        	at jadx.core.dex.visitors.regions.RegionMaker.makeRegion(RegionMaker.java:86)
        	at jadx.core.dex.visitors.regions.RegionMaker.processExcHandler(RegionMaker.java:1043)
        	at jadx.core.dex.visitors.regions.RegionMaker.processTryCatchBlocks(RegionMaker.java:975)
        	at jadx.core.dex.visitors.regions.RegionMakerVisitor.visit(RegionMakerVisitor.java:52)
        */
    public void run() {
        /*
            r8 = this;
            r0 = 0
            r1 = 0
        L_0x0002:
            r2 = 1
            boolean r3 = r8.__isClosed     // Catch:{ IOException -> 0x003f }
            if (r3 != 0) goto L_0x003e
            int r3 = r8.__read(r2)     // Catch:{ InterruptedIOException -> 0x001f, RuntimeException -> 0x001a }
            r1 = r3
            if (r3 >= 0) goto L_0x000f
            goto L_0x003e
        L_0x000f:
            r8.__processChar(r1)     // Catch:{ InterruptedException -> 0x0014 }
        L_0x0013:
            goto L_0x0002
        L_0x0014:
            r3 = move-exception
            boolean r4 = r8.__isClosed     // Catch:{ IOException -> 0x003f }
            if (r4 == 0) goto L_0x0013
            goto L_0x003e
        L_0x001a:
            r3 = move-exception
            super.close()     // Catch:{ IOException -> 0x003f }
            goto L_0x003e
        L_0x001f:
            r3 = move-exception
            int[] r4 = r8.__queue     // Catch:{ IOException -> 0x003f }
            monitor-enter(r4)     // Catch:{ IOException -> 0x003f }
            r8.__ioException = r3     // Catch:{ all -> 0x003b }
            int[] r5 = r8.__queue     // Catch:{ all -> 0x003b }
            r5.notifyAll()     // Catch:{ all -> 0x003b }
            int[] r5 = r8.__queue     // Catch:{ InterruptedException -> 0x0032 }
            r6 = 100
            r5.wait(r6)     // Catch:{ InterruptedException -> 0x0032 }
            goto L_0x0039
        L_0x0032:
            r5 = move-exception
            boolean r6 = r8.__isClosed     // Catch:{ all -> 0x003b }
            if (r6 == 0) goto L_0x0039
            monitor-exit(r4)     // Catch:{ all -> 0x003b }
            goto L_0x003e
        L_0x0039:
            monitor-exit(r4)     // Catch:{ all -> 0x003b }
            goto L_0x0002
        L_0x003b:
            r5 = move-exception
            monitor-exit(r4)     // Catch:{ all -> 0x003b }
            throw r5     // Catch:{ IOException -> 0x003f }
        L_0x003e:
            goto L_0x0046
        L_0x003f:
            r1 = move-exception
            int[] r3 = r8.__queue
            monitor-enter(r3)
            r8.__ioException = r1     // Catch:{ all -> 0x0059 }
            monitor-exit(r3)     // Catch:{ all -> 0x0059 }
        L_0x0046:
            int[] r4 = r8.__queue
            monitor-enter(r4)
            r8.__isClosed = r2     // Catch:{ all -> 0x0056 }
            r8.__hasReachedEOF = r2     // Catch:{ all -> 0x0056 }
            int[] r1 = r8.__queue     // Catch:{ all -> 0x0056 }
            r1.notify()     // Catch:{ all -> 0x0056 }
            monitor-exit(r4)     // Catch:{ all -> 0x0056 }
            r8.__threaded = r0
            return
        L_0x0056:
            r0 = move-exception
            monitor-exit(r4)     // Catch:{ all -> 0x0056 }
            throw r0
        L_0x0059:
            r0 = move-exception
            monitor-exit(r3)     // Catch:{ all -> 0x0059 }
            throw r0
        */
        throw new UnsupportedOperationException("Method not decompiled: org.apache.commons.net.telnet.TelnetInputStream.run():void");
    }
}
