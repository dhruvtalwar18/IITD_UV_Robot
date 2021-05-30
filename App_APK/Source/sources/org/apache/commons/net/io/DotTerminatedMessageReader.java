package org.apache.commons.net.io;

import java.io.IOException;
import java.io.PushbackReader;
import java.io.Reader;

public final class DotTerminatedMessageReader extends Reader {
    private static final String LS = System.getProperty("line.separator");
    private static final char[] LS_CHARS = LS.toCharArray();
    private boolean atBeginning = true;
    private boolean eof = false;
    private char[] internalBuffer = new char[(LS_CHARS.length + 3)];
    private PushbackReader internalReader;
    private int pos = this.internalBuffer.length;

    public DotTerminatedMessageReader(Reader reader) {
        super(reader);
        this.internalReader = new PushbackReader(reader);
    }

    /* JADX DEBUG: Multi-variable search result rejected for TypeSearchVarInfo{r2v5, resolved type: int} */
    /* JADX DEBUG: Multi-variable search result rejected for TypeSearchVarInfo{r4v0, resolved type: char} */
    /* JADX WARNING: Incorrect type for immutable var: ssa=char, code=int, for r1v6, types: [char] */
    /* JADX WARNING: Multi-variable type inference failed */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public int read() throws java.io.IOException {
        /*
            r8 = this;
            java.lang.Object r0 = r8.lock
            monitor-enter(r0)
            r1 = 0
            int r2 = r8.pos     // Catch:{ all -> 0x00ba }
            char[] r3 = r8.internalBuffer     // Catch:{ all -> 0x00ba }
            int r3 = r3.length     // Catch:{ all -> 0x00ba }
            if (r2 >= r3) goto L_0x0017
            char[] r2 = r8.internalBuffer     // Catch:{ all -> 0x00ba }
            int r3 = r8.pos     // Catch:{ all -> 0x00ba }
            int r4 = r3 + 1
            r8.pos = r4     // Catch:{ all -> 0x00ba }
            char r2 = r2[r3]     // Catch:{ all -> 0x00ba }
            monitor-exit(r0)     // Catch:{ all -> 0x00ba }
            return r2
        L_0x0017:
            boolean r2 = r8.eof     // Catch:{ all -> 0x00ba }
            r3 = -1
            if (r2 == 0) goto L_0x001e
            monitor-exit(r0)     // Catch:{ all -> 0x00ba }
            return r3
        L_0x001e:
            java.io.PushbackReader r2 = r8.internalReader     // Catch:{ all -> 0x00ba }
            int r2 = r2.read()     // Catch:{ all -> 0x00ba }
            r4 = r2
            r5 = 1
            if (r2 != r3) goto L_0x0030
            r8.eof = r5     // Catch:{ all -> 0x002c }
            monitor-exit(r0)     // Catch:{ all -> 0x002c }
            return r3
        L_0x002c:
            r2 = move-exception
            r1 = r4
            goto L_0x00bb
        L_0x0030:
            boolean r2 = r8.atBeginning     // Catch:{ all -> 0x002c }
            r6 = 46
            if (r2 == 0) goto L_0x004d
            r8.atBeginning = r1     // Catch:{ all -> 0x002c }
            if (r4 != r6) goto L_0x004d
            java.io.PushbackReader r1 = r8.internalReader     // Catch:{ all -> 0x002c }
            int r1 = r1.read()     // Catch:{ all -> 0x002c }
            if (r1 == r6) goto L_0x004b
            r8.eof = r5     // Catch:{ all -> 0x00ba }
            java.io.PushbackReader r2 = r8.internalReader     // Catch:{ all -> 0x00ba }
            r2.read()     // Catch:{ all -> 0x00ba }
            monitor-exit(r0)     // Catch:{ all -> 0x00ba }
            return r3
        L_0x004b:
            monitor-exit(r0)     // Catch:{ all -> 0x00ba }
            return r6
        L_0x004d:
            r2 = 13
            if (r4 != r2) goto L_0x00b7
            java.io.PushbackReader r3 = r8.internalReader     // Catch:{ all -> 0x002c }
            int r3 = r3.read()     // Catch:{ all -> 0x002c }
            r4 = 10
            if (r3 != r4) goto L_0x00ab
            java.io.PushbackReader r2 = r8.internalReader     // Catch:{ all -> 0x00a8 }
            int r2 = r2.read()     // Catch:{ all -> 0x00a8 }
            if (r2 != r6) goto L_0x0084
            java.io.PushbackReader r3 = r8.internalReader     // Catch:{ all -> 0x007f }
            int r3 = r3.read()     // Catch:{ all -> 0x007f }
            r2 = r3
            if (r2 == r6) goto L_0x0074
            java.io.PushbackReader r3 = r8.internalReader     // Catch:{ all -> 0x007f }
            r3.read()     // Catch:{ all -> 0x007f }
            r8.eof = r5     // Catch:{ all -> 0x007f }
            goto L_0x0089
        L_0x0074:
            char[] r3 = r8.internalBuffer     // Catch:{ all -> 0x007f }
            int r4 = r8.pos     // Catch:{ all -> 0x007f }
            int r4 = r4 - r5
            r8.pos = r4     // Catch:{ all -> 0x007f }
            char r5 = (char) r2     // Catch:{ all -> 0x007f }
            r3[r4] = r5     // Catch:{ all -> 0x007f }
            goto L_0x0089
        L_0x007f:
            r1 = move-exception
            r7 = r2
            r2 = r1
            r1 = r7
            goto L_0x00bb
        L_0x0084:
            java.io.PushbackReader r3 = r8.internalReader     // Catch:{ all -> 0x007f }
            r3.unread(r2)     // Catch:{ all -> 0x007f }
        L_0x0089:
            int r3 = r8.pos     // Catch:{ all -> 0x007f }
            char[] r4 = LS_CHARS     // Catch:{ all -> 0x007f }
            int r4 = r4.length     // Catch:{ all -> 0x007f }
            int r3 = r3 - r4
            r8.pos = r3     // Catch:{ all -> 0x007f }
            char[] r3 = LS_CHARS     // Catch:{ all -> 0x007f }
            char[] r4 = r8.internalBuffer     // Catch:{ all -> 0x007f }
            int r5 = r8.pos     // Catch:{ all -> 0x007f }
            char[] r6 = LS_CHARS     // Catch:{ all -> 0x007f }
            int r6 = r6.length     // Catch:{ all -> 0x007f }
            java.lang.System.arraycopy(r3, r1, r4, r5, r6)     // Catch:{ all -> 0x007f }
            char[] r1 = r8.internalBuffer     // Catch:{ all -> 0x007f }
            int r3 = r8.pos     // Catch:{ all -> 0x007f }
            int r4 = r3 + 1
            r8.pos = r4     // Catch:{ all -> 0x007f }
            char r1 = r1[r3]     // Catch:{ all -> 0x007f }
            goto L_0x00b8
        L_0x00a8:
            r2 = move-exception
            r1 = r3
            goto L_0x00bb
        L_0x00ab:
            char[] r1 = r8.internalBuffer     // Catch:{ all -> 0x00a8 }
            int r4 = r8.pos     // Catch:{ all -> 0x00a8 }
            int r4 = r4 - r5
            r8.pos = r4     // Catch:{ all -> 0x00a8 }
            char r5 = (char) r3     // Catch:{ all -> 0x00a8 }
            r1[r4] = r5     // Catch:{ all -> 0x00a8 }
            monitor-exit(r0)     // Catch:{ all -> 0x00a8 }
            return r2
        L_0x00b7:
            r1 = r4
        L_0x00b8:
            monitor-exit(r0)     // Catch:{ all -> 0x00ba }
            return r1
        L_0x00ba:
            r2 = move-exception
        L_0x00bb:
            monitor-exit(r0)     // Catch:{ all -> 0x00ba }
            throw r2
        */
        throw new UnsupportedOperationException("Method not decompiled: org.apache.commons.net.io.DotTerminatedMessageReader.read():int");
    }

    public int read(char[] buffer) throws IOException {
        return read(buffer, 0, buffer.length);
    }

    public int read(char[] buffer, int offset, int length) throws IOException {
        int offset2;
        synchronized (this.lock) {
            if (length < 1) {
                try {
                    return 0;
                } catch (Throwable th) {
                    th = th;
                    Throwable th2 = th;
                    th = th2;
                    while (true) {
                        try {
                            break;
                        } catch (Throwable th3) {
                            th = th3;
                        }
                    }
                    throw th;
                }
            } else {
                int length2 = read();
                int ch = length2;
                if (length2 == -1) {
                    try {
                        return -1;
                    } catch (Throwable th4) {
                        th = th4;
                        int i = ch;
                        Throwable th22 = th;
                        th = th22;
                        while (true) {
                            break;
                        }
                        throw th;
                    }
                } else {
                    int length3 = length;
                    int ch2 = ch;
                    int length4 = offset;
                    while (true) {
                        offset2 = length4 + 1;
                        try {
                            buffer[length4] = (char) ch2;
                            length3--;
                            if (length3 <= 0) {
                                break;
                            }
                            int read = read();
                            ch2 = read;
                            if (read == -1) {
                                break;
                            }
                            length4 = offset2;
                        } catch (Throwable th5) {
                            th = th5;
                            int i2 = offset2;
                            int off = i2;
                            while (true) {
                                break;
                            }
                            throw th;
                        }
                    }
                    int i3 = offset2 - offset;
                    return i3;
                }
            }
        }
    }

    public boolean ready() throws IOException {
        boolean z;
        synchronized (this.lock) {
            if (this.pos >= this.internalBuffer.length) {
                if (!this.internalReader.ready()) {
                    z = false;
                }
            }
            z = true;
        }
        return z;
    }

    public void close() throws IOException {
        synchronized (this.lock) {
            if (this.internalReader != null) {
                if (!this.eof) {
                    while (read() != -1) {
                    }
                }
                this.eof = true;
                this.atBeginning = false;
                this.pos = this.internalBuffer.length;
                this.internalReader = null;
            }
        }
    }
}
