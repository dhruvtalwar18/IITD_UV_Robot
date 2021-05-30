package org.apache.commons.io.output;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.io.UnsupportedEncodingException;
import java.util.ArrayList;
import java.util.List;

public class ByteArrayOutputStream extends OutputStream {
    private static final byte[] EMPTY_BYTE_ARRAY = new byte[0];
    private List buffers;
    private int count;
    private byte[] currentBuffer;
    private int currentBufferIndex;
    private int filledBufferSum;

    public ByteArrayOutputStream() {
        this(1024);
    }

    public ByteArrayOutputStream(int size) {
        this.buffers = new ArrayList();
        if (size >= 0) {
            needNewBuffer(size);
            return;
        }
        StringBuffer stringBuffer = new StringBuffer();
        stringBuffer.append("Negative initial size: ");
        stringBuffer.append(size);
        throw new IllegalArgumentException(stringBuffer.toString());
    }

    private byte[] getBuffer(int index) {
        return (byte[]) this.buffers.get(index);
    }

    private void needNewBuffer(int newcount) {
        int newBufferSize;
        if (this.currentBufferIndex < this.buffers.size() - 1) {
            this.filledBufferSum += this.currentBuffer.length;
            this.currentBufferIndex++;
            this.currentBuffer = getBuffer(this.currentBufferIndex);
            return;
        }
        if (this.currentBuffer == null) {
            newBufferSize = newcount;
            this.filledBufferSum = 0;
        } else {
            newBufferSize = Math.max(this.currentBuffer.length << 1, newcount - this.filledBufferSum);
            this.filledBufferSum += this.currentBuffer.length;
        }
        this.currentBufferIndex++;
        this.currentBuffer = new byte[newBufferSize];
        this.buffers.add(this.currentBuffer);
    }

    public void write(byte[] b, int off, int len) {
        if (off < 0 || off > b.length || len < 0 || off + len > b.length || off + len < 0) {
            throw new IndexOutOfBoundsException();
        } else if (len != 0) {
            synchronized (this) {
                int newcount = this.count + len;
                int remaining = len;
                int inBufferPos = this.count - this.filledBufferSum;
                while (remaining > 0) {
                    int part = Math.min(remaining, this.currentBuffer.length - inBufferPos);
                    System.arraycopy(b, (off + len) - remaining, this.currentBuffer, inBufferPos, part);
                    remaining -= part;
                    if (remaining > 0) {
                        needNewBuffer(newcount);
                        inBufferPos = 0;
                    }
                }
                this.count = newcount;
            }
        }
    }

    public synchronized void write(int b) {
        int inBufferPos = this.count - this.filledBufferSum;
        if (inBufferPos == this.currentBuffer.length) {
            needNewBuffer(this.count + 1);
            inBufferPos = 0;
        }
        this.currentBuffer[inBufferPos] = (byte) b;
        this.count++;
    }

    public synchronized int write(InputStream in) throws IOException {
        int readCount;
        readCount = 0;
        int inBufferPos = this.count - this.filledBufferSum;
        int n = in.read(this.currentBuffer, inBufferPos, this.currentBuffer.length - inBufferPos);
        while (n != -1) {
            readCount += n;
            inBufferPos += n;
            this.count += n;
            if (inBufferPos == this.currentBuffer.length) {
                needNewBuffer(this.currentBuffer.length);
                inBufferPos = 0;
            }
            n = in.read(this.currentBuffer, inBufferPos, this.currentBuffer.length - inBufferPos);
        }
        return readCount;
    }

    public synchronized int size() {
        return this.count;
    }

    public void close() throws IOException {
    }

    public synchronized void reset() {
        this.count = 0;
        this.filledBufferSum = 0;
        this.currentBufferIndex = 0;
        this.currentBuffer = getBuffer(this.currentBufferIndex);
    }

    public synchronized void writeTo(OutputStream out) throws IOException {
        int remaining = this.count;
        for (int i = 0; i < this.buffers.size(); i++) {
            byte[] buf = getBuffer(i);
            int c = Math.min(buf.length, remaining);
            out.write(buf, 0, c);
            remaining -= c;
            if (remaining == 0) {
                break;
            }
        }
    }

    /* JADX WARNING: Code restructure failed: missing block: B:16:0x002d, code lost:
        return r1;
     */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public synchronized byte[] toByteArray() {
        /*
            r7 = this;
            monitor-enter(r7)
            int r0 = r7.count     // Catch:{ all -> 0x002e }
            if (r0 != 0) goto L_0x0009
            byte[] r1 = EMPTY_BYTE_ARRAY     // Catch:{ all -> 0x002e }
            monitor-exit(r7)
            return r1
        L_0x0009:
            byte[] r1 = new byte[r0]     // Catch:{ all -> 0x002e }
            r2 = 0
            r3 = 0
            r4 = r2
            r2 = r0
            r0 = 0
        L_0x0010:
            java.util.List r5 = r7.buffers     // Catch:{ all -> 0x002e }
            int r5 = r5.size()     // Catch:{ all -> 0x002e }
            if (r0 >= r5) goto L_0x002c
            byte[] r5 = r7.getBuffer(r0)     // Catch:{ all -> 0x002e }
            int r6 = r5.length     // Catch:{ all -> 0x002e }
            int r6 = java.lang.Math.min(r6, r2)     // Catch:{ all -> 0x002e }
            java.lang.System.arraycopy(r5, r3, r1, r4, r6)     // Catch:{ all -> 0x002e }
            int r4 = r4 + r6
            int r2 = r2 - r6
            if (r2 != 0) goto L_0x0029
            goto L_0x002c
        L_0x0029:
            int r0 = r0 + 1
            goto L_0x0010
        L_0x002c:
            monitor-exit(r7)
            return r1
        L_0x002e:
            r0 = move-exception
            monitor-exit(r7)
            throw r0
        */
        throw new UnsupportedOperationException("Method not decompiled: org.apache.commons.io.output.ByteArrayOutputStream.toByteArray():byte[]");
    }

    public String toString() {
        return new String(toByteArray());
    }

    public String toString(String enc) throws UnsupportedEncodingException {
        return new String(toByteArray(), enc);
    }
}
