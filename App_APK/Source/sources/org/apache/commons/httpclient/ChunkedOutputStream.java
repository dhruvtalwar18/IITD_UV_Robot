package org.apache.commons.httpclient;

import java.io.IOException;
import java.io.OutputStream;
import org.apache.commons.httpclient.util.EncodingUtil;

public class ChunkedOutputStream extends OutputStream {
    private static final byte[] CRLF = {13, 10};
    private static final byte[] ENDCHUNK = CRLF;
    private static final byte[] ZERO = {48};
    private byte[] cache;
    private int cachePosition;
    private OutputStream stream;
    private boolean wroteLastChunk;

    public ChunkedOutputStream(OutputStream stream2, int bufferSize) throws IOException {
        this.stream = null;
        this.cachePosition = 0;
        this.wroteLastChunk = false;
        this.cache = new byte[bufferSize];
        this.stream = stream2;
    }

    public ChunkedOutputStream(OutputStream stream2) throws IOException {
        this(stream2, 2048);
    }

    /* access modifiers changed from: protected */
    public void flushCache() throws IOException {
        if (this.cachePosition > 0) {
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append(Integer.toHexString(this.cachePosition));
            stringBuffer.append("\r\n");
            byte[] chunkHeader = EncodingUtil.getAsciiBytes(stringBuffer.toString());
            this.stream.write(chunkHeader, 0, chunkHeader.length);
            this.stream.write(this.cache, 0, this.cachePosition);
            this.stream.write(ENDCHUNK, 0, ENDCHUNK.length);
            this.cachePosition = 0;
        }
    }

    /* access modifiers changed from: protected */
    public void flushCacheWithAppend(byte[] bufferToAppend, int off, int len) throws IOException {
        StringBuffer stringBuffer = new StringBuffer();
        stringBuffer.append(Integer.toHexString(this.cachePosition + len));
        stringBuffer.append("\r\n");
        byte[] chunkHeader = EncodingUtil.getAsciiBytes(stringBuffer.toString());
        this.stream.write(chunkHeader, 0, chunkHeader.length);
        this.stream.write(this.cache, 0, this.cachePosition);
        this.stream.write(bufferToAppend, off, len);
        this.stream.write(ENDCHUNK, 0, ENDCHUNK.length);
        this.cachePosition = 0;
    }

    /* access modifiers changed from: protected */
    public void writeClosingChunk() throws IOException {
        this.stream.write(ZERO, 0, ZERO.length);
        this.stream.write(CRLF, 0, CRLF.length);
        this.stream.write(ENDCHUNK, 0, ENDCHUNK.length);
    }

    public void finish() throws IOException {
        if (!this.wroteLastChunk) {
            flushCache();
            writeClosingChunk();
            this.wroteLastChunk = true;
        }
    }

    public void write(int b) throws IOException {
        this.cache[this.cachePosition] = (byte) b;
        this.cachePosition++;
        if (this.cachePosition == this.cache.length) {
            flushCache();
        }
    }

    public void write(byte[] b) throws IOException {
        write(b, 0, b.length);
    }

    public void write(byte[] src, int off, int len) throws IOException {
        if (len >= this.cache.length - this.cachePosition) {
            flushCacheWithAppend(src, off, len);
            return;
        }
        System.arraycopy(src, off, this.cache, this.cachePosition, len);
        this.cachePosition += len;
    }

    public void flush() throws IOException {
        this.stream.flush();
    }

    public void close() throws IOException {
        finish();
        super.close();
    }
}
