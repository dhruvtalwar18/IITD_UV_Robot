package org.apache.commons.io.input;

import java.io.IOException;
import java.io.InputStream;
import org.xbill.DNS.TTL;

public class CountingInputStream extends ProxyInputStream {
    private long count;

    public CountingInputStream(InputStream in) {
        super(in);
    }

    public int read(byte[] b) throws IOException {
        int found = super.read(b);
        this.count += found >= 0 ? (long) found : 0;
        return found;
    }

    public int read(byte[] b, int off, int len) throws IOException {
        int found = super.read(b, off, len);
        this.count += found >= 0 ? (long) found : 0;
        return found;
    }

    public int read() throws IOException {
        int found = super.read();
        this.count += found >= 0 ? 1 : 0;
        return found;
    }

    public long skip(long length) throws IOException {
        long skip = super.skip(length);
        this.count += skip;
        return skip;
    }

    public synchronized int getCount() {
        long result;
        result = getByteCount();
        if (result <= TTL.MAX_VALUE) {
        } else {
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append("The byte count ");
            stringBuffer.append(result);
            stringBuffer.append(" is too large to be converted to an int");
            throw new ArithmeticException(stringBuffer.toString());
        }
        return (int) result;
    }

    public synchronized int resetCount() {
        long result;
        result = resetByteCount();
        if (result <= TTL.MAX_VALUE) {
        } else {
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append("The byte count ");
            stringBuffer.append(result);
            stringBuffer.append(" is too large to be converted to an int");
            throw new ArithmeticException(stringBuffer.toString());
        }
        return (int) result;
    }

    public synchronized long getByteCount() {
        return this.count;
    }

    public synchronized long resetByteCount() {
        long tmp;
        tmp = this.count;
        this.count = 0;
        return tmp;
    }
}
