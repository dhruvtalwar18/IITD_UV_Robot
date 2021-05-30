package org.apache.commons.io.output;

import java.io.IOException;
import java.io.OutputStream;
import org.xbill.DNS.TTL;

public class CountingOutputStream extends ProxyOutputStream {
    private long count;

    public CountingOutputStream(OutputStream out) {
        super(out);
    }

    public void write(byte[] b) throws IOException {
        this.count += (long) b.length;
        super.write(b);
    }

    public void write(byte[] b, int off, int len) throws IOException {
        this.count += (long) len;
        super.write(b, off, len);
    }

    public void write(int b) throws IOException {
        this.count++;
        super.write(b);
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
