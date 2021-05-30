package org.apache.commons.httpclient;

import java.io.ByteArrayInputStream;
import java.io.IOException;
import java.io.InputStream;
import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

class Wire {
    public static Wire CONTENT_WIRE = new Wire(LogFactory.getLog("httpclient.wire.content"));
    public static Wire HEADER_WIRE = new Wire(LogFactory.getLog("httpclient.wire.header"));
    private Log log;

    private Wire(Log log2) {
        this.log = log2;
    }

    private void wire(String header, InputStream instream) throws IOException {
        StringBuffer buffer = new StringBuffer();
        while (true) {
            int read = instream.read();
            int ch = read;
            if (read == -1) {
                break;
            } else if (ch == 13) {
                buffer.append("[\\r]");
            } else if (ch == 10) {
                buffer.append("[\\n]\"");
                buffer.insert(0, "\"");
                buffer.insert(0, header);
                this.log.debug(buffer.toString());
                buffer.setLength(0);
            } else if (ch < 32 || ch > 127) {
                buffer.append("[0x");
                buffer.append(Integer.toHexString(ch));
                buffer.append("]");
            } else {
                buffer.append((char) ch);
            }
        }
        if (buffer.length() > 0) {
            buffer.append("\"");
            buffer.insert(0, "\"");
            buffer.insert(0, header);
            this.log.debug(buffer.toString());
        }
    }

    public boolean enabled() {
        return this.log.isDebugEnabled();
    }

    public void output(InputStream outstream) throws IOException {
        if (outstream != null) {
            wire(">> ", outstream);
            return;
        }
        throw new IllegalArgumentException("Output may not be null");
    }

    public void input(InputStream instream) throws IOException {
        if (instream != null) {
            wire("<< ", instream);
            return;
        }
        throw new IllegalArgumentException("Input may not be null");
    }

    public void output(byte[] b, int off, int len) throws IOException {
        if (b != null) {
            wire(">> ", new ByteArrayInputStream(b, off, len));
            return;
        }
        throw new IllegalArgumentException("Output may not be null");
    }

    public void input(byte[] b, int off, int len) throws IOException {
        if (b != null) {
            wire("<< ", new ByteArrayInputStream(b, off, len));
            return;
        }
        throw new IllegalArgumentException("Input may not be null");
    }

    public void output(byte[] b) throws IOException {
        if (b != null) {
            wire(">> ", new ByteArrayInputStream(b));
            return;
        }
        throw new IllegalArgumentException("Output may not be null");
    }

    public void input(byte[] b) throws IOException {
        if (b != null) {
            wire("<< ", new ByteArrayInputStream(b));
            return;
        }
        throw new IllegalArgumentException("Input may not be null");
    }

    public void output(int b) throws IOException {
        output(new byte[]{(byte) b});
    }

    public void input(int b) throws IOException {
        input(new byte[]{(byte) b});
    }

    public void output(String s) throws IOException {
        if (s != null) {
            output(s.getBytes());
            return;
        }
        throw new IllegalArgumentException("Output may not be null");
    }

    public void input(String s) throws IOException {
        if (s != null) {
            input(s.getBytes());
            return;
        }
        throw new IllegalArgumentException("Input may not be null");
    }
}
