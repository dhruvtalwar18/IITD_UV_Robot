package org.apache.commons.httpclient;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import org.apache.commons.httpclient.cookie.CookieSpec;
import org.apache.commons.httpclient.util.EncodingUtil;
import org.apache.commons.httpclient.util.ExceptionUtil;
import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

public class ChunkedInputStream extends InputStream {
    private static final Log LOG;
    static /* synthetic */ Class class$org$apache$commons$httpclient$ChunkedInputStream;
    private boolean bof;
    private int chunkSize;
    private boolean closed;
    private boolean eof;
    private InputStream in;
    private HttpMethod method;
    private int pos;

    static {
        Class cls;
        if (class$org$apache$commons$httpclient$ChunkedInputStream == null) {
            cls = class$("org.apache.commons.httpclient.ChunkedInputStream");
            class$org$apache$commons$httpclient$ChunkedInputStream = cls;
        } else {
            cls = class$org$apache$commons$httpclient$ChunkedInputStream;
        }
        LOG = LogFactory.getLog(cls);
    }

    static /* synthetic */ Class class$(String x0) {
        try {
            return Class.forName(x0);
        } catch (ClassNotFoundException x1) {
            throw new NoClassDefFoundError(x1.getMessage());
        }
    }

    public ChunkedInputStream(InputStream in2, HttpMethod method2) throws IOException {
        this.bof = true;
        this.eof = false;
        this.closed = false;
        this.method = null;
        if (in2 != null) {
            this.in = in2;
            this.method = method2;
            this.pos = 0;
            return;
        }
        throw new IllegalArgumentException("InputStream parameter may not be null");
    }

    public ChunkedInputStream(InputStream in2) throws IOException {
        this(in2, (HttpMethod) null);
    }

    public int read() throws IOException {
        if (this.closed) {
            throw new IOException("Attempted read from closed stream.");
        } else if (this.eof) {
            return -1;
        } else {
            if (this.pos >= this.chunkSize) {
                nextChunk();
                if (this.eof) {
                    return -1;
                }
            }
            this.pos++;
            return this.in.read();
        }
    }

    public int read(byte[] b, int off, int len) throws IOException {
        if (this.closed) {
            throw new IOException("Attempted read from closed stream.");
        } else if (this.eof) {
            return -1;
        } else {
            if (this.pos >= this.chunkSize) {
                nextChunk();
                if (this.eof) {
                    return -1;
                }
            }
            int count = this.in.read(b, off, Math.min(len, this.chunkSize - this.pos));
            this.pos += count;
            return count;
        }
    }

    public int read(byte[] b) throws IOException {
        return read(b, 0, b.length);
    }

    private void readCRLF() throws IOException {
        int cr = this.in.read();
        int lf = this.in.read();
        if (cr != 13 || lf != 10) {
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append("CRLF expected at end of chunk: ");
            stringBuffer.append(cr);
            stringBuffer.append(CookieSpec.PATH_DELIM);
            stringBuffer.append(lf);
            throw new IOException(stringBuffer.toString());
        }
    }

    private void nextChunk() throws IOException {
        if (!this.bof) {
            readCRLF();
        }
        this.chunkSize = getChunkSizeFromInputStream(this.in);
        this.bof = false;
        this.pos = 0;
        if (this.chunkSize == 0) {
            this.eof = true;
            parseTrailerHeaders();
        }
    }

    private static int getChunkSizeFromInputStream(InputStream in2) throws IOException {
        ByteArrayOutputStream baos = new ByteArrayOutputStream();
        int state = 0;
        while (state != -1) {
            int b = in2.read();
            if (b != -1) {
                switch (state) {
                    case 0:
                        if (b == 13) {
                            state = 1;
                            break;
                        } else {
                            if (b == 34) {
                                state = 2;
                            }
                            baos.write(b);
                            break;
                        }
                    case 1:
                        if (b == 10) {
                            state = -1;
                            break;
                        } else {
                            throw new IOException("Protocol violation: Unexpected single newline character in chunk size");
                        }
                    case 2:
                        if (b != 34) {
                            if (b == 92) {
                                baos.write(in2.read());
                                break;
                            }
                        } else {
                            state = 0;
                        }
                        baos.write(b);
                        break;
                    default:
                        throw new RuntimeException("assertion failed");
                }
            } else {
                throw new IOException("chunked stream ended unexpectedly");
            }
        }
        String dataString = EncodingUtil.getAsciiString(baos.toByteArray());
        int separator = dataString.indexOf(59);
        String dataString2 = separator > 0 ? dataString.substring(0, separator).trim() : dataString.trim();
        try {
            return Integer.parseInt(dataString2.trim(), 16);
        } catch (NumberFormatException e) {
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append("Bad chunk size: ");
            stringBuffer.append(dataString2);
            throw new IOException(stringBuffer.toString());
        }
    }

    private void parseTrailerHeaders() throws IOException {
        String charset = "US-ASCII";
        try {
            if (this.method != null) {
                charset = this.method.getParams().getHttpElementCharset();
            }
            Header[] footers = HttpParser.parseHeaders(this.in, charset);
            if (this.method != null) {
                for (Header addResponseFooter : footers) {
                    this.method.addResponseFooter(addResponseFooter);
                }
            }
        } catch (HttpException e) {
            LOG.error("Error parsing trailer headers", e);
            IOException ioe = new IOException(e.getMessage());
            ExceptionUtil.initCause(ioe, e);
            throw ioe;
        }
    }

    public void close() throws IOException {
        if (!this.closed) {
            try {
                if (!this.eof) {
                    exhaustInputStream(this);
                }
            } finally {
                this.eof = true;
                this.closed = true;
            }
        }
    }

    static void exhaustInputStream(InputStream inStream) throws IOException {
        do {
        } while (inStream.read(new byte[1024]) >= 0);
    }
}
