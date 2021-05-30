package org.apache.commons.io.input;

import java.io.EOFException;
import java.io.IOException;
import java.io.Reader;

public class NullReader extends Reader {
    private boolean eof;
    private long mark;
    private boolean markSupported;
    private long position;
    private long readlimit;
    private long size;
    private boolean throwEofException;

    public NullReader(long size2) {
        this(size2, true, false);
    }

    public NullReader(long size2, boolean markSupported2, boolean throwEofException2) {
        this.mark = -1;
        this.size = size2;
        this.markSupported = markSupported2;
        this.throwEofException = throwEofException2;
    }

    public long getPosition() {
        return this.position;
    }

    public long getSize() {
        return this.size;
    }

    public void close() throws IOException {
        this.eof = false;
        this.position = 0;
        this.mark = -1;
    }

    public synchronized void mark(int readlimit2) {
        if (this.markSupported) {
            this.mark = this.position;
            this.readlimit = (long) readlimit2;
        } else {
            throw new UnsupportedOperationException("Mark not supported");
        }
    }

    public boolean markSupported() {
        return this.markSupported;
    }

    public int read() throws IOException {
        if (this.eof) {
            throw new IOException("Read after end of file");
        } else if (this.position == this.size) {
            return doEndOfFile();
        } else {
            this.position++;
            return processChar();
        }
    }

    public int read(char[] chars) throws IOException {
        return read(chars, 0, chars.length);
    }

    public int read(char[] chars, int offset, int length) throws IOException {
        if (this.eof) {
            throw new IOException("Read after end of file");
        } else if (this.position == this.size) {
            return doEndOfFile();
        } else {
            this.position += (long) length;
            int returnLength = length;
            if (this.position > this.size) {
                returnLength = length - ((int) (this.position - this.size));
                this.position = this.size;
            }
            processChars(chars, offset, returnLength);
            return returnLength;
        }
    }

    public synchronized void reset() throws IOException {
        if (!this.markSupported) {
            throw new UnsupportedOperationException("Mark not supported");
        } else if (this.mark < 0) {
            throw new IOException("No position has been marked");
        } else if (this.position <= this.mark + this.readlimit) {
            this.position = this.mark;
            this.eof = false;
        } else {
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append("Marked position [");
            stringBuffer.append(this.mark);
            stringBuffer.append("] is no longer valid - passed the read limit [");
            stringBuffer.append(this.readlimit);
            stringBuffer.append("]");
            throw new IOException(stringBuffer.toString());
        }
    }

    public long skip(long numberOfChars) throws IOException {
        if (this.eof) {
            throw new IOException("Skip after end of file");
        } else if (this.position == this.size) {
            return (long) doEndOfFile();
        } else {
            this.position += numberOfChars;
            long returnLength = numberOfChars;
            if (this.position <= this.size) {
                return returnLength;
            }
            long returnLength2 = numberOfChars - (this.position - this.size);
            this.position = this.size;
            return returnLength2;
        }
    }

    /* access modifiers changed from: protected */
    public int processChar() {
        return 0;
    }

    /* access modifiers changed from: protected */
    public void processChars(char[] chars, int offset, int length) {
    }

    private int doEndOfFile() throws EOFException {
        this.eof = true;
        if (!this.throwEofException) {
            return -1;
        }
        throw new EOFException();
    }
}
