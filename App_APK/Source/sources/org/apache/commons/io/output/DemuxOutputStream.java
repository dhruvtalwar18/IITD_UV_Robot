package org.apache.commons.io.output;

import java.io.IOException;
import java.io.OutputStream;

public class DemuxOutputStream extends OutputStream {
    private InheritableThreadLocal m_streams = new InheritableThreadLocal();

    public OutputStream bindStream(OutputStream output) {
        OutputStream stream = getStream();
        this.m_streams.set(output);
        return stream;
    }

    public void close() throws IOException {
        OutputStream output = getStream();
        if (output != null) {
            output.close();
        }
    }

    public void flush() throws IOException {
        OutputStream output = getStream();
        if (output != null) {
            output.flush();
        }
    }

    public void write(int ch) throws IOException {
        OutputStream output = getStream();
        if (output != null) {
            output.write(ch);
        }
    }

    private OutputStream getStream() {
        return (OutputStream) this.m_streams.get();
    }
}
