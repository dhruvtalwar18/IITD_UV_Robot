package org.apache.xmlrpc.common;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;

public interface ServerStreamConnection {
    void close() throws IOException;

    InputStream newInputStream() throws IOException;

    OutputStream newOutputStream() throws IOException;
}
