package org.apache.commons.net.io;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.io.Reader;
import java.io.Writer;

public final class Util {
    public static final int DEFAULT_COPY_BUFFER_SIZE = 1024;

    private Util() {
    }

    public static final long copyStream(InputStream source, OutputStream dest, int bufferSize, long streamSize, CopyStreamListener listener, boolean flush) throws CopyStreamException {
        long total;
        OutputStream outputStream = dest;
        byte[] buffer = new byte[bufferSize];
        long total2 = 0;
        int bytes = 0;
        while (true) {
            InputStream inputStream = source;
            try {
                int read = source.read(buffer);
                int bytes2 = read;
                if (read == -1) {
                    break;
                }
                if (bytes2 == 0) {
                    try {
                        bytes2 = source.read();
                        if (bytes2 < 0) {
                            break;
                        }
                        dest.write(bytes2);
                        if (flush) {
                            dest.flush();
                        }
                        total = total2 + 1;
                        if (listener != null) {
                            try {
                                listener.bytesTransferred(total, 1, streamSize);
                            } catch (IOException e) {
                                e = e;
                                total2 = total;
                            }
                        }
                    } catch (IOException e2) {
                        e = e2;
                        throw new CopyStreamException("IOException caught while copying.", total2, e);
                    }
                } else {
                    dest.write(buffer, 0, bytes2);
                    if (flush) {
                        dest.flush();
                    }
                    total = total2 + ((long) bytes2);
                    if (listener != null) {
                        listener.bytesTransferred(total, bytes2, streamSize);
                    }
                }
                bytes = bytes2;
                total2 = total;
            } catch (IOException e3) {
                e = e3;
                int i = bytes;
                throw new CopyStreamException("IOException caught while copying.", total2, e);
            }
        }
        return total2;
    }

    public static final long copyStream(InputStream source, OutputStream dest, int bufferSize, long streamSize, CopyStreamListener listener) throws CopyStreamException {
        return copyStream(source, dest, bufferSize, streamSize, listener, true);
    }

    public static final long copyStream(InputStream source, OutputStream dest, int bufferSize) throws CopyStreamException {
        return copyStream(source, dest, bufferSize, -1, (CopyStreamListener) null);
    }

    public static final long copyStream(InputStream source, OutputStream dest) throws CopyStreamException {
        return copyStream(source, dest, 1024);
    }

    public static final long copyReader(Reader source, Writer dest, int bufferSize, long streamSize, CopyStreamListener listener) throws CopyStreamException {
        long total;
        Writer writer = dest;
        char[] buffer = new char[bufferSize];
        long total2 = 0;
        int chars = 0;
        while (true) {
            Reader reader = source;
            try {
                int read = source.read(buffer);
                int chars2 = read;
                if (read == -1) {
                    break;
                }
                if (chars2 == 0) {
                    try {
                        chars2 = source.read();
                        if (chars2 < 0) {
                            break;
                        }
                        dest.write(chars2);
                        dest.flush();
                        total = total2 + 1;
                        if (listener != null) {
                            try {
                                listener.bytesTransferred(total, chars2, streamSize);
                            } catch (IOException e) {
                                e = e;
                                total2 = total;
                            }
                        }
                    } catch (IOException e2) {
                        e = e2;
                        throw new CopyStreamException("IOException caught while copying.", total2, e);
                    }
                } else {
                    dest.write(buffer, 0, chars2);
                    dest.flush();
                    total = total2 + ((long) chars2);
                    if (listener != null) {
                        listener.bytesTransferred(total, chars2, streamSize);
                    }
                }
                chars = chars2;
                total2 = total;
            } catch (IOException e3) {
                e = e3;
                int i = chars;
                throw new CopyStreamException("IOException caught while copying.", total2, e);
            }
        }
        return total2;
    }

    public static final long copyReader(Reader source, Writer dest, int bufferSize) throws CopyStreamException {
        return copyReader(source, dest, bufferSize, -1, (CopyStreamListener) null);
    }

    public static final long copyReader(Reader source, Writer dest) throws CopyStreamException {
        return copyReader(source, dest, 1024);
    }
}
