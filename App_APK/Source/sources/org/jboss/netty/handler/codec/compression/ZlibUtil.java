package org.jboss.netty.handler.codec.compression;

import org.jboss.netty.util.internal.jzlib.JZlib;
import org.jboss.netty.util.internal.jzlib.ZStream;

final class ZlibUtil {
    static void fail(ZStream z, String message, int resultCode) {
        throw exception(z, message, resultCode);
    }

    static CompressionException exception(ZStream z, String message, int resultCode) {
        String str;
        StringBuilder sb = new StringBuilder();
        sb.append(message);
        sb.append(" (");
        sb.append(resultCode);
        sb.append(")");
        if (z.msg != null) {
            str = ": " + z.msg;
        } else {
            str = "";
        }
        sb.append(str);
        return new CompressionException(sb.toString());
    }

    static Enum<?> convertWrapperType(ZlibWrapper wrapper) {
        switch (wrapper) {
            case NONE:
                return JZlib.W_NONE;
            case ZLIB:
                return JZlib.W_ZLIB;
            case GZIP:
                return JZlib.W_GZIP;
            case ZLIB_OR_NONE:
                return JZlib.W_ZLIB_OR_NONE;
            default:
                throw new Error();
        }
    }

    private ZlibUtil() {
    }
}
