package org.jboss.netty.handler.codec.http.websocketx;

import java.security.MessageDigest;
import java.security.NoSuchAlgorithmException;
import org.jboss.netty.buffer.ChannelBuffers;
import org.jboss.netty.handler.codec.base64.Base64;
import org.jboss.netty.util.CharsetUtil;

final class WebSocketUtil {
    static byte[] md5(byte[] bytes) {
        try {
            return MessageDigest.getInstance("MD5").digest(bytes);
        } catch (NoSuchAlgorithmException e) {
            throw new InternalError("MD5 not supported on this platform");
        }
    }

    static byte[] sha1(byte[] bytes) {
        try {
            return MessageDigest.getInstance("SHA1").digest(bytes);
        } catch (NoSuchAlgorithmException e) {
            throw new InternalError("SHA-1 not supported on this platform");
        }
    }

    static String base64(byte[] bytes) {
        return Base64.encode(ChannelBuffers.wrappedBuffer(bytes)).toString(CharsetUtil.UTF_8);
    }

    static byte[] randomBytes(int size) {
        byte[] bytes = new byte[size];
        for (int i = 0; i < size; i++) {
            bytes[i] = (byte) randomNumber(0, 255);
        }
        return bytes;
    }

    static int randomNumber(int min, int max) {
        double random = Math.random();
        double d = (double) max;
        Double.isNaN(d);
        double d2 = random * d;
        double d3 = (double) min;
        Double.isNaN(d3);
        return (int) (d2 + d3);
    }

    private WebSocketUtil() {
    }
}
