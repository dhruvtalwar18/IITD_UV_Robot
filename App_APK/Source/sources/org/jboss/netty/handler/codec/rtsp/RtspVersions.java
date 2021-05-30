package org.jboss.netty.handler.codec.rtsp;

import org.jboss.netty.handler.codec.http.HttpVersion;

public final class RtspVersions {
    public static final HttpVersion RTSP_1_0 = new HttpVersion("RTSP", 1, 0, true);

    public static HttpVersion valueOf(String text) {
        if (text != null) {
            String text2 = text.trim().toUpperCase();
            if (text2.equals("RTSP/1.0")) {
                return RTSP_1_0;
            }
            return new HttpVersion(text2, true);
        }
        throw new NullPointerException("text");
    }

    private RtspVersions() {
    }
}
