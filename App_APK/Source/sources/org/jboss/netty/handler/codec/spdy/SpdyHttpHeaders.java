package org.jboss.netty.handler.codec.spdy;

import org.jboss.netty.handler.codec.http.HttpHeaders;
import org.jboss.netty.handler.codec.http.HttpMessage;

public final class SpdyHttpHeaders {

    public static final class Names {
        public static final String ASSOCIATED_TO_STREAM_ID = "X-SPDY-Associated-To-Stream-ID";
        public static final String PRIORITY = "X-SPDY-Priority";
        public static final String SCHEME = "X-SPDY-Scheme";
        public static final String STREAM_ID = "X-SPDY-Stream-ID";
        public static final String URL = "X-SPDY-URL";

        private Names() {
        }
    }

    private SpdyHttpHeaders() {
    }

    @Deprecated
    public static void removeStreamID(HttpMessage message) {
        removeStreamId(message);
    }

    public static void removeStreamId(HttpMessage message) {
        message.removeHeader(Names.STREAM_ID);
    }

    @Deprecated
    public static int getStreamID(HttpMessage message) {
        return getStreamId(message);
    }

    public static int getStreamId(HttpMessage message) {
        return HttpHeaders.getIntHeader(message, Names.STREAM_ID);
    }

    @Deprecated
    public static void setStreamID(HttpMessage message, int streamId) {
        setStreamId(message, streamId);
    }

    public static void setStreamId(HttpMessage message, int streamId) {
        HttpHeaders.setIntHeader(message, Names.STREAM_ID, streamId);
    }

    @Deprecated
    public static void removeAssociatedToStreamID(HttpMessage message) {
        removeAssociatedToStreamId(message);
    }

    public static void removeAssociatedToStreamId(HttpMessage message) {
        message.removeHeader(Names.ASSOCIATED_TO_STREAM_ID);
    }

    @Deprecated
    public static int getAssociatedToStreamID(HttpMessage message) {
        return getAssociatedToStreamId(message);
    }

    public static int getAssociatedToStreamId(HttpMessage message) {
        return HttpHeaders.getIntHeader(message, Names.ASSOCIATED_TO_STREAM_ID, 0);
    }

    @Deprecated
    public static void setAssociatedToStreamID(HttpMessage message, int associatedToStreamId) {
        setAssociatedToStreamId(message, associatedToStreamId);
    }

    public static void setAssociatedToStreamId(HttpMessage message, int associatedToStreamId) {
        HttpHeaders.setIntHeader(message, Names.ASSOCIATED_TO_STREAM_ID, associatedToStreamId);
    }

    public static void removePriority(HttpMessage message) {
        message.removeHeader(Names.PRIORITY);
    }

    public static byte getPriority(HttpMessage message) {
        return (byte) HttpHeaders.getIntHeader(message, Names.PRIORITY, 0);
    }

    public static void setPriority(HttpMessage message, byte priority) {
        HttpHeaders.setIntHeader(message, Names.PRIORITY, (int) priority);
    }

    public static void removeUrl(HttpMessage message) {
        message.removeHeader(Names.URL);
    }

    public static String getUrl(HttpMessage message) {
        return message.getHeader(Names.URL);
    }

    public static void setUrl(HttpMessage message, String url) {
        message.setHeader(Names.URL, (Object) url);
    }

    public static void removeScheme(HttpMessage message) {
        message.removeHeader(Names.SCHEME);
    }

    public static String getScheme(HttpMessage message) {
        return message.getHeader(Names.SCHEME);
    }

    public static void setScheme(HttpMessage message, String scheme) {
        message.setHeader(Names.SCHEME, (Object) scheme);
    }
}
