package org.jboss.netty.handler.codec.http;

import org.jboss.netty.handler.codec.spdy.SpdyHeaders;
import org.jboss.netty.util.internal.StringUtil;

public class DefaultHttpRequest extends DefaultHttpMessage implements HttpRequest {
    private HttpMethod method;
    private String uri;

    public DefaultHttpRequest(HttpVersion httpVersion, HttpMethod method2, String uri2) {
        super(httpVersion);
        setMethod(method2);
        setUri(uri2);
    }

    public HttpMethod getMethod() {
        return this.method;
    }

    public void setMethod(HttpMethod method2) {
        if (method2 != null) {
            this.method = method2;
            return;
        }
        throw new NullPointerException(SpdyHeaders.Spdy2HttpNames.METHOD);
    }

    public String getUri() {
        return this.uri;
    }

    public void setUri(String uri2) {
        if (uri2 != null) {
            this.uri = uri2;
            return;
        }
        throw new NullPointerException("uri");
    }

    public String toString() {
        StringBuilder buf = new StringBuilder();
        buf.append(getClass().getSimpleName());
        buf.append("(chunked: ");
        buf.append(isChunked());
        buf.append(')');
        buf.append(StringUtil.NEWLINE);
        buf.append(getMethod().toString());
        buf.append(' ');
        buf.append(getUri());
        buf.append(' ');
        buf.append(getProtocolVersion().getText());
        buf.append(StringUtil.NEWLINE);
        appendHeaders(buf);
        buf.setLength(buf.length() - StringUtil.NEWLINE.length());
        return buf.toString();
    }
}
