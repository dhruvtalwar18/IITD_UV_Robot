package org.jboss.netty.handler.codec.rtsp;

import org.jboss.netty.buffer.ChannelBuffer;
import org.jboss.netty.handler.codec.http.HttpMessage;
import org.jboss.netty.handler.codec.http.HttpRequest;

public class RtspRequestEncoder extends RtspMessageEncoder {
    /* access modifiers changed from: protected */
    public void encodeInitialLine(ChannelBuffer buf, HttpMessage message) throws Exception {
        HttpRequest request = (HttpRequest) message;
        buf.writeBytes(request.getMethod().toString().getBytes(NTLM.DEFAULT_CHARSET));
        buf.writeByte(32);
        buf.writeBytes(request.getUri().getBytes(NTLM.DEFAULT_CHARSET));
        buf.writeByte(32);
        buf.writeBytes(request.getProtocolVersion().toString().getBytes(NTLM.DEFAULT_CHARSET));
        buf.writeByte(13);
        buf.writeByte(10);
    }
}
