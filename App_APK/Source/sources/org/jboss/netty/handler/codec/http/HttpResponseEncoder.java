package org.jboss.netty.handler.codec.http;

import org.jboss.netty.buffer.ChannelBuffer;

public class HttpResponseEncoder extends HttpMessageEncoder {
    /* access modifiers changed from: protected */
    public void encodeInitialLine(ChannelBuffer buf, HttpMessage message) throws Exception {
        HttpResponse response = (HttpResponse) message;
        buf.writeBytes(response.getProtocolVersion().toString().getBytes(NTLM.DEFAULT_CHARSET));
        buf.writeByte(32);
        buf.writeBytes(String.valueOf(response.getStatus().getCode()).getBytes(NTLM.DEFAULT_CHARSET));
        buf.writeByte(32);
        buf.writeBytes(String.valueOf(response.getStatus().getReasonPhrase()).getBytes(NTLM.DEFAULT_CHARSET));
        buf.writeByte(13);
        buf.writeByte(10);
    }
}
