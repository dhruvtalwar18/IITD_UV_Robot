package org.ros.internal.node.service;

import org.jboss.netty.buffer.ChannelBuffer;
import org.jboss.netty.channel.Channel;
import org.jboss.netty.channel.ChannelHandlerContext;
import org.jboss.netty.handler.codec.replay.ReplayingDecoder;

class ServiceResponseDecoder<ResponseType> extends ReplayingDecoder<ServiceResponseDecoderState> {
    private ServiceServerResponse response;

    public ServiceResponseDecoder() {
        reset();
    }

    /* access modifiers changed from: protected */
    public Object decode(ChannelHandlerContext ctx, Channel channel, ChannelBuffer buffer, ServiceResponseDecoderState state) throws Exception {
        switch (state) {
            case ERROR_CODE:
                this.response.setErrorCode(buffer.readByte());
                checkpoint(ServiceResponseDecoderState.MESSAGE_LENGTH);
                break;
            case MESSAGE_LENGTH:
                break;
            case MESSAGE:
                break;
            default:
                throw new IllegalStateException();
        }
        this.response.setMessageLength(buffer.readInt());
        checkpoint(ServiceResponseDecoderState.MESSAGE);
        this.response.setMessage(buffer.readBytes(this.response.getMessageLength()));
        try {
            return this.response;
        } finally {
            reset();
        }
    }

    private void reset() {
        checkpoint(ServiceResponseDecoderState.ERROR_CODE);
        this.response = new ServiceServerResponse();
    }
}
