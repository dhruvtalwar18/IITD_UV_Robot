package org.jboss.netty.handler.codec.replay;

import java.lang.Enum;
import java.net.SocketAddress;
import org.jboss.netty.buffer.ChannelBuffer;
import org.jboss.netty.channel.Channel;
import org.jboss.netty.channel.ChannelHandlerContext;
import org.jboss.netty.channel.ChannelStateEvent;
import org.jboss.netty.channel.MessageEvent;
import org.jboss.netty.handler.codec.frame.FrameDecoder;

public abstract class ReplayingDecoder<T extends Enum<T>> extends FrameDecoder {
    private int checkpoint;
    private boolean needsCleanup;
    private final ReplayingDecoderBuffer replayable;
    private T state;

    /* access modifiers changed from: protected */
    public abstract Object decode(ChannelHandlerContext channelHandlerContext, Channel channel, ChannelBuffer channelBuffer, T t) throws Exception;

    protected ReplayingDecoder() {
        this((Enum) null);
    }

    protected ReplayingDecoder(boolean unfold) {
        this((Enum) null, unfold);
    }

    protected ReplayingDecoder(T initialState) {
        this(initialState, false);
    }

    protected ReplayingDecoder(T initialState, boolean unfold) {
        super(unfold);
        this.replayable = new ReplayingDecoderBuffer(this);
        this.state = initialState;
    }

    /* access modifiers changed from: protected */
    public ChannelBuffer internalBuffer() {
        return super.internalBuffer();
    }

    /* access modifiers changed from: protected */
    public void checkpoint() {
        ChannelBuffer cumulation = this.cumulation;
        if (cumulation != null) {
            this.checkpoint = cumulation.readerIndex();
        } else {
            this.checkpoint = -1;
        }
    }

    /* access modifiers changed from: protected */
    public void checkpoint(T state2) {
        checkpoint();
        setState(state2);
    }

    /* access modifiers changed from: protected */
    public T getState() {
        return this.state;
    }

    /* access modifiers changed from: protected */
    public T setState(T newState) {
        T oldState = this.state;
        this.state = newState;
        return oldState;
    }

    /* access modifiers changed from: protected */
    public Object decodeLast(ChannelHandlerContext ctx, Channel channel, ChannelBuffer buffer, T state2) throws Exception {
        return decode(ctx, channel, buffer, state2);
    }

    /* access modifiers changed from: protected */
    public final Object decode(ChannelHandlerContext ctx, Channel channel, ChannelBuffer buffer) throws Exception {
        return decode(ctx, channel, buffer, this.state);
    }

    /* access modifiers changed from: protected */
    public final Object decodeLast(ChannelHandlerContext ctx, Channel channel, ChannelBuffer buffer) throws Exception {
        return decodeLast(ctx, channel, buffer, this.state);
    }

    public void messageReceived(ChannelHandlerContext ctx, MessageEvent e) throws Exception {
        ChannelHandlerContext channelHandlerContext = ctx;
        Object m = e.getMessage();
        if (!(m instanceof ChannelBuffer)) {
            ctx.sendUpstream(e);
            return;
        }
        ChannelBuffer input = (ChannelBuffer) m;
        if (input.readable()) {
            this.needsCleanup = true;
            if (this.cumulation == null) {
                this.cumulation = input;
                int oldReaderIndex = input.readerIndex();
                int inputSize = input.readableBytes();
                boolean z = false;
                try {
                    callDecode(ctx, e.getChannel(), input, this.replayable, e.getRemoteAddress());
                    int readableBytes = input.readableBytes();
                    if (readableBytes > 0) {
                        int inputCapacity = input.capacity();
                        if (readableBytes != inputCapacity && inputCapacity > getMaxCumulationBufferCapacity()) {
                            z = true;
                        }
                        boolean copy = z;
                        if (this.checkpoint > 0) {
                            int bytesToPreserve = inputSize - (this.checkpoint - oldReaderIndex);
                            if (copy) {
                                ChannelBuffer cumulation = newCumulationBuffer(channelHandlerContext, bytesToPreserve);
                                this.cumulation = cumulation;
                                cumulation.writeBytes(input, this.checkpoint, bytesToPreserve);
                                return;
                            }
                            ChannelBuffer slice = input.slice(this.checkpoint, bytesToPreserve);
                            ChannelBuffer channelBuffer = slice;
                            this.cumulation = slice;
                        } else if (this.checkpoint == 0) {
                            if (copy) {
                                ChannelBuffer newCumulationBuffer = newCumulationBuffer(channelHandlerContext, inputSize);
                                ChannelBuffer cumulation2 = newCumulationBuffer;
                                this.cumulation = newCumulationBuffer;
                                cumulation2.writeBytes(input, oldReaderIndex, inputSize);
                                cumulation2.readerIndex(input.readerIndex());
                                return;
                            }
                            ChannelBuffer cumulation3 = input.slice(oldReaderIndex, inputSize);
                            this.cumulation = cumulation3;
                            cumulation3.readerIndex(input.readerIndex());
                        } else if (copy) {
                            ChannelBuffer cumulation4 = newCumulationBuffer(channelHandlerContext, input.readableBytes());
                            this.cumulation = cumulation4;
                            cumulation4.writeBytes(input);
                        } else {
                            ChannelBuffer channelBuffer2 = input;
                            this.cumulation = input;
                        }
                    } else {
                        this.cumulation = null;
                    }
                } catch (Throwable th) {
                    int oldReaderIndex2 = oldReaderIndex;
                    int inputSize2 = inputSize;
                    ChannelHandlerContext ctx2 = ctx;
                    MessageEvent messageEvent = e;
                    Object obj = m;
                    ChannelBuffer input2 = input;
                    int readableBytes2 = input2.readableBytes();
                    if (readableBytes2 > 0) {
                        int inputCapacity2 = input2.capacity();
                        if (readableBytes2 != inputCapacity2 && inputCapacity2 > getMaxCumulationBufferCapacity()) {
                            z = true;
                        }
                        boolean copy2 = z;
                        if (this.checkpoint > 0) {
                            int bytesToPreserve2 = inputSize2 - (this.checkpoint - oldReaderIndex2);
                            if (copy2) {
                                ChannelBuffer cumulation5 = newCumulationBuffer(ctx2, bytesToPreserve2);
                                this.cumulation = cumulation5;
                                cumulation5.writeBytes(input2, this.checkpoint, bytesToPreserve2);
                            } else {
                                ChannelBuffer slice2 = input2.slice(this.checkpoint, bytesToPreserve2);
                                ChannelBuffer channelBuffer3 = slice2;
                                this.cumulation = slice2;
                            }
                        } else if (this.checkpoint == 0) {
                            if (copy2) {
                                ChannelBuffer newCumulationBuffer2 = newCumulationBuffer(ctx2, inputSize2);
                                ChannelBuffer cumulation6 = newCumulationBuffer2;
                                this.cumulation = newCumulationBuffer2;
                                cumulation6.writeBytes(input2, oldReaderIndex2, inputSize2);
                                cumulation6.readerIndex(input2.readerIndex());
                            } else {
                                ChannelBuffer cumulation7 = input2.slice(oldReaderIndex2, inputSize2);
                                this.cumulation = cumulation7;
                                cumulation7.readerIndex(input2.readerIndex());
                            }
                        } else if (copy2) {
                            ChannelBuffer cumulation8 = newCumulationBuffer(ctx2, input2.readableBytes());
                            this.cumulation = cumulation8;
                            cumulation8.writeBytes(input2);
                        } else {
                            ChannelBuffer channelBuffer4 = input2;
                            this.cumulation = input2;
                        }
                    } else {
                        this.cumulation = null;
                    }
                    throw th;
                }
            } else {
                ChannelBuffer input3 = appendToCumulation(input);
                try {
                    callDecode(ctx, e.getChannel(), input3, this.replayable, e.getRemoteAddress());
                    updateCumulation(channelHandlerContext, input3);
                } catch (Throwable th2) {
                    MessageEvent messageEvent2 = e;
                    Object obj2 = m;
                    updateCumulation(ctx, input3);
                    throw th2;
                }
            }
        }
    }

    private void callDecode(ChannelHandlerContext context, Channel channel, ChannelBuffer input, ChannelBuffer replayableInput, SocketAddress remoteAddress) throws Exception {
        while (input.readable()) {
            int oldReaderIndex = input.readerIndex();
            this.checkpoint = oldReaderIndex;
            Object result = null;
            T oldState = this.state;
            try {
                result = decode(context, channel, replayableInput, this.state);
                if (result == null) {
                    if (oldReaderIndex != input.readerIndex()) {
                        continue;
                    } else if (oldState == this.state) {
                        throw new IllegalStateException("null cannot be returned if no data is consumed and state didn't change.");
                    }
                }
            } catch (ReplayError e) {
                int checkpoint2 = this.checkpoint;
                if (checkpoint2 >= 0) {
                    input.readerIndex(checkpoint2);
                }
            }
            if (result != null) {
                if (oldReaderIndex == input.readerIndex() && oldState == this.state) {
                    throw new IllegalStateException("decode() method must consume at least one byte if it returned a decoded message (caused by: " + getClass() + ")");
                }
                unfoldAndFireMessageReceived(context, remoteAddress, result);
            } else {
                return;
            }
        }
    }

    /* access modifiers changed from: protected */
    public void cleanup(ChannelHandlerContext ctx, ChannelStateEvent e) throws Exception {
        try {
            ChannelBuffer cumulation = this.cumulation;
            if (!this.needsCleanup) {
                ctx.sendUpstream(e);
                return;
            }
            this.needsCleanup = false;
            this.replayable.terminate();
            if (cumulation != null && cumulation.readable()) {
                callDecode(ctx, e.getChannel(), cumulation, this.replayable, (SocketAddress) null);
            }
            Object partiallyDecoded = decodeLast(ctx, e.getChannel(), this.replayable, this.state);
            this.cumulation = null;
            if (partiallyDecoded != null) {
                unfoldAndFireMessageReceived(ctx, (SocketAddress) null, partiallyDecoded);
            }
            ctx.sendUpstream(e);
        } catch (ReplayError e2) {
            ctx.sendUpstream(e);
        } catch (Throwable th) {
            ctx.sendUpstream(e);
            throw th;
        }
    }
}
