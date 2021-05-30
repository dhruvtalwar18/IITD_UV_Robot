package org.jboss.netty.handler.ssl;

import java.nio.ByteBuffer;

public class SslBufferPool {
    private static final int DEFAULT_POOL_SIZE = 19162112;
    private static final int MAX_PACKET_SIZE = 18713;
    private int index;
    private final int maxBufferCount;
    private final ByteBuffer[] pool;

    public SslBufferPool() {
        this(DEFAULT_POOL_SIZE);
    }

    public SslBufferPool(int maxPoolSize) {
        if (maxPoolSize > 0) {
            int maxBufferCount2 = maxPoolSize / MAX_PACKET_SIZE;
            maxBufferCount2 = maxPoolSize % MAX_PACKET_SIZE != 0 ? maxBufferCount2 + 1 : maxBufferCount2;
            this.pool = new ByteBuffer[maxBufferCount2];
            this.maxBufferCount = maxBufferCount2;
            return;
        }
        throw new IllegalArgumentException("maxPoolSize: " + maxPoolSize);
    }

    public int getMaxPoolSize() {
        return this.maxBufferCount * MAX_PACKET_SIZE;
    }

    public synchronized int getUnacquiredPoolSize() {
        return this.index * MAX_PACKET_SIZE;
    }

    public ByteBuffer acquireBuffer() {
        return acquire();
    }

    /* access modifiers changed from: package-private */
    @Deprecated
    public synchronized ByteBuffer acquire() {
        if (this.index == 0) {
            return ByteBuffer.allocate(MAX_PACKET_SIZE);
        }
        ByteBuffer[] byteBufferArr = this.pool;
        int i = this.index - 1;
        this.index = i;
        return (ByteBuffer) byteBufferArr[i].clear();
    }

    public void releaseBuffer(ByteBuffer buffer) {
        release(buffer);
    }

    /* access modifiers changed from: package-private */
    @Deprecated
    public synchronized void release(ByteBuffer buffer) {
        if (this.index < this.maxBufferCount) {
            ByteBuffer[] byteBufferArr = this.pool;
            int i = this.index;
            this.index = i + 1;
            byteBufferArr[i] = buffer;
        }
    }
}
