package org.jboss.netty.channel.socket.nio;

import java.lang.ref.SoftReference;
import java.nio.ByteBuffer;
import org.apache.commons.net.bsd.RCommandClient;

final class SocketReceiveBufferPool {
    private static final int POOL_SIZE = 8;
    private final SoftReference<ByteBuffer>[] pool = new SoftReference[8];

    SocketReceiveBufferPool() {
    }

    /* access modifiers changed from: package-private */
    public ByteBuffer acquire(int size) {
        SoftReference<ByteBuffer>[] pool2 = this.pool;
        for (int i = 0; i < 8; i++) {
            SoftReference<ByteBuffer> ref = pool2[i];
            if (ref != null) {
                ByteBuffer buf = ref.get();
                if (buf == null) {
                    pool2[i] = null;
                } else if (buf.capacity() >= size) {
                    pool2[i] = null;
                    buf.clear();
                    return buf;
                }
            }
        }
        return ByteBuffer.allocateDirect(normalizeCapacity(size));
    }

    /* access modifiers changed from: package-private */
    public void release(ByteBuffer buffer) {
        SoftReference<ByteBuffer>[] pool2 = this.pool;
        for (int i = 0; i < 8; i++) {
            SoftReference<ByteBuffer> ref = pool2[i];
            if (ref == null || ref.get() == null) {
                pool2[i] = new SoftReference<>(buffer);
                return;
            }
        }
        int capacity = buffer.capacity();
        for (int i2 = 0; i2 < 8; i2++) {
            ByteBuffer pooled = pool2[i2].get();
            if (pooled == null) {
                pool2[i2] = null;
            } else if (pooled.capacity() < capacity) {
                pool2[i2] = new SoftReference<>(buffer);
                return;
            }
        }
    }

    private static int normalizeCapacity(int capacity) {
        int q = capacity >>> 10;
        if ((capacity & RCommandClient.MAX_CLIENT_PORT) != 0) {
            q++;
        }
        return q << 10;
    }
}
