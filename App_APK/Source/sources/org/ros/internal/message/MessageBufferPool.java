package org.ros.internal.message;

import org.apache.commons.pool.ObjectPool;
import org.apache.commons.pool.PoolableObjectFactory;
import org.apache.commons.pool.impl.StackObjectPool;
import org.jboss.netty.buffer.ChannelBuffer;
import org.ros.exception.RosMessageRuntimeException;

public class MessageBufferPool {
    private final ObjectPool<ChannelBuffer> pool = new StackObjectPool(new PoolableObjectFactory<ChannelBuffer>() {
        public ChannelBuffer makeObject() throws Exception {
            return MessageBuffers.dynamicBuffer();
        }

        public void destroyObject(ChannelBuffer channelBuffer) throws Exception {
        }

        public boolean validateObject(ChannelBuffer channelBuffer) {
            return true;
        }

        public void activateObject(ChannelBuffer channelBuffer) throws Exception {
        }

        public void passivateObject(ChannelBuffer channelBuffer) throws Exception {
            channelBuffer.clear();
        }
    });

    public ChannelBuffer acquire() {
        try {
            return this.pool.borrowObject();
        } catch (Exception e) {
            throw new RosMessageRuntimeException((Throwable) e);
        }
    }

    public void release(ChannelBuffer channelBuffer) {
        try {
            this.pool.returnObject(channelBuffer);
        } catch (Exception e) {
            throw new RosMessageRuntimeException((Throwable) e);
        }
    }
}
