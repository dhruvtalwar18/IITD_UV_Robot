package org.jboss.netty.channel;

import java.util.Map;
import org.jboss.netty.buffer.ChannelBufferFactory;
import org.jboss.netty.buffer.HeapChannelBufferFactory;

public class DefaultServerChannelConfig implements ChannelConfig {
    private volatile ChannelBufferFactory bufferFactory = HeapChannelBufferFactory.getInstance();
    private volatile ChannelPipelineFactory pipelineFactory;

    public void setOptions(Map<String, Object> options) {
        for (Map.Entry<String, Object> e : options.entrySet()) {
            setOption(e.getKey(), e.getValue());
        }
    }

    public boolean setOption(String key, Object value) {
        if (key.equals("pipelineFactory")) {
            setPipelineFactory((ChannelPipelineFactory) value);
            return true;
        } else if (!key.equals("bufferFactory")) {
            return false;
        } else {
            setBufferFactory((ChannelBufferFactory) value);
            return true;
        }
    }

    public ChannelPipelineFactory getPipelineFactory() {
        return this.pipelineFactory;
    }

    public void setPipelineFactory(ChannelPipelineFactory pipelineFactory2) {
        if (pipelineFactory2 != null) {
            this.pipelineFactory = pipelineFactory2;
            return;
        }
        throw new NullPointerException("pipelineFactory");
    }

    public ChannelBufferFactory getBufferFactory() {
        return this.bufferFactory;
    }

    public void setBufferFactory(ChannelBufferFactory bufferFactory2) {
        if (bufferFactory2 != null) {
            this.bufferFactory = bufferFactory2;
            return;
        }
        throw new NullPointerException("bufferFactory");
    }

    public int getConnectTimeoutMillis() {
        return 0;
    }

    public void setConnectTimeoutMillis(int connectTimeoutMillis) {
    }
}
