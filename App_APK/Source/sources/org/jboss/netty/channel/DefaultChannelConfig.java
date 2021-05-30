package org.jboss.netty.channel;

import java.util.Map;
import org.jboss.netty.buffer.ChannelBufferFactory;
import org.jboss.netty.buffer.HeapChannelBufferFactory;
import org.jboss.netty.util.internal.ConversionUtil;

public class DefaultChannelConfig implements ChannelConfig {
    private volatile ChannelBufferFactory bufferFactory = HeapChannelBufferFactory.getInstance();
    private volatile int connectTimeoutMillis = 10000;

    public void setOptions(Map<String, Object> options) {
        for (Map.Entry<String, Object> e : options.entrySet()) {
            setOption(e.getKey(), e.getValue());
        }
    }

    public boolean setOption(String key, Object value) {
        if (key.equals("pipelineFactory")) {
            setPipelineFactory((ChannelPipelineFactory) value);
            return true;
        } else if (key.equals("connectTimeoutMillis")) {
            setConnectTimeoutMillis(ConversionUtil.toInt(value));
            return true;
        } else if (!key.equals("bufferFactory")) {
            return false;
        } else {
            setBufferFactory((ChannelBufferFactory) value);
            return true;
        }
    }

    public int getConnectTimeoutMillis() {
        return this.connectTimeoutMillis;
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

    public ChannelPipelineFactory getPipelineFactory() {
        return null;
    }

    public void setConnectTimeoutMillis(int connectTimeoutMillis2) {
        if (connectTimeoutMillis2 >= 0) {
            this.connectTimeoutMillis = connectTimeoutMillis2;
            return;
        }
        throw new IllegalArgumentException("connectTimeoutMillis: " + connectTimeoutMillis2);
    }

    public void setPipelineFactory(ChannelPipelineFactory pipelineFactory) {
    }
}
