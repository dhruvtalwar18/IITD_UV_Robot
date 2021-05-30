package org.jboss.netty.channel;

import java.util.Collections;
import java.util.Iterator;
import java.util.Map;
import java.util.concurrent.ConcurrentMap;
import org.jboss.netty.util.internal.ConcurrentIdentityWeakKeyHashMap;

public class ChannelLocal<T> implements Iterable<Map.Entry<Channel, T>> {
    private final ConcurrentMap<Channel, T> map;
    private final boolean removeOnClose;
    private final ChannelFutureListener remover;

    public ChannelLocal() {
        this(false);
    }

    public ChannelLocal(boolean removeOnClose2) {
        this.map = new ConcurrentIdentityWeakKeyHashMap();
        this.remover = new ChannelFutureListener() {
            public void operationComplete(ChannelFuture future) throws Exception {
                ChannelLocal.this.remove(future.getChannel());
            }
        };
        this.removeOnClose = removeOnClose2;
    }

    /* access modifiers changed from: protected */
    public T initialValue(Channel channel) {
        return null;
    }

    public T get(Channel channel) {
        T oldValue;
        if (channel != null) {
            T value = this.map.get(channel);
            if (value != null) {
                return value;
            }
            T value2 = initialValue(channel);
            if (value2 == null || (oldValue = setIfAbsent(channel, value2)) == null) {
                return value2;
            }
            return oldValue;
        }
        throw new NullPointerException("channel");
    }

    public T set(Channel channel, T value) {
        if (value == null) {
            return remove(channel);
        }
        if (channel != null) {
            T old = this.map.put(channel, value);
            if (this.removeOnClose) {
                channel.getCloseFuture().addListener(this.remover);
            }
            return old;
        }
        throw new NullPointerException("channel");
    }

    public T setIfAbsent(Channel channel, T value) {
        if (value == null) {
            return get(channel);
        }
        if (channel != null) {
            T mapping = this.map.putIfAbsent(channel, value);
            if (this.removeOnClose && mapping == null) {
                channel.getCloseFuture().addListener(this.remover);
            }
            return mapping;
        }
        throw new NullPointerException("channel");
    }

    public T remove(Channel channel) {
        if (channel != null) {
            T removed = this.map.remove(channel);
            if (removed == null) {
                return initialValue(channel);
            }
            if (this.removeOnClose) {
                channel.getCloseFuture().removeListener(this.remover);
            }
            return removed;
        }
        throw new NullPointerException("channel");
    }

    public Iterator<Map.Entry<Channel, T>> iterator() {
        return Collections.unmodifiableSet(this.map.entrySet()).iterator();
    }
}
