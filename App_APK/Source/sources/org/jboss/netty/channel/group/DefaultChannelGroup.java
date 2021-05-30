package org.jboss.netty.channel.group;

import java.net.SocketAddress;
import java.util.AbstractSet;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.concurrent.ConcurrentMap;
import java.util.concurrent.atomic.AtomicInteger;
import org.jboss.netty.buffer.ChannelBuffer;
import org.jboss.netty.channel.Channel;
import org.jboss.netty.channel.ChannelFuture;
import org.jboss.netty.channel.ChannelFutureListener;
import org.jboss.netty.channel.ServerChannel;
import org.jboss.netty.util.internal.ConcurrentHashMap;

public class DefaultChannelGroup extends AbstractSet<Channel> implements ChannelGroup {
    private static final AtomicInteger nextId = new AtomicInteger();
    private final String name;
    private final ConcurrentMap<Integer, Channel> nonServerChannels;
    private final ChannelFutureListener remover;
    private final ConcurrentMap<Integer, Channel> serverChannels;

    public DefaultChannelGroup() {
        this("group-0x" + Integer.toHexString(nextId.incrementAndGet()));
    }

    public DefaultChannelGroup(String name2) {
        this.serverChannels = new ConcurrentHashMap();
        this.nonServerChannels = new ConcurrentHashMap();
        this.remover = new ChannelFutureListener() {
            public void operationComplete(ChannelFuture future) throws Exception {
                DefaultChannelGroup.this.remove(future.getChannel());
            }
        };
        if (name2 != null) {
            this.name = name2;
            return;
        }
        throw new NullPointerException("name");
    }

    public String getName() {
        return this.name;
    }

    public boolean isEmpty() {
        return this.nonServerChannels.isEmpty() && this.serverChannels.isEmpty();
    }

    public int size() {
        return this.nonServerChannels.size() + this.serverChannels.size();
    }

    public Channel find(Integer id) {
        Channel c = (Channel) this.nonServerChannels.get(id);
        if (c != null) {
            return c;
        }
        return (Channel) this.serverChannels.get(id);
    }

    public boolean contains(Object o) {
        if (o instanceof Integer) {
            if (this.nonServerChannels.containsKey(o) || this.serverChannels.containsKey(o)) {
                return true;
            }
            return false;
        } else if (!(o instanceof Channel)) {
            return false;
        } else {
            Channel c = (Channel) o;
            if (o instanceof ServerChannel) {
                return this.serverChannels.containsKey(c.getId());
            }
            return this.nonServerChannels.containsKey(c.getId());
        }
    }

    public boolean add(Channel channel) {
        boolean added = (channel instanceof ServerChannel ? this.serverChannels : this.nonServerChannels).putIfAbsent(channel.getId(), channel) == null;
        if (added) {
            channel.getCloseFuture().addListener(this.remover);
        }
        return added;
    }

    /* JADX DEBUG: Multi-variable search result rejected for TypeSearchVarInfo{r1v7, resolved type: java.lang.Object} */
    /* JADX DEBUG: Multi-variable search result rejected for TypeSearchVarInfo{r0v4, resolved type: org.jboss.netty.channel.Channel} */
    /* JADX DEBUG: Multi-variable search result rejected for TypeSearchVarInfo{r1v9, resolved type: java.lang.Object} */
    /* JADX DEBUG: Multi-variable search result rejected for TypeSearchVarInfo{r0v6, resolved type: org.jboss.netty.channel.Channel} */
    /* JADX DEBUG: Multi-variable search result rejected for TypeSearchVarInfo{r1v11, resolved type: java.lang.Object} */
    /* JADX DEBUG: Multi-variable search result rejected for TypeSearchVarInfo{r0v8, resolved type: org.jboss.netty.channel.Channel} */
    /* JADX DEBUG: Multi-variable search result rejected for TypeSearchVarInfo{r1v13, resolved type: java.lang.Object} */
    /* JADX DEBUG: Multi-variable search result rejected for TypeSearchVarInfo{r0v10, resolved type: org.jboss.netty.channel.Channel} */
    /* JADX WARNING: Multi-variable type inference failed */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public boolean remove(java.lang.Object r4) {
        /*
            r3 = this;
            r0 = 0
            boolean r1 = r4 instanceof java.lang.Integer
            if (r1 == 0) goto L_0x001a
            java.util.concurrent.ConcurrentMap<java.lang.Integer, org.jboss.netty.channel.Channel> r1 = r3.nonServerChannels
            java.lang.Object r1 = r1.remove(r4)
            r0 = r1
            org.jboss.netty.channel.Channel r0 = (org.jboss.netty.channel.Channel) r0
            if (r0 != 0) goto L_0x0040
            java.util.concurrent.ConcurrentMap<java.lang.Integer, org.jboss.netty.channel.Channel> r1 = r3.serverChannels
            java.lang.Object r1 = r1.remove(r4)
            r0 = r1
            org.jboss.netty.channel.Channel r0 = (org.jboss.netty.channel.Channel) r0
            goto L_0x0040
        L_0x001a:
            boolean r1 = r4 instanceof org.jboss.netty.channel.Channel
            if (r1 == 0) goto L_0x0040
            r0 = r4
            org.jboss.netty.channel.Channel r0 = (org.jboss.netty.channel.Channel) r0
            boolean r1 = r0 instanceof org.jboss.netty.channel.ServerChannel
            if (r1 == 0) goto L_0x0033
            java.util.concurrent.ConcurrentMap<java.lang.Integer, org.jboss.netty.channel.Channel> r1 = r3.serverChannels
            java.lang.Integer r2 = r0.getId()
            java.lang.Object r1 = r1.remove(r2)
            r0 = r1
            org.jboss.netty.channel.Channel r0 = (org.jboss.netty.channel.Channel) r0
            goto L_0x0040
        L_0x0033:
            java.util.concurrent.ConcurrentMap<java.lang.Integer, org.jboss.netty.channel.Channel> r1 = r3.nonServerChannels
            java.lang.Integer r2 = r0.getId()
            java.lang.Object r1 = r1.remove(r2)
            r0 = r1
            org.jboss.netty.channel.Channel r0 = (org.jboss.netty.channel.Channel) r0
        L_0x0040:
            if (r0 != 0) goto L_0x0044
            r1 = 0
            return r1
        L_0x0044:
            org.jboss.netty.channel.ChannelFuture r1 = r0.getCloseFuture()
            org.jboss.netty.channel.ChannelFutureListener r2 = r3.remover
            r1.removeListener(r2)
            r1 = 1
            return r1
        */
        throw new UnsupportedOperationException("Method not decompiled: org.jboss.netty.channel.group.DefaultChannelGroup.remove(java.lang.Object):boolean");
    }

    public void clear() {
        this.nonServerChannels.clear();
        this.serverChannels.clear();
    }

    public Iterator<Channel> iterator() {
        return new CombinedIterator(this.serverChannels.values().iterator(), this.nonServerChannels.values().iterator());
    }

    public Object[] toArray() {
        Collection<Channel> channels = new ArrayList<>(size());
        channels.addAll(this.serverChannels.values());
        channels.addAll(this.nonServerChannels.values());
        return channels.toArray();
    }

    public <T> T[] toArray(T[] a) {
        Collection<Channel> channels = new ArrayList<>(size());
        channels.addAll(this.serverChannels.values());
        channels.addAll(this.nonServerChannels.values());
        return channels.toArray(a);
    }

    public ChannelGroupFuture close() {
        Map<Integer, ChannelFuture> futures = new LinkedHashMap<>(size());
        for (Channel c : this.serverChannels.values()) {
            futures.put(c.getId(), c.close().awaitUninterruptibly());
        }
        for (Channel c2 : this.nonServerChannels.values()) {
            futures.put(c2.getId(), c2.close());
        }
        return new DefaultChannelGroupFuture((ChannelGroup) this, futures);
    }

    public ChannelGroupFuture disconnect() {
        Map<Integer, ChannelFuture> futures = new LinkedHashMap<>(size());
        for (Channel c : this.serverChannels.values()) {
            futures.put(c.getId(), c.disconnect().awaitUninterruptibly());
        }
        for (Channel c2 : this.nonServerChannels.values()) {
            futures.put(c2.getId(), c2.disconnect());
        }
        return new DefaultChannelGroupFuture((ChannelGroup) this, futures);
    }

    public ChannelGroupFuture setInterestOps(int interestOps) {
        Map<Integer, ChannelFuture> futures = new LinkedHashMap<>(size());
        for (Channel c : this.serverChannels.values()) {
            futures.put(c.getId(), c.setInterestOps(interestOps).awaitUninterruptibly());
        }
        for (Channel c2 : this.nonServerChannels.values()) {
            futures.put(c2.getId(), c2.setInterestOps(interestOps));
        }
        return new DefaultChannelGroupFuture((ChannelGroup) this, futures);
    }

    public ChannelGroupFuture setReadable(boolean readable) {
        Map<Integer, ChannelFuture> futures = new LinkedHashMap<>(size());
        for (Channel c : this.serverChannels.values()) {
            futures.put(c.getId(), c.setReadable(readable).awaitUninterruptibly());
        }
        for (Channel c2 : this.nonServerChannels.values()) {
            futures.put(c2.getId(), c2.setReadable(readable));
        }
        return new DefaultChannelGroupFuture((ChannelGroup) this, futures);
    }

    public ChannelGroupFuture unbind() {
        Map<Integer, ChannelFuture> futures = new LinkedHashMap<>(size());
        for (Channel c : this.serverChannels.values()) {
            futures.put(c.getId(), c.unbind().awaitUninterruptibly());
        }
        for (Channel c2 : this.nonServerChannels.values()) {
            futures.put(c2.getId(), c2.unbind());
        }
        return new DefaultChannelGroupFuture((ChannelGroup) this, futures);
    }

    public ChannelGroupFuture write(Object message) {
        Map<Integer, ChannelFuture> futures = new LinkedHashMap<>(size());
        if (message instanceof ChannelBuffer) {
            ChannelBuffer buf = (ChannelBuffer) message;
            for (Channel c : this.nonServerChannels.values()) {
                futures.put(c.getId(), c.write(buf.duplicate()));
            }
        } else {
            for (Channel c2 : this.nonServerChannels.values()) {
                futures.put(c2.getId(), c2.write(message));
            }
        }
        return new DefaultChannelGroupFuture((ChannelGroup) this, futures);
    }

    public ChannelGroupFuture write(Object message, SocketAddress remoteAddress) {
        Map<Integer, ChannelFuture> futures = new LinkedHashMap<>(size());
        if (message instanceof ChannelBuffer) {
            ChannelBuffer buf = (ChannelBuffer) message;
            for (Channel c : this.nonServerChannels.values()) {
                futures.put(c.getId(), c.write(buf.duplicate(), remoteAddress));
            }
        } else {
            for (Channel c2 : this.nonServerChannels.values()) {
                futures.put(c2.getId(), c2.write(message, remoteAddress));
            }
        }
        return new DefaultChannelGroupFuture((ChannelGroup) this, futures);
    }

    public int hashCode() {
        return System.identityHashCode(this);
    }

    public boolean equals(Object o) {
        return this == o;
    }

    public int compareTo(ChannelGroup o) {
        int v = getName().compareTo(o.getName());
        if (v != 0) {
            return v;
        }
        return System.identityHashCode(this) - System.identityHashCode(o);
    }

    public String toString() {
        return getClass().getSimpleName() + "(name: " + getName() + ", size: " + size() + ')';
    }
}
