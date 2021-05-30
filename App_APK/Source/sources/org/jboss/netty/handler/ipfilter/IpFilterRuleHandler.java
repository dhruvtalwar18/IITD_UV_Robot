package org.jboss.netty.handler.ipfilter;

import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.util.Collection;
import java.util.Iterator;
import java.util.List;
import java.util.concurrent.CopyOnWriteArrayList;
import org.jboss.netty.channel.ChannelEvent;
import org.jboss.netty.channel.ChannelHandler;
import org.jboss.netty.channel.ChannelHandlerContext;

@ChannelHandler.Sharable
public class IpFilterRuleHandler extends IpFilteringHandlerImpl {
    private final CopyOnWriteArrayList<IpFilterRule> ipFilterRuleList = new CopyOnWriteArrayList<>();

    public IpFilterRuleHandler(List<IpFilterRule> newList) {
        if (newList != null) {
            this.ipFilterRuleList.addAll(newList);
        }
    }

    public IpFilterRuleHandler() {
    }

    public void add(IpFilterRule ipFilterRule) {
        if (ipFilterRule != null) {
            this.ipFilterRuleList.add(ipFilterRule);
            return;
        }
        throw new NullPointerException("IpFilterRule can not be null");
    }

    public void add(int index, IpFilterRule ipFilterRule) {
        if (ipFilterRule != null) {
            this.ipFilterRuleList.add(index, ipFilterRule);
            return;
        }
        throw new NullPointerException("IpFilterRule can not be null");
    }

    public void addAll(Collection<IpFilterRule> c) {
        if (c != null) {
            this.ipFilterRuleList.addAll(c);
            return;
        }
        throw new NullPointerException("Collection can not be null");
    }

    public void addAll(int index, Collection<IpFilterRule> c) {
        if (c != null) {
            this.ipFilterRuleList.addAll(index, c);
            return;
        }
        throw new NullPointerException("Collection can not be null");
    }

    public int addAllAbsent(Collection<IpFilterRule> c) {
        if (c != null) {
            return this.ipFilterRuleList.addAllAbsent(c);
        }
        throw new NullPointerException("Collection can not be null");
    }

    public boolean addIfAbsent(IpFilterRule ipFilterRule) {
        if (ipFilterRule != null) {
            return this.ipFilterRuleList.addIfAbsent(ipFilterRule);
        }
        throw new NullPointerException("IpFilterRule can not be null");
    }

    public void clear() {
        this.ipFilterRuleList.clear();
    }

    public boolean contains(IpFilterRule ipFilterRule) {
        if (ipFilterRule != null) {
            return this.ipFilterRuleList.contains(ipFilterRule);
        }
        throw new NullPointerException("IpFilterRule can not be null");
    }

    public boolean containsAll(Collection<IpFilterRule> c) {
        if (c != null) {
            return this.ipFilterRuleList.containsAll(c);
        }
        throw new NullPointerException("Collection can not be null");
    }

    public IpFilterRule get(int index) {
        return this.ipFilterRuleList.get(index);
    }

    public boolean isEmpty() {
        return this.ipFilterRuleList.isEmpty();
    }

    public void remove(IpFilterRule ipFilterRule) {
        if (ipFilterRule != null) {
            this.ipFilterRuleList.remove(ipFilterRule);
            return;
        }
        throw new NullPointerException("IpFilterRule can not be null");
    }

    public IpFilterRule remove(int index) {
        return this.ipFilterRuleList.remove(index);
    }

    public void removeAll(Collection<IpFilterRule> c) {
        if (c != null) {
            this.ipFilterRuleList.removeAll(c);
            return;
        }
        throw new NullPointerException("Collection can not be null");
    }

    public void retainAll(Collection<IpFilterRule> c) {
        if (c != null) {
            this.ipFilterRuleList.retainAll(c);
            return;
        }
        throw new NullPointerException("Collection can not be null");
    }

    public IpFilterRule set(int index, IpFilterRule ipFilterRule) {
        if (ipFilterRule != null) {
            return this.ipFilterRuleList.set(index, ipFilterRule);
        }
        throw new NullPointerException("IpFilterRule can not be null");
    }

    public int size() {
        return this.ipFilterRuleList.size();
    }

    /* access modifiers changed from: protected */
    public boolean accept(ChannelHandlerContext ctx, ChannelEvent e, InetSocketAddress inetSocketAddress) throws Exception {
        if (this.ipFilterRuleList.isEmpty()) {
            return true;
        }
        InetAddress inetAddress = inetSocketAddress.getAddress();
        Iterator<IpFilterRule> iterator = this.ipFilterRuleList.iterator();
        while (iterator.hasNext()) {
            IpFilterRule ipFilterRule = iterator.next();
            if (ipFilterRule.contains(inetAddress)) {
                return ipFilterRule.isAllowRule();
            }
        }
        return true;
    }
}
