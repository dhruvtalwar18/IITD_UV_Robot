package org.jboss.netty.handler.execution;

import java.util.concurrent.Executor;
import org.jboss.netty.util.ExternalResourceReleasable;
import org.jboss.netty.util.internal.ExecutorUtil;

public class ChainedExecutor implements Executor, ExternalResourceReleasable {
    static final /* synthetic */ boolean $assertionsDisabled = false;
    private final Executor cur;
    private final ChannelEventRunnableFilter filter;
    private final Executor next;

    public ChainedExecutor(ChannelEventRunnableFilter filter2, Executor cur2, Executor next2) {
        if (filter2 == null) {
            throw new NullPointerException("filter");
        } else if (cur2 == null) {
            throw new NullPointerException("cur");
        } else if (next2 != null) {
            this.filter = filter2;
            this.cur = cur2;
            this.next = next2;
        } else {
            throw new NullPointerException("next");
        }
    }

    public void execute(Runnable command) {
        if (this.filter.filter((ChannelEventRunnable) command)) {
            this.cur.execute(command);
        } else {
            this.next.execute(command);
        }
    }

    public void releaseExternalResources() {
        ExecutorUtil.terminate(this.cur, this.next);
        releaseExternal(this.cur);
        releaseExternal(this.next);
    }

    private static void releaseExternal(Executor executor) {
        if (executor instanceof ExternalResourceReleasable) {
            ((ExternalResourceReleasable) executor).releaseExternalResources();
        }
    }
}
