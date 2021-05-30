package org.bytedeco.javacpp;

import java.util.ArrayDeque;
import java.util.Deque;
import org.bytedeco.javacpp.tools.Logger;

public class PointerScope implements AutoCloseable {
    private static final Logger logger = Logger.create(PointerScope.class);
    static final ThreadLocal<Deque<PointerScope>> scopeStack = new ThreadLocal<Deque<PointerScope>>() {
        /* access modifiers changed from: protected */
        public Deque initialValue() {
            return new ArrayDeque();
        }
    };
    boolean deallocateOnClose;
    Deque<Pointer> pointerStack;

    public static PointerScope getInnerScope() {
        return (PointerScope) scopeStack.get().peek();
    }

    public PointerScope() {
        this(true);
    }

    public PointerScope(boolean deallocateOnClose2) {
        this.pointerStack = new ArrayDeque();
        this.deallocateOnClose = true;
        if (logger.isDebugEnabled()) {
            Logger logger2 = logger;
            logger2.debug("Opening " + this);
        }
        this.deallocateOnClose = deallocateOnClose2;
        scopeStack.get().push(this);
    }

    public PointerScope deallocateOnClose(boolean deallocateOnClose2) {
        this.deallocateOnClose = deallocateOnClose2;
        return this;
    }

    public boolean deallocateOnClose() {
        return this.deallocateOnClose;
    }

    public PointerScope attach(Pointer p) {
        if (logger.isDebugEnabled()) {
            Logger logger2 = logger;
            logger2.debug("Attaching " + p + " to " + this);
        }
        this.pointerStack.push(p);
        return this;
    }

    public PointerScope detach(Pointer p) {
        if (logger.isDebugEnabled()) {
            Logger logger2 = logger;
            logger2.debug("Detaching " + p + " from " + this);
        }
        this.pointerStack.remove(p);
        return this;
    }

    public void close() {
        if (logger.isDebugEnabled()) {
            Logger logger2 = logger;
            logger2.debug("Closing " + this);
        }
        if (deallocateOnClose()) {
            deallocate();
        }
        scopeStack.get().remove(this);
    }

    public void deallocate() {
        if (logger.isDebugEnabled()) {
            Logger logger2 = logger;
            logger2.debug("Deallocating " + this);
        }
        while (this.pointerStack.size() > 0) {
            this.pointerStack.pop().deallocate();
        }
        this.pointerStack = null;
    }
}
