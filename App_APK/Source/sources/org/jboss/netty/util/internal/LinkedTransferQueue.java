package org.jboss.netty.util.internal;

import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.io.Serializable;
import java.lang.reflect.Field;
import java.security.AccessController;
import java.security.PrivilegedActionException;
import java.security.PrivilegedExceptionAction;
import java.util.AbstractQueue;
import java.util.Collection;
import java.util.Iterator;
import java.util.NoSuchElementException;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.locks.LockSupport;
import sun.misc.Unsafe;

public class LinkedTransferQueue<E> extends AbstractQueue<E> implements BlockingQueue<E>, Serializable {
    private static final int ASYNC = 1;
    private static final int CHAINED_SPINS = 64;
    private static final int FRONT_SPINS = 128;
    private static final boolean MP;
    private static final int NOW = 0;
    static final int SWEEP_THRESHOLD = 32;
    private static final int SYNC = 2;
    private static final int TIMED = 3;
    private static final Unsafe UNSAFE;
    private static final long headOffset;
    private static final long serialVersionUID = -3223113410248163686L;
    private static final long sweepVotesOffset;
    private static final long tailOffset;
    volatile transient Node head;
    private volatile transient int sweepVotes;
    private volatile transient Node tail;

    static {
        boolean z = true;
        if (Runtime.getRuntime().availableProcessors() <= 1) {
            z = false;
        }
        MP = z;
        try {
            UNSAFE = getUnsafe();
            Class<LinkedTransferQueue> cls = LinkedTransferQueue.class;
            headOffset = UNSAFE.objectFieldOffset(cls.getDeclaredField("head"));
            tailOffset = UNSAFE.objectFieldOffset(cls.getDeclaredField("tail"));
            sweepVotesOffset = UNSAFE.objectFieldOffset(cls.getDeclaredField("sweepVotes"));
        } catch (Exception e) {
            throw new Error(e);
        }
    }

    static final class Node {
        private static final Unsafe UNSAFE;
        private static final long itemOffset;
        private static final long nextOffset;
        private static final long waiterOffset;
        final boolean isData;
        volatile Object item;
        volatile Node next;
        volatile Thread waiter;

        /* access modifiers changed from: package-private */
        public final boolean casNext(Node cmp, Node val) {
            return UNSAFE.compareAndSwapObject(this, nextOffset, cmp, val);
        }

        /* access modifiers changed from: package-private */
        public final boolean casItem(Object cmp, Object val) {
            return UNSAFE.compareAndSwapObject(this, itemOffset, cmp, val);
        }

        Node(Object item2, boolean isData2) {
            UNSAFE.putObject(this, itemOffset, item2);
            this.isData = isData2;
        }

        /* access modifiers changed from: package-private */
        public final void forgetNext() {
            UNSAFE.putObject(this, nextOffset, this);
        }

        /* access modifiers changed from: package-private */
        public final void forgetContents() {
            UNSAFE.putObject(this, itemOffset, this);
            UNSAFE.putObject(this, waiterOffset, (Object) null);
        }

        /* access modifiers changed from: package-private */
        public final boolean isMatched() {
            Object x = this.item;
            if (x != this) {
                return (x == null) == this.isData;
            }
        }

        /* access modifiers changed from: package-private */
        public final boolean isUnmatchedRequest() {
            return !this.isData && this.item == null;
        }

        /* access modifiers changed from: package-private */
        public final boolean cannotPrecede(boolean haveData) {
            boolean d = this.isData;
            if (d != haveData) {
                Object obj = this.item;
                Object x = obj;
                if (obj != this) {
                    if ((x != null) == d) {
                        return true;
                    }
                }
            }
            return false;
        }

        /* access modifiers changed from: package-private */
        public final boolean tryMatchData() {
            Object x = this.item;
            if (x == null || x == this || !casItem(x, (Object) null)) {
                return false;
            }
            LockSupport.unpark(this.waiter);
            return true;
        }

        static {
            try {
                UNSAFE = LinkedTransferQueue.getUnsafe();
                Class<Node> cls = Node.class;
                itemOffset = UNSAFE.objectFieldOffset(cls.getDeclaredField("item"));
                nextOffset = UNSAFE.objectFieldOffset(cls.getDeclaredField("next"));
                waiterOffset = UNSAFE.objectFieldOffset(cls.getDeclaredField("waiter"));
            } catch (Exception e) {
                throw new Error(e);
            }
        }
    }

    private boolean casTail(Node cmp, Node val) {
        return UNSAFE.compareAndSwapObject(this, tailOffset, cmp, val);
    }

    private boolean casHead(Node cmp, Node val) {
        return UNSAFE.compareAndSwapObject(this, headOffset, cmp, val);
    }

    private boolean casSweepVotes(int cmp, int val) {
        return UNSAFE.compareAndSwapInt(this, sweepVotesOffset, cmp, val);
    }

    static <E> E cast(Object item) {
        return item;
    }

    private E xfer(E e, boolean haveData, int how, long nanos) {
        boolean z;
        Node node;
        E e2 = e;
        boolean z2 = haveData;
        int i = how;
        if (!z2 || e2 != null) {
            Node s = null;
            while (true) {
                Node p = this.head;
                Node h = p;
                while (true) {
                    z = true;
                    if (p == null) {
                        break;
                    }
                    boolean isData = p.isData;
                    Object item = p.item;
                    if (item != p) {
                        if ((item != null) == isData) {
                            if (isData == z2) {
                                break;
                            } else if (p.casItem(item, e)) {
                                Node h2 = h;
                                Node q = p;
                                while (true) {
                                    if (q == h2) {
                                        break;
                                    }
                                    Node n = q.next;
                                    if (this.head == h2) {
                                        if (casHead(h2, n == null ? q : n)) {
                                            h2.forgetNext();
                                            break;
                                        }
                                    }
                                    Node node2 = this.head;
                                    h2 = node2;
                                    if (node2 == null) {
                                        break;
                                    }
                                    Node node3 = h2.next;
                                    q = node3;
                                    if (node3 != null) {
                                        if (!q.isMatched()) {
                                            break;
                                        }
                                    } else {
                                        break;
                                    }
                                }
                                LockSupport.unpark(p.waiter);
                                return cast(item);
                            }
                        }
                    }
                    Node n2 = p.next;
                    if (p != n2) {
                        node = n2;
                    } else {
                        node = this.head;
                        h = node;
                    }
                    p = node;
                }
                if (i == 0) {
                    break;
                }
                if (s == null) {
                    s = new Node(e, haveData);
                }
                Node s2 = s;
                Node pred = tryAppend(s2, haveData);
                if (pred == null) {
                    s = s2;
                } else if (i != 1) {
                    if (i != 3) {
                        z = false;
                    }
                    return awaitMatch(s2, pred, e, z, nanos);
                }
            }
            return e2;
        }
        throw new NullPointerException();
    }

    private Node tryAppend(Node s, boolean haveData) {
        Node p = this.tail;
        Node t = p;
        while (true) {
            Node node = null;
            if (p == null) {
                Node node2 = this.head;
                p = node2;
                if (node2 == null) {
                    if (casHead((Node) null, s)) {
                        return s;
                    }
                }
            }
            if (p.cannotPrecede(haveData)) {
                return null;
            }
            Node node3 = p.next;
            Node n = node3;
            if (node3 != null) {
                if (p != t) {
                    Node node4 = this.tail;
                    Node u = node4;
                    if (t != node4) {
                        t = u;
                        node = u;
                        p = node;
                    }
                }
                if (p != n) {
                    node = n;
                }
                p = node;
            } else if (!p.casNext((Node) null, s)) {
                p = p.next;
            } else {
                if (p != t) {
                    do {
                        if (this.tail == t && casTail(t, s)) {
                            break;
                        }
                        Node node5 = this.tail;
                        t = node5;
                        if (node5 == null) {
                            break;
                        }
                        Node node6 = t.next;
                        Node s2 = node6;
                        if (node6 == null) {
                            break;
                        }
                        Node node7 = s2.next;
                        s = node7;
                        if (node7 == null) {
                            break;
                        }
                    } while (s != t);
                }
                return p;
            }
        }
    }

    private E awaitMatch(Node s, Node pred, E e, boolean timed, long nanos) {
        Node node = s;
        Node node2 = pred;
        Object obj = e;
        long lastTime = timed ? System.nanoTime() : 0;
        Thread w = Thread.currentThread();
        int spins = -1;
        ThreadLocalRandom randomYields = null;
        long lastTime2 = lastTime;
        long nanos2 = nanos;
        while (true) {
            Object item = node.item;
            if (item != obj) {
                s.forgetContents();
                return cast(item);
            } else if ((w.isInterrupted() || (timed && nanos2 <= 0)) && node.casItem(obj, node)) {
                unsplice(node2, node);
                return obj;
            } else {
                if (spins < 0) {
                    int spinsFor = spinsFor(node2, node.isData);
                    spins = spinsFor;
                    if (spinsFor > 0) {
                        randomYields = ThreadLocalRandom.current();
                    }
                } else if (spins > 0) {
                    spins--;
                    if (randomYields.nextInt(64) == 0) {
                        Thread.yield();
                    }
                } else if (node.waiter == null) {
                    node.waiter = w;
                } else if (timed) {
                    long now = System.nanoTime();
                    long j = nanos2 - (now - lastTime2);
                    nanos2 = j;
                    if (j > 0) {
                        LockSupport.parkNanos(nanos2);
                    }
                    lastTime2 = now;
                } else {
                    LockSupport.park();
                }
            }
        }
    }

    private static int spinsFor(Node pred, boolean haveData) {
        if (!MP || pred == null) {
            return 0;
        }
        if (pred.isData != haveData) {
            return 192;
        }
        if (pred.isMatched()) {
            return 128;
        }
        if (pred.waiter == null) {
            return 64;
        }
        return 0;
    }

    /* access modifiers changed from: package-private */
    public final Node succ(Node p) {
        Node next = p.next;
        return p == next ? this.head : next;
    }

    private Node firstOfMode(boolean isData) {
        Node p = this.head;
        while (p != null) {
            if (p.isMatched()) {
                p = succ(p);
            } else if (p.isData == isData) {
                return p;
            } else {
                return null;
            }
        }
        return null;
    }

    private E firstDataItem() {
        Node p = this.head;
        while (p != null) {
            Object item = p.item;
            if (p.isData) {
                if (!(item == null || item == p)) {
                    return cast(item);
                }
            } else if (item == null) {
                return null;
            }
            p = succ(p);
        }
        return null;
    }

    private int countOfMode(boolean data) {
        int count = 0;
        Node p = this.head;
        while (p != null) {
            if (!p.isMatched()) {
                if (p.isData == data) {
                    count++;
                    if (count == Integer.MAX_VALUE) {
                        break;
                    }
                } else {
                    return 0;
                }
            }
            Node n = p.next;
            if (n != p) {
                p = n;
            } else {
                count = 0;
                p = this.head;
            }
        }
        return count;
    }

    final class Itr implements Iterator<E> {
        private Node lastPred;
        private Node lastRet;
        private E nextItem;
        private Node nextNode;

        private void advance(Node prev) {
            Node node = this.lastRet;
            Node r = node;
            if (node == null || r.isMatched()) {
                Node node2 = this.lastPred;
                Node b = node2;
                if (node2 != null && !b.isMatched()) {
                    while (true) {
                        Node node3 = b.next;
                        Node s = node3;
                        if (node3 != null && s != b && s.isMatched()) {
                            Node node4 = s.next;
                            Node n = node4;
                            if (node4 == null || n == s) {
                                break;
                            }
                            b.casNext(s, n);
                        } else {
                            break;
                        }
                    }
                } else {
                    this.lastPred = null;
                }
            } else {
                this.lastPred = r;
            }
            this.lastRet = prev;
            Node p = prev;
            while (true) {
                Node s2 = p == null ? LinkedTransferQueue.this.head : p.next;
                if (s2 == null) {
                    break;
                } else if (s2 == p) {
                    p = null;
                } else {
                    Object item = s2.item;
                    if (!s2.isData) {
                        if (item == null) {
                            break;
                        }
                    } else if (!(item == null || item == s2)) {
                        this.nextItem = LinkedTransferQueue.cast(item);
                        this.nextNode = s2;
                        return;
                    }
                    if (p == null) {
                        p = s2;
                    } else {
                        Node node5 = s2.next;
                        Node n2 = node5;
                        if (node5 == null) {
                            break;
                        } else if (s2 == n2) {
                            p = null;
                        } else {
                            p.casNext(s2, n2);
                        }
                    }
                }
            }
            this.nextNode = null;
            this.nextItem = null;
        }

        Itr() {
            advance((Node) null);
        }

        public final boolean hasNext() {
            return this.nextNode != null;
        }

        public final E next() {
            Node p = this.nextNode;
            if (p != null) {
                E e = this.nextItem;
                advance(p);
                return e;
            }
            throw new NoSuchElementException();
        }

        public final void remove() {
            Node lastRet2 = this.lastRet;
            if (lastRet2 != null) {
                this.lastRet = null;
                if (lastRet2.tryMatchData()) {
                    LinkedTransferQueue.this.unsplice(this.lastPred, lastRet2);
                    return;
                }
                return;
            }
            throw new IllegalStateException();
        }
    }

    /* access modifiers changed from: package-private */
    public final void unsplice(Node pred, Node s) {
        s.forgetContents();
        if (pred != null && pred != s && pred.next == s) {
            Node n = s.next;
            if (n == null || (n != s && pred.casNext(s, n) && pred.isMatched())) {
                while (true) {
                    Node h = this.head;
                    if (h != pred && h != s && h != null) {
                        if (h.isMatched()) {
                            Node hn = h.next;
                            if (hn != null) {
                                if (hn != h && casHead(h, hn)) {
                                    h.forgetNext();
                                }
                            } else {
                                return;
                            }
                        } else if (pred.next != pred && s.next != s) {
                            while (true) {
                                int v = this.sweepVotes;
                                if (v < 32) {
                                    if (casSweepVotes(v, v + 1)) {
                                        return;
                                    }
                                } else if (casSweepVotes(v, 0)) {
                                    sweep();
                                    return;
                                }
                            }
                        } else {
                            return;
                        }
                    } else {
                        return;
                    }
                }
            }
        }
    }

    private void sweep() {
        Node p = this.head;
        while (p != null) {
            Node node = p.next;
            Node s = node;
            if (node == null) {
                return;
            }
            if (!s.isMatched()) {
                p = s;
            } else {
                Node node2 = s.next;
                Node n = node2;
                if (node2 != null) {
                    if (s == n) {
                        p = this.head;
                    } else {
                        p.casNext(s, n);
                    }
                } else {
                    return;
                }
            }
        }
    }

    private boolean findAndRemove(Object e) {
        if (e == null) {
            return false;
        }
        Node pred = null;
        Node p = this.head;
        while (p != null) {
            Object item = p.item;
            if (p.isData) {
                if (item != null && item != p && e.equals(item) && p.tryMatchData()) {
                    unsplice(pred, p);
                    return true;
                }
            } else if (item == null) {
                return false;
            }
            pred = p;
            Node node = p.next;
            p = node;
            if (node == pred) {
                pred = null;
                p = this.head;
            }
        }
        return false;
    }

    public LinkedTransferQueue() {
    }

    public LinkedTransferQueue(Collection<? extends E> c) {
        this();
        addAll(c);
    }

    public void put(E e) {
        xfer(e, true, 1, 0);
    }

    public boolean offer(E e, long timeout, TimeUnit unit) {
        xfer(e, true, 1, 0);
        return true;
    }

    public boolean offer(E e) {
        xfer(e, true, 1, 0);
        return true;
    }

    public boolean add(E e) {
        xfer(e, true, 1, 0);
        return true;
    }

    public boolean tryTransfer(E e) {
        return xfer(e, true, 0, 0) == null;
    }

    public void transfer(E e) throws InterruptedException {
        if (xfer(e, true, 2, 0) != null) {
            Thread.interrupted();
            throw new InterruptedException();
        }
    }

    public boolean tryTransfer(E e, long timeout, TimeUnit unit) throws InterruptedException {
        if (xfer(e, true, 3, unit.toNanos(timeout)) == null) {
            return true;
        }
        if (!Thread.interrupted()) {
            return false;
        }
        throw new InterruptedException();
    }

    public E take() throws InterruptedException {
        E e = xfer((Object) null, false, 2, 0);
        if (e != null) {
            return e;
        }
        Thread.interrupted();
        throw new InterruptedException();
    }

    public E poll(long timeout, TimeUnit unit) throws InterruptedException {
        E e = xfer((Object) null, false, 3, unit.toNanos(timeout));
        if (e != null || !Thread.interrupted()) {
            return e;
        }
        throw new InterruptedException();
    }

    public E poll() {
        return xfer((Object) null, false, 0, 0);
    }

    public int drainTo(Collection<? super E> c) {
        if (c == null) {
            throw new NullPointerException();
        } else if (c != this) {
            int n = 0;
            while (true) {
                E poll = poll();
                E e = poll;
                if (poll == null) {
                    return n;
                }
                c.add(e);
                n++;
            }
        } else {
            throw new IllegalArgumentException();
        }
    }

    public int drainTo(Collection<? super E> c, int maxElements) {
        if (c == null) {
            throw new NullPointerException();
        } else if (c != this) {
            int n = 0;
            while (n < maxElements) {
                E poll = poll();
                E e = poll;
                if (poll == null) {
                    break;
                }
                c.add(e);
                n++;
            }
            return n;
        } else {
            throw new IllegalArgumentException();
        }
    }

    public Iterator<E> iterator() {
        return new Itr();
    }

    public E peek() {
        return firstDataItem();
    }

    public boolean isEmpty() {
        Node p = this.head;
        while (p != null) {
            if (!p.isMatched()) {
                return true ^ p.isData;
            }
            p = succ(p);
        }
        return true;
    }

    public boolean hasWaitingConsumer() {
        return firstOfMode(false) != null;
    }

    public int size() {
        return countOfMode(true);
    }

    public int getWaitingConsumerCount() {
        return countOfMode(false);
    }

    public boolean remove(Object o) {
        return findAndRemove(o);
    }

    public boolean contains(Object o) {
        if (o == null) {
            return false;
        }
        Node p = this.head;
        while (p != null) {
            Object item = p.item;
            if (p.isData) {
                if (!(item == null || item == p || !o.equals(item))) {
                    return true;
                }
            } else if (item == null) {
                break;
            }
            p = succ(p);
        }
        return false;
    }

    public int remainingCapacity() {
        return Integer.MAX_VALUE;
    }

    private void writeObject(ObjectOutputStream s) throws IOException {
        s.defaultWriteObject();
        Iterator i$ = iterator();
        while (i$.hasNext()) {
            s.writeObject(i$.next());
        }
        s.writeObject((Object) null);
    }

    private void readObject(ObjectInputStream s) throws IOException, ClassNotFoundException {
        s.defaultReadObject();
        while (true) {
            E item = s.readObject();
            if (item != null) {
                offer(item);
            } else {
                return;
            }
        }
    }

    static Unsafe getUnsafe() {
        try {
            return Unsafe.getUnsafe();
        } catch (SecurityException e) {
            try {
                return (Unsafe) AccessController.doPrivileged(new PrivilegedExceptionAction<Unsafe>() {
                    public Unsafe run() throws Exception {
                        Field f = Unsafe.class.getDeclaredField("theUnsafe");
                        f.setAccessible(true);
                        return (Unsafe) f.get((Object) null);
                    }
                });
            } catch (PrivilegedActionException e2) {
                throw new RuntimeException("Could not initialize intrinsics", e2.getCause());
            }
        }
    }
}