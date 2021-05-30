package org.bytedeco.javacpp;

import java.lang.ref.PhantomReference;
import java.lang.ref.ReferenceQueue;
import java.lang.reflect.Constructor;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.nio.Buffer;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import org.apache.commons.httpclient.HttpState;
import org.bytedeco.javacpp.annotation.Name;
import org.bytedeco.javacpp.annotation.Platform;
import org.bytedeco.javacpp.tools.Logger;

@Platform
public class Pointer implements AutoCloseable {
    static final Thread deallocatorThread;
    /* access modifiers changed from: private */
    public static final Logger logger = Logger.create(Pointer.class);
    static final long maxBytes;
    static final long maxPhysicalBytes;
    static final int maxRetries;
    /* access modifiers changed from: private */
    public static final ReferenceQueue<Pointer> referenceQueue;
    protected long address = 0;
    protected long capacity = 0;
    private Deallocator deallocator = null;
    protected long limit = 0;
    protected long position = 0;

    protected interface Deallocator {
        void deallocate();
    }

    private native void allocate(Buffer buffer);

    private native ByteBuffer asDirectBuffer();

    @Name({"JavaCPP_availablePhysicalBytes"})
    public static native long availablePhysicalBytes();

    public static native Pointer calloc(long j, long j2);

    public static native void free(Pointer pointer);

    public static native Pointer malloc(long j);

    public static native Pointer memchr(Pointer pointer, int i, long j);

    public static native int memcmp(Pointer pointer, Pointer pointer2, long j);

    public static native Pointer memcpy(Pointer pointer, Pointer pointer2, long j);

    public static native Pointer memmove(Pointer pointer, Pointer pointer2, long j);

    public static native Pointer memset(Pointer pointer, int i, long j);

    @Name({"JavaCPP_physicalBytes"})
    public static native long physicalBytes();

    public static native Pointer realloc(Pointer pointer, long j);

    @Name({"JavaCPP_totalPhysicalBytes"})
    public static native long totalPhysicalBytes();

    @Name({"JavaCPP_trimMemory"})
    private static native boolean trimMemory();

    public Pointer() {
    }

    public Pointer(final Pointer p) {
        if (p != null) {
            this.address = p.address;
            this.position = p.position;
            this.limit = p.limit;
            this.capacity = p.capacity;
            if (p.deallocator != null) {
                this.deallocator = new Deallocator() {
                    public void deallocate() {
                        p.deallocate();
                    }
                };
            }
        }
    }

    public Pointer(final Buffer b) {
        if (b != null) {
            allocate(b);
        }
        if (!isNull()) {
            this.address -= (long) (b.position() * sizeof());
            this.position = (long) b.position();
            this.limit = (long) b.limit();
            this.capacity = (long) b.capacity();
            this.deallocator = new Deallocator() {
                Buffer bb = b;

                public void deallocate() {
                    this.bb = null;
                }
            };
        }
    }

    /* access modifiers changed from: package-private */
    public void init(long allocatedAddress, long allocatedCapacity, long ownerAddress, long deallocatorAddress) {
        long j = allocatedCapacity;
        this.address = allocatedAddress;
        this.position = 0;
        this.limit = j;
        this.capacity = j;
        if (ownerAddress != 0 && deallocatorAddress != 0) {
            deallocator(new NativeDeallocator(this, ownerAddress, deallocatorAddress));
        }
    }

    protected static <P extends Pointer> P withDeallocator(P p) {
        return p.deallocator(new CustomDeallocator(p));
    }

    protected static class CustomDeallocator extends DeallocatorReference implements Deallocator {
        Method method = null;
        Pointer pointer = null;

        public CustomDeallocator(Pointer p) {
            super(p, (Deallocator) null);
            this.deallocator = this;
            Class<?> cls = p.getClass();
            Method[] declaredMethods = cls.getDeclaredMethods();
            int length = declaredMethods.length;
            int i = 0;
            while (true) {
                if (i >= length) {
                    break;
                }
                Method m = declaredMethods[i];
                Class[] parameters = m.getParameterTypes();
                if (Modifier.isStatic(m.getModifiers()) && m.getReturnType().equals(Void.TYPE) && m.getName().equals("deallocate") && parameters.length == 1 && Pointer.class.isAssignableFrom(parameters[0])) {
                    m.setAccessible(true);
                    this.method = m;
                    break;
                }
                i++;
            }
            if (this.method != null) {
                try {
                    Constructor<?> constructor = cls.getConstructor(new Class[]{Pointer.class});
                    constructor.setAccessible(true);
                    this.pointer = (Pointer) constructor.newInstance(new Object[]{p});
                } catch (Exception ex) {
                    throw new RuntimeException(ex);
                }
            } else {
                throw new RuntimeException(new NoSuchMethodException("static void " + cls.getCanonicalName() + ".deallocate(" + Pointer.class.getCanonicalName() + ")"));
            }
        }

        public void deallocate() {
            try {
                this.method.invoke((Object) null, new Object[]{this.pointer});
                this.pointer.setNull();
            } catch (Exception ex) {
                throw new RuntimeException(ex);
            }
        }

        public String toString() {
            return getClass().getName() + "[pointer=" + this.pointer + ",method=" + this.method + "]";
        }
    }

    protected static class NativeDeallocator extends DeallocatorReference implements Deallocator {
        private long deallocatorAddress;
        private long ownerAddress;

        private native void deallocate(long j, long j2);

        NativeDeallocator(Pointer p, long ownerAddress2, long deallocatorAddress2) {
            super(p, (Deallocator) null);
            this.deallocator = this;
            this.ownerAddress = ownerAddress2;
            this.deallocatorAddress = deallocatorAddress2;
        }

        public void deallocate() {
            if (this.ownerAddress != 0 && this.deallocatorAddress != 0) {
                deallocate(this.ownerAddress, this.deallocatorAddress);
                this.deallocatorAddress = 0;
                this.ownerAddress = 0;
            }
        }

        public String toString() {
            return getClass().getName() + "[ownerAddress=0x" + Long.toHexString(this.ownerAddress) + ",deallocatorAddress=0x" + Long.toHexString(this.deallocatorAddress) + "]";
        }
    }

    static class DeallocatorReference extends PhantomReference<Pointer> {
        static volatile DeallocatorReference head = null;
        static volatile long totalBytes = 0;
        long bytes;
        Deallocator deallocator;
        volatile DeallocatorReference next = null;
        volatile DeallocatorReference prev = null;

        DeallocatorReference(Pointer p, Deallocator deallocator2) {
            super(p, Pointer.referenceQueue);
            this.deallocator = deallocator2;
            this.bytes = p.capacity * ((long) p.sizeof());
        }

        /* access modifiers changed from: package-private */
        public final void add() {
            synchronized (DeallocatorReference.class) {
                if (head == null) {
                    head = this;
                } else {
                    this.next = head;
                    DeallocatorReference deallocatorReference = this.next;
                    head = this;
                    deallocatorReference.prev = this;
                }
                totalBytes += this.bytes;
            }
        }

        /* access modifiers changed from: package-private */
        public final void remove() {
            synchronized (DeallocatorReference.class) {
                if (this.prev != this || this.next != this) {
                    if (this.prev == null) {
                        head = this.next;
                    } else {
                        this.prev.next = this.next;
                    }
                    if (this.next != null) {
                        this.next.prev = this.prev;
                    }
                    this.next = this;
                    this.prev = this;
                    totalBytes -= this.bytes;
                }
            }
        }

        public void clear() {
            super.clear();
            if (this.deallocator != null) {
                if (Pointer.logger.isDebugEnabled()) {
                    Logger access$100 = Pointer.logger;
                    access$100.debug("Collecting " + this);
                }
                this.deallocator.deallocate();
                this.deallocator = null;
            }
        }

        public String toString() {
            return getClass().getName() + "[deallocator=" + this.deallocator + "]";
        }
    }

    static class DeallocatorThread extends Thread {
        DeallocatorThread() {
            super("JavaCPP Deallocator");
            setPriority(10);
            setDaemon(true);
            start();
        }

        public void run() {
            while (true) {
                try {
                    DeallocatorReference r = (DeallocatorReference) Pointer.referenceQueue.remove();
                    r.clear();
                    r.remove();
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    return;
                }
            }
        }
    }

    static {
        String s = System.getProperty("org.bytedeco.javacpp.noPointerGC", System.getProperty("org.bytedeco.javacpp.nopointergc", HttpState.PREEMPTIVE_DEFAULT).toLowerCase()).toLowerCase();
        if (s.equals("true") || s.equals("t") || s.equals("")) {
            referenceQueue = null;
            deallocatorThread = null;
        } else {
            referenceQueue = new ReferenceQueue<>();
            deallocatorThread = new DeallocatorThread();
        }
        long m = Runtime.getRuntime().maxMemory();
        String s2 = System.getProperty("org.bytedeco.javacpp.maxBytes", System.getProperty("org.bytedeco.javacpp.maxbytes"));
        if (s2 != null && s2.length() > 0) {
            try {
                m = parseBytes(s2);
            } catch (NumberFormatException e) {
                throw new RuntimeException(e);
            }
        }
        maxBytes = m;
        long m2 = Runtime.getRuntime().maxMemory() * 2;
        String s3 = System.getProperty("org.bytedeco.javacpp.maxPhysicalBytes", System.getProperty("org.bytedeco.javacpp.maxphysicalbytes"));
        if (s3 != null && s3.length() > 0) {
            try {
                m2 = parseBytes(s3);
            } catch (NumberFormatException e2) {
                throw new RuntimeException(e2);
            }
        }
        maxPhysicalBytes = m2;
        int n = 10;
        String s4 = System.getProperty("org.bytedeco.javacpp.maxRetries", System.getProperty("org.bytedeco.javacpp.maxretries"));
        if (s4 != null && s4.length() > 0) {
            try {
                n = Integer.parseInt(s4);
            } catch (NumberFormatException e3) {
                throw new RuntimeException(e3);
            }
        }
        maxRetries = n;
    }

    public static String formatBytes(long bytes) {
        if (bytes < 102400) {
            return bytes + "";
        }
        long j = bytes / 1024;
        long bytes2 = j;
        if (j < 102400) {
            return bytes2 + "K";
        }
        long j2 = bytes2 / 1024;
        long bytes3 = j2;
        if (j2 < 102400) {
            return bytes3 + "M";
        }
        long j3 = bytes3 / 1024;
        long bytes4 = j3;
        if (j3 < 102400) {
            return bytes4 + "G";
        }
        return (bytes4 / 1024) + "T";
    }

    /* JADX WARNING: Code restructure failed: missing block: B:37:0x0080, code lost:
        if (r4.equals("t") != false) goto L_0x00ad;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:56:0x00cb, code lost:
        r2 = r2 * 1024;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:57:0x00cd, code lost:
        r2 = r2 * 1024;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:61:?, code lost:
        return r2 * 1024;
     */
    /* JADX WARNING: Removed duplicated region for block: B:53:0x00b2  */
    /* JADX WARNING: Removed duplicated region for block: B:55:0x00c9  */
    /* JADX WARNING: Removed duplicated region for block: B:62:? A[RETURN, SYNTHETIC] */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public static long parseBytes(java.lang.String r8) throws java.lang.NumberFormatException {
        /*
            r0 = 0
            r1 = 0
        L_0x0002:
            int r2 = r8.length()
            if (r1 >= r2) goto L_0x0016
            char r2 = r8.charAt(r1)
            boolean r2 = java.lang.Character.isDigit(r2)
            if (r2 != 0) goto L_0x0013
            goto L_0x0016
        L_0x0013:
            int r1 = r1 + 1
            goto L_0x0002
        L_0x0016:
            java.lang.String r2 = r8.substring(r0, r1)
            long r2 = java.lang.Long.parseLong(r2)
            java.lang.String r4 = r8.substring(r1)
            java.lang.String r4 = r4.trim()
            java.lang.String r4 = r4.toLowerCase()
            r5 = -1
            int r6 = r4.hashCode()
            if (r6 == 0) goto L_0x00a1
            r7 = 103(0x67, float:1.44E-43)
            if (r6 == r7) goto L_0x0097
            r7 = 107(0x6b, float:1.5E-43)
            if (r6 == r7) goto L_0x008d
            r7 = 109(0x6d, float:1.53E-43)
            if (r6 == r7) goto L_0x0083
            r7 = 116(0x74, float:1.63E-43)
            if (r6 == r7) goto L_0x007a
            r0 = 3291(0xcdb, float:4.612E-42)
            if (r6 == r0) goto L_0x0070
            r0 = 3415(0xd57, float:4.785E-42)
            if (r6 == r0) goto L_0x0066
            r0 = 3477(0xd95, float:4.872E-42)
            if (r6 == r0) goto L_0x005c
            r0 = 3694(0xe6e, float:5.176E-42)
            if (r6 == r0) goto L_0x0052
            goto L_0x00ac
        L_0x0052:
            java.lang.String r0 = "tb"
            boolean r0 = r4.equals(r0)
            if (r0 == 0) goto L_0x00ac
            r0 = 1
            goto L_0x00ad
        L_0x005c:
            java.lang.String r0 = "mb"
            boolean r0 = r4.equals(r0)
            if (r0 == 0) goto L_0x00ac
            r0 = 5
            goto L_0x00ad
        L_0x0066:
            java.lang.String r0 = "kb"
            boolean r0 = r4.equals(r0)
            if (r0 == 0) goto L_0x00ac
            r0 = 7
            goto L_0x00ad
        L_0x0070:
            java.lang.String r0 = "gb"
            boolean r0 = r4.equals(r0)
            if (r0 == 0) goto L_0x00ac
            r0 = 3
            goto L_0x00ad
        L_0x007a:
            java.lang.String r6 = "t"
            boolean r4 = r4.equals(r6)
            if (r4 == 0) goto L_0x00ac
            goto L_0x00ad
        L_0x0083:
            java.lang.String r0 = "m"
            boolean r0 = r4.equals(r0)
            if (r0 == 0) goto L_0x00ac
            r0 = 4
            goto L_0x00ad
        L_0x008d:
            java.lang.String r0 = "k"
            boolean r0 = r4.equals(r0)
            if (r0 == 0) goto L_0x00ac
            r0 = 6
            goto L_0x00ad
        L_0x0097:
            java.lang.String r0 = "g"
            boolean r0 = r4.equals(r0)
            if (r0 == 0) goto L_0x00ac
            r0 = 2
            goto L_0x00ad
        L_0x00a1:
            java.lang.String r0 = ""
            boolean r0 = r4.equals(r0)
            if (r0 == 0) goto L_0x00ac
            r0 = 8
            goto L_0x00ad
        L_0x00ac:
            r0 = -1
        L_0x00ad:
            r4 = 1024(0x400, double:5.06E-321)
            switch(r0) {
                case 0: goto L_0x00c9;
                case 1: goto L_0x00c9;
                case 2: goto L_0x00cb;
                case 3: goto L_0x00cb;
                case 4: goto L_0x00cd;
                case 5: goto L_0x00cd;
                case 6: goto L_0x00cf;
                case 7: goto L_0x00cf;
                case 8: goto L_0x00d1;
                default: goto L_0x00b2;
            }
        L_0x00b2:
            java.lang.NumberFormatException r0 = new java.lang.NumberFormatException
            java.lang.StringBuilder r4 = new java.lang.StringBuilder
            r4.<init>()
            java.lang.String r5 = "Cannot parse into bytes: "
            r4.append(r5)
            r4.append(r8)
            java.lang.String r4 = r4.toString()
            r0.<init>(r4)
            throw r0
        L_0x00c9:
            long r2 = r2 * r4
        L_0x00cb:
            long r2 = r2 * r4
        L_0x00cd:
            long r2 = r2 * r4
        L_0x00cf:
            long r2 = r2 * r4
        L_0x00d1:
            return r2
        */
        throw new UnsupportedOperationException("Method not decompiled: org.bytedeco.javacpp.Pointer.parseBytes(java.lang.String):long");
    }

    public static void deallocateReferences() {
        while (referenceQueue != null) {
            DeallocatorReference deallocatorReference = (DeallocatorReference) referenceQueue.poll();
            DeallocatorReference r = deallocatorReference;
            if (deallocatorReference != null) {
                r.clear();
                r.remove();
            } else {
                return;
            }
        }
    }

    public static long maxBytes() {
        return maxBytes;
    }

    public static long totalBytes() {
        return DeallocatorReference.totalBytes;
    }

    public static long maxPhysicalBytes() {
        return maxPhysicalBytes;
    }

    public boolean isNull() {
        return this.address == 0;
    }

    public void setNull() {
        this.address = 0;
    }

    public long address() {
        return this.address;
    }

    public long position() {
        return this.position;
    }

    public <P extends Pointer> P position(long position2) {
        this.position = position2;
        return this;
    }

    public long limit() {
        return this.limit;
    }

    public <P extends Pointer> P limit(long limit2) {
        this.limit = limit2;
        return this;
    }

    public long capacity() {
        return this.capacity;
    }

    public <P extends Pointer> P capacity(long capacity2) {
        this.limit = capacity2;
        this.capacity = capacity2;
        return this;
    }

    /* access modifiers changed from: protected */
    public Deallocator deallocator() {
        return this.deallocator;
    }

    /* access modifiers changed from: protected */
    public <P extends Pointer> P deallocator(Deallocator deallocator2) {
        if (this.deallocator != null) {
            if (logger.isDebugEnabled()) {
                Logger logger2 = logger;
                logger2.debug("Predeallocating " + this);
            }
            this.deallocator.deallocate();
            this.deallocator = null;
        }
        if (deallocator2 != null && !deallocator2.equals((Object) null)) {
            this.deallocator = deallocator2;
            DeallocatorReference r = deallocator2 instanceof DeallocatorReference ? (DeallocatorReference) deallocator2 : new DeallocatorReference(this, deallocator2);
            int count = 0;
            long lastPhysicalBytes = maxPhysicalBytes > 0 ? physicalBytes() : 0;
            synchronized (DeallocatorThread.class) {
                while (true) {
                    int count2 = count + 1;
                    try {
                        if (count >= maxRetries || ((maxBytes <= 0 || DeallocatorReference.totalBytes + r.bytes <= maxBytes) && (maxPhysicalBytes <= 0 || lastPhysicalBytes <= maxPhysicalBytes))) {
                            break;
                        }
                        if (logger.isDebugEnabled()) {
                            Logger logger3 = logger;
                            logger3.debug("Calling System.gc() and Pointer.trimMemory() in " + this);
                        }
                        System.gc();
                        Thread.sleep(100);
                        trimMemory();
                        lastPhysicalBytes = maxPhysicalBytes > 0 ? physicalBytes() : 0;
                        count = count2;
                    } catch (InterruptedException e) {
                        Thread.currentThread().interrupt();
                    } catch (UnsatisfiedLinkError e2) {
                        if (logger.isDebugEnabled()) {
                            logger.debug(e2.getMessage());
                        }
                    }
                }
                if (maxBytes > 0) {
                    if (DeallocatorReference.totalBytes + r.bytes > maxBytes) {
                        deallocate();
                        throw new OutOfMemoryError("Failed to allocate memory within limits: totalBytes (" + formatBytes(DeallocatorReference.totalBytes) + " + " + formatBytes(r.bytes) + ") > maxBytes (" + formatBytes(maxBytes) + ")");
                    }
                }
                if (maxPhysicalBytes > 0) {
                    if (lastPhysicalBytes > maxPhysicalBytes) {
                        deallocate();
                        throw new OutOfMemoryError("Physical memory usage is too high: physicalBytes (" + formatBytes(lastPhysicalBytes) + ") > maxPhysicalBytes (" + formatBytes(maxPhysicalBytes) + ")");
                    }
                }
                if (logger.isDebugEnabled()) {
                    Logger logger4 = logger;
                    logger4.debug("Registering " + this);
                }
                r.add();
                PointerScope s = PointerScope.getInnerScope();
                if (s != null) {
                    s.attach(this);
                }
            }
        }
        return this;
    }

    public void close() {
        deallocate();
    }

    public void deallocate() {
        deallocate(true);
    }

    public void deallocate(boolean deallocate) {
        if (deallocate && this.deallocator != null) {
            if (logger.isDebugEnabled()) {
                Logger logger2 = logger;
                logger2.debug("Deallocating " + this);
            }
            this.deallocator.deallocate();
            this.address = 0;
        }
        if (!deallocate || referenceQueue == null) {
            synchronized (DeallocatorReference.class) {
                DeallocatorReference r = DeallocatorReference.head;
                while (true) {
                    if (r == null) {
                        break;
                    } else if (r.deallocator == this.deallocator) {
                        r.deallocator = null;
                        r.clear();
                        r.remove();
                        break;
                    } else {
                        r = r.next;
                    }
                }
            }
        }
    }

    public int offsetof(String member) {
        try {
            Class<?> cls = getClass();
            if (cls != Pointer.class) {
                return Loader.offsetof(cls, member);
            }
            return -1;
        } catch (ClassCastException | NullPointerException e) {
            return -1;
        }
    }

    public int sizeof() {
        Class c = getClass();
        if (c == Pointer.class || c == BytePointer.class) {
            return 1;
        }
        return offsetof("sizeof");
    }

    public ByteBuffer asByteBuffer() {
        if (isNull()) {
            return null;
        }
        if (this.limit <= 0 || this.limit >= this.position) {
            int size = sizeof();
            Pointer p = new Pointer();
            p.address = this.address;
            return p.position(((long) size) * this.position).capacity(((long) size) * (this.limit <= 0 ? this.position + 1 : this.limit)).asDirectBuffer().order(ByteOrder.nativeOrder());
        }
        throw new IllegalArgumentException("limit < position: (" + this.limit + " < " + this.position + ")");
    }

    public Buffer asBuffer() {
        return asByteBuffer();
    }

    public <P extends Pointer> P put(Pointer p) {
        if (p.limit <= 0 || p.limit >= p.position) {
            int size = sizeof();
            int psize = p.sizeof();
            long length = ((long) psize) * (p.limit <= 0 ? 1 : p.limit - p.position);
            this.position *= (long) size;
            p.position *= (long) psize;
            memcpy(this, p, length);
            this.position /= (long) size;
            p.position /= (long) psize;
            return this;
        }
        throw new IllegalArgumentException("limit < position: (" + p.limit + " < " + p.position + ")");
    }

    public <P extends Pointer> P fill(int b) {
        if (this.limit <= 0 || this.limit >= this.position) {
            int size = sizeof();
            long length = ((long) size) * (this.limit <= 0 ? 1 : this.limit - this.position);
            this.position *= (long) size;
            memset(this, b, length);
            this.position /= (long) size;
            return this;
        }
        throw new IllegalArgumentException("limit < position: (" + this.limit + " < " + this.position + ")");
    }

    public <P extends Pointer> P zero() {
        return fill(0);
    }

    public boolean equals(Object obj) {
        if (obj == this) {
            return true;
        }
        if (obj == null) {
            return isNull();
        }
        if (obj.getClass() != getClass() && obj.getClass() != Pointer.class && getClass() != Pointer.class) {
            return false;
        }
        Pointer other = (Pointer) obj;
        if (this.address == other.address && this.position == other.position) {
            return true;
        }
        return false;
    }

    public int hashCode() {
        return (int) this.address;
    }

    public String toString() {
        return getClass().getName() + "[address=0x" + Long.toHexString(this.address) + ",position=" + this.position + ",limit=" + this.limit + ",capacity=" + this.capacity + ",deallocator=" + this.deallocator + "]";
    }
}
