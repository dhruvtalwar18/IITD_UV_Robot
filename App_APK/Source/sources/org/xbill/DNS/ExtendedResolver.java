package org.xbill.DNS;

import java.io.IOException;
import java.net.UnknownHostException;
import java.util.ArrayList;
import java.util.List;

public class ExtendedResolver implements Resolver {
    private static final int quantum = 5;
    /* access modifiers changed from: private */
    public int lbStart = 0;
    /* access modifiers changed from: private */
    public boolean loadBalance = false;
    /* access modifiers changed from: private */
    public List resolvers;
    /* access modifiers changed from: private */
    public int retries = 3;

    static /* synthetic */ int access$208(ExtendedResolver x0) {
        int i = x0.lbStart;
        x0.lbStart = i + 1;
        return i;
    }

    static /* synthetic */ int access$244(ExtendedResolver x0, int x1) {
        int i = x0.lbStart % x1;
        x0.lbStart = i;
        return i;
    }

    private static class Resolution implements ResolverListener {
        boolean done;
        Object[] inprogress;
        ResolverListener listener;
        int outstanding;
        Message query;
        Resolver[] resolvers;
        Message response;
        int retries;
        int[] sent;
        Throwable thrown;

        public Resolution(ExtendedResolver eres, Message query2) {
            List l = eres.resolvers;
            this.resolvers = (Resolver[]) l.toArray(new Resolver[l.size()]);
            if (eres.loadBalance) {
                int nresolvers = this.resolvers.length;
                int start = ExtendedResolver.access$208(eres) % nresolvers;
                if (eres.lbStart > nresolvers) {
                    ExtendedResolver.access$244(eres, nresolvers);
                }
                if (start > 0) {
                    Resolver[] shuffle = new Resolver[nresolvers];
                    for (int i = 0; i < nresolvers; i++) {
                        shuffle[i] = this.resolvers[(i + start) % nresolvers];
                    }
                    this.resolvers = shuffle;
                }
            }
            this.sent = new int[this.resolvers.length];
            this.inprogress = new Object[this.resolvers.length];
            this.retries = eres.retries;
            this.query = query2;
        }

        /* JADX WARNING: No exception handlers in catch block: Catch:{  } */
        /* Code decompiled incorrectly, please refer to instructions dump. */
        public void send(int r5) {
            /*
                r4 = this;
                int[] r0 = r4.sent
                r1 = r0[r5]
                r2 = 1
                int r1 = r1 + r2
                r0[r5] = r1
                int r0 = r4.outstanding
                int r0 = r0 + r2
                r4.outstanding = r0
                java.lang.Object[] r0 = r4.inprogress     // Catch:{ Throwable -> 0x001c }
                org.xbill.DNS.Resolver[] r1 = r4.resolvers     // Catch:{ Throwable -> 0x001c }
                r1 = r1[r5]     // Catch:{ Throwable -> 0x001c }
                org.xbill.DNS.Message r3 = r4.query     // Catch:{ Throwable -> 0x001c }
                java.lang.Object r1 = r1.sendAsync(r3, r4)     // Catch:{ Throwable -> 0x001c }
                r0[r5] = r1     // Catch:{ Throwable -> 0x001c }
                goto L_0x002c
            L_0x001c:
                r0 = move-exception
                monitor-enter(r4)
                r4.thrown = r0     // Catch:{ all -> 0x002d }
                r4.done = r2     // Catch:{ all -> 0x002d }
                org.xbill.DNS.ResolverListener r1 = r4.listener     // Catch:{ all -> 0x002d }
                if (r1 != 0) goto L_0x002b
                r4.notifyAll()     // Catch:{ all -> 0x002d }
                monitor-exit(r4)     // Catch:{ all -> 0x002d }
                return
            L_0x002b:
                monitor-exit(r4)     // Catch:{ all -> 0x002d }
            L_0x002c:
                return
            L_0x002d:
                r1 = move-exception
                monitor-exit(r4)     // Catch:{ all -> 0x002d }
                throw r1
            */
            throw new UnsupportedOperationException("Method not decompiled: org.xbill.DNS.ExtendedResolver.Resolution.send(int):void");
        }

        public Message start() throws IOException {
            try {
                int[] iArr = this.sent;
                iArr[0] = iArr[0] + 1;
                this.outstanding++;
                this.inprogress[0] = new Object();
                return this.resolvers[0].send(this.query);
            } catch (Exception e) {
                handleException(this.inprogress[0], e);
                synchronized (this) {
                    while (!this.done) {
                        try {
                            wait();
                        } catch (InterruptedException e2) {
                        }
                    }
                    if (this.response != null) {
                        return this.response;
                    }
                    if (this.thrown instanceof IOException) {
                        throw ((IOException) this.thrown);
                    } else if (this.thrown instanceof RuntimeException) {
                        throw ((RuntimeException) this.thrown);
                    } else if (this.thrown instanceof Error) {
                        throw ((Error) this.thrown);
                    } else {
                        throw new IllegalStateException("ExtendedResolver failure");
                    }
                }
            }
        }

        public void startAsync(ResolverListener listener2) {
            this.listener = listener2;
            send(0);
        }

        public void receiveMessage(Object id, Message m) {
            if (Options.check("verbose")) {
                System.err.println("ExtendedResolver: received message");
            }
            synchronized (this) {
                if (!this.done) {
                    this.response = m;
                    this.done = true;
                    if (this.listener == null) {
                        notifyAll();
                    } else {
                        this.listener.receiveMessage(this, this.response);
                    }
                }
            }
        }

        /* JADX WARNING: Code restructure failed: missing block: B:64:0x00a6, code lost:
            if ((r5.thrown instanceof java.lang.Exception) != false) goto L_0x00b5;
         */
        /* JADX WARNING: Code restructure failed: missing block: B:65:0x00a8, code lost:
            r5.thrown = new java.lang.RuntimeException(r5.thrown.getMessage());
         */
        /* JADX WARNING: Code restructure failed: missing block: B:66:0x00b5, code lost:
            r5.listener.handleException(r5, (java.lang.Exception) r5.thrown);
         */
        /* JADX WARNING: Code restructure failed: missing block: B:67:0x00be, code lost:
            return;
         */
        /* Code decompiled incorrectly, please refer to instructions dump. */
        public void handleException(java.lang.Object r6, java.lang.Exception r7) {
            /*
                r5 = this;
                java.lang.String r0 = "verbose"
                boolean r0 = org.xbill.DNS.Options.check(r0)
                if (r0 == 0) goto L_0x001e
                java.io.PrintStream r0 = java.lang.System.err
                java.lang.StringBuffer r1 = new java.lang.StringBuffer
                r1.<init>()
                java.lang.String r2 = "ExtendedResolver: got "
                r1.append(r2)
                r1.append(r7)
                java.lang.String r1 = r1.toString()
                r0.println(r1)
            L_0x001e:
                monitor-enter(r5)
                int r0 = r5.outstanding     // Catch:{ all -> 0x00bf }
                r1 = 1
                int r0 = r0 - r1
                r5.outstanding = r0     // Catch:{ all -> 0x00bf }
                boolean r0 = r5.done     // Catch:{ all -> 0x00bf }
                if (r0 == 0) goto L_0x002b
                monitor-exit(r5)     // Catch:{ all -> 0x00bf }
                return
            L_0x002b:
                r0 = 0
            L_0x002c:
                java.lang.Object[] r2 = r5.inprogress     // Catch:{ all -> 0x00bf }
                int r2 = r2.length     // Catch:{ all -> 0x00bf }
                if (r0 >= r2) goto L_0x003b
                java.lang.Object[] r2 = r5.inprogress     // Catch:{ all -> 0x00bf }
                r2 = r2[r0]     // Catch:{ all -> 0x00bf }
                if (r2 != r6) goto L_0x0038
                goto L_0x003b
            L_0x0038:
                int r0 = r0 + 1
                goto L_0x002c
            L_0x003b:
                java.lang.Object[] r2 = r5.inprogress     // Catch:{ all -> 0x00bf }
                int r2 = r2.length     // Catch:{ all -> 0x00bf }
                if (r0 != r2) goto L_0x0042
                monitor-exit(r5)     // Catch:{ all -> 0x00bf }
                return
            L_0x0042:
                r2 = 0
                int[] r3 = r5.sent     // Catch:{ all -> 0x00bf }
                r3 = r3[r0]     // Catch:{ all -> 0x00bf }
                if (r3 != r1) goto L_0x0050
                org.xbill.DNS.Resolver[] r3 = r5.resolvers     // Catch:{ all -> 0x00bf }
                int r3 = r3.length     // Catch:{ all -> 0x00bf }
                int r3 = r3 - r1
                if (r0 >= r3) goto L_0x0050
                r2 = 1
            L_0x0050:
                boolean r3 = r7 instanceof java.io.InterruptedIOException     // Catch:{ all -> 0x00bf }
                if (r3 == 0) goto L_0x0066
                int[] r3 = r5.sent     // Catch:{ all -> 0x00bf }
                r3 = r3[r0]     // Catch:{ all -> 0x00bf }
                int r4 = r5.retries     // Catch:{ all -> 0x00bf }
                if (r3 >= r4) goto L_0x005f
                r5.send(r0)     // Catch:{ all -> 0x00bf }
            L_0x005f:
                java.lang.Throwable r3 = r5.thrown     // Catch:{ all -> 0x00bf }
                if (r3 != 0) goto L_0x0079
                r5.thrown = r7     // Catch:{ all -> 0x00bf }
                goto L_0x0079
            L_0x0066:
                boolean r3 = r7 instanceof java.net.SocketException     // Catch:{ all -> 0x00bf }
                if (r3 == 0) goto L_0x0077
                java.lang.Throwable r3 = r5.thrown     // Catch:{ all -> 0x00bf }
                if (r3 == 0) goto L_0x0074
                java.lang.Throwable r3 = r5.thrown     // Catch:{ all -> 0x00bf }
                boolean r3 = r3 instanceof java.io.InterruptedIOException     // Catch:{ all -> 0x00bf }
                if (r3 == 0) goto L_0x0079
            L_0x0074:
                r5.thrown = r7     // Catch:{ all -> 0x00bf }
                goto L_0x0079
            L_0x0077:
                r5.thrown = r7     // Catch:{ all -> 0x00bf }
            L_0x0079:
                boolean r3 = r5.done     // Catch:{ all -> 0x00bf }
                if (r3 == 0) goto L_0x007f
                monitor-exit(r5)     // Catch:{ all -> 0x00bf }
                return
            L_0x007f:
                if (r2 == 0) goto L_0x0086
                int r3 = r0 + 1
                r5.send(r3)     // Catch:{ all -> 0x00bf }
            L_0x0086:
                boolean r3 = r5.done     // Catch:{ all -> 0x00bf }
                if (r3 == 0) goto L_0x008c
                monitor-exit(r5)     // Catch:{ all -> 0x00bf }
                return
            L_0x008c:
                int r3 = r5.outstanding     // Catch:{ all -> 0x00bf }
                if (r3 != 0) goto L_0x009b
                r5.done = r1     // Catch:{ all -> 0x00bf }
                org.xbill.DNS.ResolverListener r1 = r5.listener     // Catch:{ all -> 0x00bf }
                if (r1 != 0) goto L_0x009b
                r5.notifyAll()     // Catch:{ all -> 0x00bf }
                monitor-exit(r5)     // Catch:{ all -> 0x00bf }
                return
            L_0x009b:
                boolean r1 = r5.done     // Catch:{ all -> 0x00bf }
                if (r1 != 0) goto L_0x00a1
                monitor-exit(r5)     // Catch:{ all -> 0x00bf }
                return
            L_0x00a1:
                monitor-exit(r5)     // Catch:{ all -> 0x00bf }
                java.lang.Throwable r0 = r5.thrown
                boolean r0 = r0 instanceof java.lang.Exception
                if (r0 != 0) goto L_0x00b5
                java.lang.RuntimeException r0 = new java.lang.RuntimeException
                java.lang.Throwable r1 = r5.thrown
                java.lang.String r1 = r1.getMessage()
                r0.<init>(r1)
                r5.thrown = r0
            L_0x00b5:
                org.xbill.DNS.ResolverListener r0 = r5.listener
                java.lang.Throwable r1 = r5.thrown
                java.lang.Exception r1 = (java.lang.Exception) r1
                r0.handleException(r5, r1)
                return
            L_0x00bf:
                r0 = move-exception
                monitor-exit(r5)     // Catch:{ all -> 0x00bf }
                throw r0
            */
            throw new UnsupportedOperationException("Method not decompiled: org.xbill.DNS.ExtendedResolver.Resolution.handleException(java.lang.Object, java.lang.Exception):void");
        }
    }

    private void init() {
        this.resolvers = new ArrayList();
    }

    public ExtendedResolver() throws UnknownHostException {
        init();
        String[] servers = ResolverConfig.getCurrentConfig().servers();
        if (servers != null) {
            for (String simpleResolver : servers) {
                Resolver r = new SimpleResolver(simpleResolver);
                r.setTimeout(5);
                this.resolvers.add(r);
            }
            return;
        }
        this.resolvers.add(new SimpleResolver());
    }

    public ExtendedResolver(String[] servers) throws UnknownHostException {
        init();
        for (String simpleResolver : servers) {
            Resolver r = new SimpleResolver(simpleResolver);
            r.setTimeout(5);
            this.resolvers.add(r);
        }
    }

    public ExtendedResolver(Resolver[] res) throws UnknownHostException {
        init();
        for (Resolver add : res) {
            this.resolvers.add(add);
        }
    }

    public void setPort(int port) {
        for (int i = 0; i < this.resolvers.size(); i++) {
            ((Resolver) this.resolvers.get(i)).setPort(port);
        }
    }

    public void setTCP(boolean flag) {
        for (int i = 0; i < this.resolvers.size(); i++) {
            ((Resolver) this.resolvers.get(i)).setTCP(flag);
        }
    }

    public void setIgnoreTruncation(boolean flag) {
        for (int i = 0; i < this.resolvers.size(); i++) {
            ((Resolver) this.resolvers.get(i)).setIgnoreTruncation(flag);
        }
    }

    public void setEDNS(int level) {
        for (int i = 0; i < this.resolvers.size(); i++) {
            ((Resolver) this.resolvers.get(i)).setEDNS(level);
        }
    }

    public void setEDNS(int level, int payloadSize, int flags, List options) {
        for (int i = 0; i < this.resolvers.size(); i++) {
            ((Resolver) this.resolvers.get(i)).setEDNS(level, payloadSize, flags, options);
        }
    }

    public void setTSIGKey(TSIG key) {
        for (int i = 0; i < this.resolvers.size(); i++) {
            ((Resolver) this.resolvers.get(i)).setTSIGKey(key);
        }
    }

    public void setTimeout(int secs, int msecs) {
        for (int i = 0; i < this.resolvers.size(); i++) {
            ((Resolver) this.resolvers.get(i)).setTimeout(secs, msecs);
        }
    }

    public void setTimeout(int secs) {
        setTimeout(secs, 0);
    }

    public Message send(Message query) throws IOException {
        return new Resolution(this, query).start();
    }

    public Object sendAsync(Message query, ResolverListener listener) {
        Resolution res = new Resolution(this, query);
        res.startAsync(listener);
        return res;
    }

    public Resolver getResolver(int n) {
        if (n < this.resolvers.size()) {
            return (Resolver) this.resolvers.get(n);
        }
        return null;
    }

    public Resolver[] getResolvers() {
        return (Resolver[]) this.resolvers.toArray(new Resolver[this.resolvers.size()]);
    }

    public void addResolver(Resolver r) {
        this.resolvers.add(r);
    }

    public void deleteResolver(Resolver r) {
        this.resolvers.remove(r);
    }

    public void setLoadBalance(boolean flag) {
        this.loadBalance = flag;
    }

    public void setRetries(int retries2) {
        this.retries = retries2;
    }
}
