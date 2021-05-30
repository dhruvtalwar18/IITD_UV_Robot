package org.jboss.netty.channel.socket.nio;

import java.io.IOException;
import java.io.PrintStream;
import java.nio.channels.SelectionKey;
import java.nio.channels.Selector;
import java.util.Map;
import java.util.Set;
import org.jboss.netty.logging.InternalLogger;
import org.jboss.netty.logging.InternalLoggerFactory;
import org.jboss.netty.util.internal.SystemPropertyUtil;

final class NioProviderMetadata {
    static final int CONSTRAINT_LEVEL;
    private static final String CONSTRAINT_LEVEL_PROPERTY = "org.jboss.netty.channel.socket.nio.constraintLevel";
    private static final String OLD_CONSTRAINT_LEVEL_PROPERTY = "java.nio.channels.spi.constraintLevel";
    static final InternalLogger logger = InternalLoggerFactory.getInstance((Class<?>) NioProviderMetadata.class);

    static {
        int constraintLevel = SystemPropertyUtil.get(CONSTRAINT_LEVEL_PROPERTY, -1);
        if (constraintLevel < 0 || constraintLevel > 2) {
            constraintLevel = SystemPropertyUtil.get(OLD_CONSTRAINT_LEVEL_PROPERTY, -1);
            if (constraintLevel < 0 || constraintLevel > 2) {
                constraintLevel = -1;
            } else {
                logger.warn("System property 'java.nio.channels.spi.constraintLevel' has been deprecated.  Use 'org.jboss.netty.channel.socket.nio.constraintLevel' instead.");
            }
        }
        if (constraintLevel >= 0) {
            InternalLogger internalLogger = logger;
            internalLogger.debug("Setting the NIO constraint level to: " + constraintLevel);
        }
        if (constraintLevel < 0) {
            constraintLevel = detectConstraintLevelFromSystemProperties();
            if (constraintLevel < 0) {
                constraintLevel = 2;
                if (logger.isDebugEnabled()) {
                    logger.debug("Couldn't determine the NIO constraint level from the system properties; using the safest level (2)");
                }
            } else if (constraintLevel != 0) {
                if (logger.isInfoEnabled()) {
                    InternalLogger internalLogger2 = logger;
                    internalLogger2.info("Using the autodetected NIO constraint level: " + constraintLevel + " (Use better NIO provider for better performance)");
                }
            } else if (logger.isDebugEnabled()) {
                InternalLogger internalLogger3 = logger;
                internalLogger3.debug("Using the autodetected NIO constraint level: " + constraintLevel);
            }
        }
        CONSTRAINT_LEVEL = constraintLevel;
        if (CONSTRAINT_LEVEL < 0 || CONSTRAINT_LEVEL > 2) {
            throw new Error("Unexpected NIO constraint level: " + CONSTRAINT_LEVEL + ", please report this error.");
        }
    }

    /* JADX WARNING: Removed duplicated region for block: B:89:0x016c A[RETURN] */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    private static int detectConstraintLevelFromSystemProperties() {
        /*
            java.lang.String r0 = "java.specification.version"
            java.lang.String r0 = org.jboss.netty.util.internal.SystemPropertyUtil.get(r0)
            java.lang.String r1 = "java.vm.info"
            java.lang.String r2 = ""
            java.lang.String r1 = org.jboss.netty.util.internal.SystemPropertyUtil.get((java.lang.String) r1, (java.lang.String) r2)
            java.lang.String r2 = "os.name"
            java.lang.String r2 = org.jboss.netty.util.internal.SystemPropertyUtil.get(r2)
            java.lang.String r3 = "java.vm.vendor"
            java.lang.String r3 = org.jboss.netty.util.internal.SystemPropertyUtil.get(r3)
            java.nio.channels.spi.SelectorProvider r4 = java.nio.channels.spi.SelectorProvider.provider()     // Catch:{ Exception -> 0x0027 }
            java.lang.Class r4 = r4.getClass()     // Catch:{ Exception -> 0x0027 }
            java.lang.String r4 = r4.getName()     // Catch:{ Exception -> 0x0027 }
            goto L_0x0029
        L_0x0027:
            r4 = move-exception
            r4 = 0
        L_0x0029:
            r5 = -1
            if (r0 == 0) goto L_0x016d
            if (r2 == 0) goto L_0x016d
            if (r3 == 0) goto L_0x016d
            if (r4 != 0) goto L_0x0034
            goto L_0x016d
        L_0x0034:
            java.lang.String r2 = r2.toLowerCase()
            java.lang.String r3 = r3.toLowerCase()
            java.lang.String r6 = "sun"
            int r6 = r3.indexOf(r6)
            r7 = 0
            if (r6 < 0) goto L_0x0088
            java.lang.String r6 = "linux"
            int r6 = r2.indexOf(r6)
            if (r6 < 0) goto L_0x005e
            java.lang.String r6 = "sun.nio.ch.EPollSelectorProvider"
            boolean r6 = r4.equals(r6)
            if (r6 != 0) goto L_0x005d
            java.lang.String r6 = "sun.nio.ch.PollSelectorProvider"
            boolean r6 = r4.equals(r6)
            if (r6 == 0) goto L_0x016c
        L_0x005d:
            return r7
        L_0x005e:
            java.lang.String r6 = "windows"
            int r6 = r2.indexOf(r6)
            if (r6 < 0) goto L_0x006f
            java.lang.String r6 = "sun.nio.ch.WindowsSelectorProvider"
            boolean r6 = r4.equals(r6)
            if (r6 == 0) goto L_0x016c
            return r7
        L_0x006f:
            java.lang.String r6 = "sun"
            int r6 = r2.indexOf(r6)
            if (r6 >= 0) goto L_0x007f
            java.lang.String r6 = "solaris"
            int r6 = r2.indexOf(r6)
            if (r6 < 0) goto L_0x016c
        L_0x007f:
            java.lang.String r6 = "sun.nio.ch.DevPollSelectorProvider"
            boolean r6 = r4.equals(r6)
            if (r6 == 0) goto L_0x016c
            return r7
        L_0x0088:
            java.lang.String r6 = "apple"
            int r6 = r3.indexOf(r6)
            if (r6 < 0) goto L_0x00a9
            java.lang.String r6 = "mac"
            int r6 = r2.indexOf(r6)
            if (r6 < 0) goto L_0x016c
            java.lang.String r6 = "os"
            int r6 = r2.indexOf(r6)
            if (r6 < 0) goto L_0x016c
            java.lang.String r6 = "sun.nio.ch.KQueueSelectorProvider"
            boolean r6 = r4.equals(r6)
            if (r6 == 0) goto L_0x016c
            return r7
        L_0x00a9:
            java.lang.String r6 = "ibm"
            int r6 = r3.indexOf(r6)
            r8 = 1
            if (r6 < 0) goto L_0x0120
            java.lang.String r6 = "linux"
            int r6 = r2.indexOf(r6)
            if (r6 >= 0) goto L_0x00c2
            java.lang.String r6 = "aix"
            int r6 = r2.indexOf(r6)
            if (r6 < 0) goto L_0x016c
        L_0x00c2:
            java.lang.String r6 = "1.5"
            boolean r6 = r0.equals(r6)
            if (r6 != 0) goto L_0x0117
            java.lang.String r6 = "^1\\.5\\D.*$"
            boolean r6 = r0.matches(r6)
            if (r6 == 0) goto L_0x00d3
            goto L_0x0117
        L_0x00d3:
            java.lang.String r6 = "1.6"
            boolean r6 = r0.equals(r6)
            if (r6 != 0) goto L_0x00e3
            java.lang.String r6 = "^1\\.6\\D.*$"
            boolean r6 = r0.matches(r6)
            if (r6 == 0) goto L_0x016c
        L_0x00e3:
            java.lang.String r6 = "(?:^|[^0-9])([2-9][0-9]{3}(?:0[1-9]|1[0-2])(?:0[1-9]|[12][0-9]|3[01]))(?:$|[^0-9])"
            java.util.regex.Pattern r6 = java.util.regex.Pattern.compile(r6)
            java.util.regex.Matcher r9 = r6.matcher(r1)
            boolean r10 = r9.find()
            if (r10 == 0) goto L_0x0116
            java.lang.String r10 = r9.group(r8)
            long r10 = java.lang.Long.parseLong(r10)
            r12 = 20081105(0x13269d1, double:9.921384E-317)
            int r14 = (r10 > r12 ? 1 : (r10 == r12 ? 0 : -1))
            if (r14 >= 0) goto L_0x0104
            r5 = 2
            return r5
        L_0x0104:
            java.lang.String r12 = "sun.nio.ch.EPollSelectorProvider"
            boolean r12 = r4.equals(r12)
            if (r12 == 0) goto L_0x010d
            return r7
        L_0x010d:
            java.lang.String r7 = "sun.nio.ch.PollSelectorProvider"
            boolean r7 = r4.equals(r7)
            if (r7 == 0) goto L_0x0116
            return r8
        L_0x0116:
            goto L_0x016c
        L_0x0117:
            java.lang.String r6 = "sun.nio.ch.PollSelectorProvider"
            boolean r6 = r4.equals(r6)
            if (r6 == 0) goto L_0x016c
            return r8
        L_0x0120:
            java.lang.String r6 = "bea"
            int r6 = r3.indexOf(r6)
            if (r6 >= 0) goto L_0x0142
            java.lang.String r6 = "oracle"
            int r6 = r3.indexOf(r6)
            if (r6 < 0) goto L_0x0131
            goto L_0x0142
        L_0x0131:
            java.lang.String r6 = "apache"
            int r6 = r3.indexOf(r6)
            if (r6 < 0) goto L_0x016c
            java.lang.String r6 = "org.apache.harmony.nio.internal.SelectorProviderImpl"
            boolean r6 = r4.equals(r6)
            if (r6 == 0) goto L_0x016c
            return r8
        L_0x0142:
            java.lang.String r6 = "linux"
            int r6 = r2.indexOf(r6)
            if (r6 < 0) goto L_0x015b
            java.lang.String r6 = "sun.nio.ch.EPollSelectorProvider"
            boolean r6 = r4.equals(r6)
            if (r6 != 0) goto L_0x015a
            java.lang.String r6 = "sun.nio.ch.PollSelectorProvider"
            boolean r6 = r4.equals(r6)
            if (r6 == 0) goto L_0x016c
        L_0x015a:
            return r7
        L_0x015b:
            java.lang.String r6 = "windows"
            int r6 = r2.indexOf(r6)
            if (r6 < 0) goto L_0x016c
            java.lang.String r6 = "sun.nio.ch.WindowsSelectorProvider"
            boolean r6 = r4.equals(r6)
            if (r6 == 0) goto L_0x016c
            return r7
        L_0x016c:
            return r5
        L_0x016d:
            return r5
        */
        throw new UnsupportedOperationException("Method not decompiled: org.jboss.netty.channel.socket.nio.NioProviderMetadata.detectConstraintLevelFromSystemProperties():int");
    }

    /* JADX WARNING: Code restructure failed: missing block: B:101:0x0131, code lost:
        if (r2 != null) goto L_0x0133;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:103:?, code lost:
        r2.close();
     */
    /* JADX WARNING: Code restructure failed: missing block: B:104:0x0137, code lost:
        r0 = move-exception;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:105:0x0138, code lost:
        r0 = r0;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:106:0x0140, code lost:
        if (logger.isWarnEnabled() != false) goto L_0x0142;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:107:0x0142, code lost:
        logger.warn("Failed to close a temporary socket.", r0);
     */
    /* JADX WARNING: Code restructure failed: missing block: B:108:0x0149, code lost:
        if (r3 == null) goto L_?;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:109:0x014b, code lost:
        r3.done = true;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:111:?, code lost:
        r1.shutdownNow();
     */
    /* JADX WARNING: Code restructure failed: missing block: B:122:0x016a, code lost:
        r0 = move-exception;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:124:0x0171, code lost:
        if (logger.isWarnEnabled() != false) goto L_0x0173;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:125:0x0173, code lost:
        logger.warn("Failed to close a temporary selector.", r0);
     */
    /* JADX WARNING: Code restructure failed: missing block: B:140:0x01a8, code lost:
        if (0 == 0) goto L_?;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:141:0x01aa, code lost:
        null.done = true;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:143:?, code lost:
        r1.shutdownNow();
     */
    /* JADX WARNING: Code restructure failed: missing block: B:158:0x01dc, code lost:
        r0 = move-exception;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:159:0x01dd, code lost:
        r4 = r0;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:162:0x01e4, code lost:
        if (logger.isWarnEnabled() != false) goto L_0x01e6;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:163:0x01e6, code lost:
        logger.warn("Failed to configure a temporary socket.", r4);
     */
    /* JADX WARNING: Code restructure failed: missing block: B:165:0x01ef, code lost:
        if (r2 != null) goto L_0x01f1;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:167:?, code lost:
        r2.close();
     */
    /* JADX WARNING: Code restructure failed: missing block: B:168:0x01f5, code lost:
        r0 = move-exception;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:169:0x01f6, code lost:
        r0 = r0;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:170:0x01fe, code lost:
        if (logger.isWarnEnabled() != false) goto L_0x0200;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:171:0x0200, code lost:
        logger.warn("Failed to close a temporary socket.", r0);
     */
    /* JADX WARNING: Code restructure failed: missing block: B:172:0x0207, code lost:
        if (0 == 0) goto L_?;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:173:0x0209, code lost:
        null.done = true;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:175:?, code lost:
        r1.shutdownNow();
     */
    /* JADX WARNING: Code restructure failed: missing block: B:186:0x0228, code lost:
        r0 = move-exception;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:188:0x022f, code lost:
        if (logger.isWarnEnabled() != false) goto L_0x0231;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:189:0x0231, code lost:
        logger.warn("Failed to close a temporary selector.", r0);
     */
    /* JADX WARNING: Code restructure failed: missing block: B:190:0x023b, code lost:
        r0 = th;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:191:0x023c, code lost:
        r10 = false;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:195:?, code lost:
        r9.close();
     */
    /* JADX WARNING: Code restructure failed: missing block: B:196:0x024b, code lost:
        r0 = move-exception;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:197:0x024c, code lost:
        r0 = r0;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:198:0x0254, code lost:
        if (logger.isWarnEnabled() != false) goto L_0x0256;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:199:0x0256, code lost:
        logger.warn("Failed to close a temporary socket.", r0);
     */
    /* JADX WARNING: Code restructure failed: missing block: B:201:0x025f, code lost:
        r3.done = true;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:203:?, code lost:
        r1.shutdownNow();
     */
    /* JADX WARNING: Code restructure failed: missing block: B:219:0x0291, code lost:
        r0 = th;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:220:0x0292, code lost:
        r10 = false;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:221:0x0293, code lost:
        r4 = r0;
        r9 = r10;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:222:0x0297, code lost:
        if (r2 != null) goto L_0x0299;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:224:?, code lost:
        r2.close();
     */
    /* JADX WARNING: Code restructure failed: missing block: B:225:0x029d, code lost:
        r0 = move-exception;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:226:0x029e, code lost:
        r0 = r0;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:227:0x02a6, code lost:
        if (logger.isWarnEnabled() != false) goto L_0x02a8;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:228:0x02a8, code lost:
        logger.warn("Failed to close a temporary socket.", r0);
     */
    /* JADX WARNING: Code restructure failed: missing block: B:229:0x02af, code lost:
        if (r3 != null) goto L_0x02b1;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:230:0x02b1, code lost:
        r3.done = true;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:232:?, code lost:
        r1.shutdownNow();
     */
    /* JADX WARNING: Code restructure failed: missing block: B:235:?, code lost:
        r3.selector.wakeup();
     */
    /* JADX WARNING: Code restructure failed: missing block: B:238:0x02c3, code lost:
        if (r1.awaitTermination(1, java.util.concurrent.TimeUnit.SECONDS) == false) goto L_0x02b8;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:242:?, code lost:
        r3.selector.close();
     */
    /* JADX WARNING: Code restructure failed: missing block: B:243:0x02d0, code lost:
        r0 = move-exception;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:245:0x02d7, code lost:
        if (logger.isWarnEnabled() != false) goto L_0x02d9;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:246:0x02d9, code lost:
        logger.warn("Failed to close a temporary selector.", r0);
     */
    /* JADX WARNING: Code restructure failed: missing block: B:285:?, code lost:
        return -1;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:286:?, code lost:
        return -1;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:287:?, code lost:
        return -1;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:290:?, code lost:
        return -1;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:293:?, code lost:
        return -1;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:294:?, code lost:
        return -1;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:295:?, code lost:
        return -1;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:297:?, code lost:
        return -1;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:298:?, code lost:
        return -1;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:299:?, code lost:
        return -1;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:300:?, code lost:
        return -1;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:94:0x011e, code lost:
        r0 = move-exception;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:95:0x011f, code lost:
        r4 = r0;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:98:0x0126, code lost:
        if (logger.isWarnEnabled() != false) goto L_0x0128;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:99:0x0128, code lost:
        logger.warn("Failed to register a temporary selector.", r4);
     */
    /* JADX WARNING: Failed to process nested try/catch */
    /* JADX WARNING: Removed duplicated region for block: B:190:0x023b A[ExcHandler: all (th java.lang.Throwable), PHI: r2 r3 
      PHI: (r2v5 'ch' java.nio.channels.ServerSocketChannel) = (r2v0 'ch' java.nio.channels.ServerSocketChannel), (r2v6 'ch' java.nio.channels.ServerSocketChannel), (r2v6 'ch' java.nio.channels.ServerSocketChannel), (r2v6 'ch' java.nio.channels.ServerSocketChannel), (r2v6 'ch' java.nio.channels.ServerSocketChannel), (r2v6 'ch' java.nio.channels.ServerSocketChannel), (r2v6 'ch' java.nio.channels.ServerSocketChannel), (r2v6 'ch' java.nio.channels.ServerSocketChannel), (r2v6 'ch' java.nio.channels.ServerSocketChannel), (r2v6 'ch' java.nio.channels.ServerSocketChannel), (r2v6 'ch' java.nio.channels.ServerSocketChannel), (r2v6 'ch' java.nio.channels.ServerSocketChannel) binds: [B:1:0x000b, B:4:0x0010, B:160:0x01de, B:5:?, B:6:0x0020, B:128:0x017f, B:7:?, B:9:0x0027, B:96:0x0120, B:10:?, B:11:0x002d, B:12:?] A[DONT_GENERATE, DONT_INLINE]
      PHI: (r3v4 'loop' org.jboss.netty.channel.socket.nio.NioProviderMetadata$SelectorLoop) = (r3v0 'loop' org.jboss.netty.channel.socket.nio.NioProviderMetadata$SelectorLoop), (r3v0 'loop' org.jboss.netty.channel.socket.nio.NioProviderMetadata$SelectorLoop), (r3v0 'loop' org.jboss.netty.channel.socket.nio.NioProviderMetadata$SelectorLoop), (r3v0 'loop' org.jboss.netty.channel.socket.nio.NioProviderMetadata$SelectorLoop), (r3v0 'loop' org.jboss.netty.channel.socket.nio.NioProviderMetadata$SelectorLoop), (r3v0 'loop' org.jboss.netty.channel.socket.nio.NioProviderMetadata$SelectorLoop), (r3v0 'loop' org.jboss.netty.channel.socket.nio.NioProviderMetadata$SelectorLoop), (r3v5 'loop' org.jboss.netty.channel.socket.nio.NioProviderMetadata$SelectorLoop), (r3v5 'loop' org.jboss.netty.channel.socket.nio.NioProviderMetadata$SelectorLoop), (r3v5 'loop' org.jboss.netty.channel.socket.nio.NioProviderMetadata$SelectorLoop), (r3v5 'loop' org.jboss.netty.channel.socket.nio.NioProviderMetadata$SelectorLoop), (r3v5 'loop' org.jboss.netty.channel.socket.nio.NioProviderMetadata$SelectorLoop) binds: [B:1:0x000b, B:4:0x0010, B:160:0x01de, B:5:?, B:6:0x0020, B:128:0x017f, B:7:?, B:9:0x0027, B:96:0x0120, B:10:?, B:11:0x002d, B:12:?] A[DONT_GENERATE, DONT_INLINE], Splitter:B:1:0x000b] */
    /* JADX WARNING: Removed duplicated region for block: B:194:0x0247 A[SYNTHETIC, Splitter:B:194:0x0247] */
    /* JADX WARNING: Removed duplicated region for block: B:201:0x025f  */
    /* JADX WARNING: Removed duplicated region for block: B:278:0x0266 A[SYNTHETIC] */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    private static int autodetect() {
        /*
            java.util.concurrent.ExecutorService r1 = java.util.concurrent.Executors.newCachedThreadPool()
            r2 = 0
            r0 = 0
            r3 = r0
            r5 = 1
            r7 = 0
            r8 = 1
            java.nio.channels.ServerSocketChannel r0 = java.nio.channels.ServerSocketChannel.open()     // Catch:{ Throwable -> 0x0291, all -> 0x023b }
            r2 = r0
            java.net.ServerSocket r0 = r2.socket()     // Catch:{ Throwable -> 0x01dc, all -> 0x023b }
            java.net.InetSocketAddress r9 = new java.net.InetSocketAddress     // Catch:{ Throwable -> 0x01dc, all -> 0x023b }
            r9.<init>(r7)     // Catch:{ Throwable -> 0x01dc, all -> 0x023b }
            r0.bind(r9)     // Catch:{ Throwable -> 0x01dc, all -> 0x023b }
            r2.configureBlocking(r7)     // Catch:{ Throwable -> 0x01dc, all -> 0x023b }
            org.jboss.netty.channel.socket.nio.NioProviderMetadata$SelectorLoop r0 = new org.jboss.netty.channel.socket.nio.NioProviderMetadata$SelectorLoop     // Catch:{ Throwable -> 0x017d, all -> 0x023b }
            r0.<init>()     // Catch:{ Throwable -> 0x017d, all -> 0x023b }
            r3 = r0
            java.nio.channels.Selector r0 = r3.selector     // Catch:{ Throwable -> 0x011e, all -> 0x023b }
            r2.register(r0, r7)     // Catch:{ Throwable -> 0x011e, all -> 0x023b }
            java.nio.channels.Selector r0 = r3.selector     // Catch:{ Throwable -> 0x0291, all -> 0x023b }
            java.nio.channels.SelectionKey r0 = r2.keyFor(r0)     // Catch:{ Throwable -> 0x0291, all -> 0x023b }
            r9 = r0
            r1.execute(r3)     // Catch:{ Throwable -> 0x0291, all -> 0x023b }
            r10 = 1
            r0 = 0
        L_0x0039:
            r11 = r0
            r12 = 500000000(0x1dcd6500, double:2.47032823E-315)
            r14 = 50
            r4 = 10
            if (r11 >= r4) goto L_0x0080
        L_0x0043:
            boolean r0 = r3.selecting     // Catch:{ Throwable -> 0x007d, all -> 0x007a }
            if (r0 != 0) goto L_0x004b
            java.lang.Thread.yield()     // Catch:{ Throwable -> 0x007d, all -> 0x007a }
            goto L_0x0043
        L_0x004b:
            java.lang.Thread.sleep(r14)     // Catch:{ InterruptedException -> 0x004f }
            goto L_0x0050
        L_0x004f:
            r0 = move-exception
        L_0x0050:
            boolean r0 = r3.selecting     // Catch:{ Throwable -> 0x007d, all -> 0x007a }
            if (r0 == 0) goto L_0x0043
            long r16 = java.lang.System.nanoTime()     // Catch:{ Throwable -> 0x007d, all -> 0x007a }
            int r0 = r9.interestOps()     // Catch:{ Throwable -> 0x007d, all -> 0x007a }
            r0 = r0 | 16
            r9.interestOps(r0)     // Catch:{ Throwable -> 0x007d, all -> 0x007a }
            int r0 = r9.interestOps()     // Catch:{ Throwable -> 0x007d, all -> 0x007a }
            r0 = r0 & -17
            r9.interestOps(r0)     // Catch:{ Throwable -> 0x007d, all -> 0x007a }
            long r18 = java.lang.System.nanoTime()     // Catch:{ Throwable -> 0x007d, all -> 0x007a }
            r0 = 0
            long r18 = r18 - r16
            int r0 = (r18 > r12 ? 1 : (r18 == r12 ? 0 : -1))
            if (r0 < 0) goto L_0x0077
            r10 = 0
            goto L_0x0080
        L_0x0077:
            int r0 = r11 + 1
            goto L_0x0039
        L_0x007a:
            r0 = move-exception
            goto L_0x023d
        L_0x007d:
            r0 = move-exception
            goto L_0x0293
        L_0x0080:
            if (r10 == 0) goto L_0x0084
            r0 = 0
            goto L_0x00cf
        L_0x0084:
            r10 = 1
            r0 = 0
        L_0x0086:
            r11 = r0
            if (r11 >= r4) goto L_0x00ca
        L_0x0089:
            boolean r0 = r3.selecting     // Catch:{ Throwable -> 0x007d, all -> 0x007a }
            if (r0 != 0) goto L_0x0091
            java.lang.Thread.yield()     // Catch:{ Throwable -> 0x007d, all -> 0x007a }
            goto L_0x0089
        L_0x0091:
            java.lang.Thread.sleep(r14)     // Catch:{ InterruptedException -> 0x0095 }
            goto L_0x0096
        L_0x0095:
            r0 = move-exception
        L_0x0096:
            boolean r0 = r3.selecting     // Catch:{ Throwable -> 0x007d, all -> 0x007a }
            if (r0 == 0) goto L_0x0089
            long r16 = java.lang.System.nanoTime()     // Catch:{ Throwable -> 0x007d, all -> 0x007a }
            int r0 = r9.interestOps()     // Catch:{ Throwable -> 0x007d, all -> 0x007a }
            r18 = r0
            monitor-enter(r3)     // Catch:{ Throwable -> 0x007d, all -> 0x007a }
            java.nio.channels.Selector r0 = r3.selector     // Catch:{ all -> 0x00c7 }
            r0.wakeup()     // Catch:{ all -> 0x00c7 }
            r0 = r18 | 16
            r9.interestOps(r0)     // Catch:{ all -> 0x00c7 }
            r0 = r18 & -17
            r9.interestOps(r0)     // Catch:{ all -> 0x00c7 }
            monitor-exit(r3)     // Catch:{ all -> 0x00c7 }
            long r19 = java.lang.System.nanoTime()     // Catch:{ Throwable -> 0x007d, all -> 0x007a }
            r0 = 0
            long r19 = r19 - r16
            int r0 = (r19 > r12 ? 1 : (r19 == r12 ? 0 : -1))
            if (r0 < 0) goto L_0x00c4
            r0 = 0
            r10 = r0
            goto L_0x00ca
        L_0x00c4:
            int r0 = r11 + 1
            goto L_0x0086
        L_0x00c7:
            r0 = move-exception
            monitor-exit(r3)     // Catch:{ all -> 0x00c7 }
            throw r0     // Catch:{ Throwable -> 0x007d, all -> 0x007a }
        L_0x00ca:
            if (r10 == 0) goto L_0x00ce
            r0 = 1
            goto L_0x00cf
        L_0x00ce:
            r0 = 2
        L_0x00cf:
            r4 = r0
            r7 = r10
            if (r2 == 0) goto L_0x00ea
            r2.close()     // Catch:{ Throwable -> 0x00d8 }
            goto L_0x00ea
        L_0x00d8:
            r0 = move-exception
            r9 = r0
            r0 = r9
            org.jboss.netty.logging.InternalLogger r9 = logger
            boolean r9 = r9.isWarnEnabled()
            if (r9 == 0) goto L_0x00ea
            org.jboss.netty.logging.InternalLogger r9 = logger
            java.lang.String r10 = "Failed to close a temporary socket."
            r9.warn(r10, r0)
        L_0x00ea:
            if (r3 == 0) goto L_0x011b
            r3.done = r8
            r1.shutdownNow()     // Catch:{ NullPointerException -> 0x00f2 }
            goto L_0x00f3
        L_0x00f2:
            r0 = move-exception
        L_0x00f3:
            java.nio.channels.Selector r0 = r3.selector     // Catch:{ Throwable -> 0x0104 }
            r0.wakeup()     // Catch:{ Throwable -> 0x0104 }
            java.util.concurrent.TimeUnit r0 = java.util.concurrent.TimeUnit.SECONDS     // Catch:{ InterruptedException -> 0x0102 }
            boolean r0 = r1.awaitTermination(r5, r0)     // Catch:{ InterruptedException -> 0x0102 }
            if (r0 == 0) goto L_0x0103
            goto L_0x0105
        L_0x0102:
            r0 = move-exception
        L_0x0103:
            goto L_0x00f3
        L_0x0104:
            r0 = move-exception
        L_0x0105:
            java.nio.channels.Selector r0 = r3.selector     // Catch:{ Throwable -> 0x010b }
            r0.close()     // Catch:{ Throwable -> 0x010b }
            goto L_0x011b
        L_0x010b:
            r0 = move-exception
            org.jboss.netty.logging.InternalLogger r5 = logger
            boolean r5 = r5.isWarnEnabled()
            if (r5 == 0) goto L_0x011b
            org.jboss.netty.logging.InternalLogger r5 = logger
            java.lang.String r6 = "Failed to close a temporary selector."
            r5.warn(r6, r0)
        L_0x011b:
            return r4
        L_0x011e:
            r0 = move-exception
            r4 = r0
            org.jboss.netty.logging.InternalLogger r0 = logger     // Catch:{ Throwable -> 0x0291, all -> 0x023b }
            boolean r0 = r0.isWarnEnabled()     // Catch:{ Throwable -> 0x0291, all -> 0x023b }
            if (r0 == 0) goto L_0x012f
            org.jboss.netty.logging.InternalLogger r0 = logger     // Catch:{ Throwable -> 0x0291, all -> 0x023b }
            java.lang.String r9 = "Failed to register a temporary selector."
            r0.warn(r9, r4)     // Catch:{ Throwable -> 0x0291, all -> 0x023b }
        L_0x012f:
            r9 = r7
            if (r2 == 0) goto L_0x0149
            r2.close()     // Catch:{ Throwable -> 0x0137 }
            goto L_0x0149
        L_0x0137:
            r0 = move-exception
            r10 = r0
            r0 = r10
            org.jboss.netty.logging.InternalLogger r10 = logger
            boolean r10 = r10.isWarnEnabled()
            if (r10 == 0) goto L_0x0149
            org.jboss.netty.logging.InternalLogger r10 = logger
            java.lang.String r11 = "Failed to close a temporary socket."
            r10.warn(r11, r0)
        L_0x0149:
            if (r3 == 0) goto L_0x017a
            r3.done = r8
            r1.shutdownNow()     // Catch:{ NullPointerException -> 0x0151 }
            goto L_0x0152
        L_0x0151:
            r0 = move-exception
        L_0x0152:
            java.nio.channels.Selector r0 = r3.selector     // Catch:{ Throwable -> 0x0163 }
            r0.wakeup()     // Catch:{ Throwable -> 0x0163 }
            java.util.concurrent.TimeUnit r0 = java.util.concurrent.TimeUnit.SECONDS     // Catch:{ InterruptedException -> 0x0161 }
            boolean r0 = r1.awaitTermination(r5, r0)     // Catch:{ InterruptedException -> 0x0161 }
            if (r0 == 0) goto L_0x0162
            goto L_0x0164
        L_0x0161:
            r0 = move-exception
        L_0x0162:
            goto L_0x0152
        L_0x0163:
            r0 = move-exception
        L_0x0164:
            java.nio.channels.Selector r0 = r3.selector     // Catch:{ Throwable -> 0x016a }
            r0.close()     // Catch:{ Throwable -> 0x016a }
            goto L_0x017a
        L_0x016a:
            r0 = move-exception
            org.jboss.netty.logging.InternalLogger r5 = logger
            boolean r5 = r5.isWarnEnabled()
            if (r5 == 0) goto L_0x017a
            org.jboss.netty.logging.InternalLogger r5 = logger
            java.lang.String r6 = "Failed to close a temporary selector."
            r5.warn(r6, r0)
        L_0x017a:
            r5 = -1
            return r5
        L_0x017d:
            r0 = move-exception
            r4 = r0
            org.jboss.netty.logging.InternalLogger r0 = logger     // Catch:{ Throwable -> 0x0291, all -> 0x023b }
            boolean r0 = r0.isWarnEnabled()     // Catch:{ Throwable -> 0x0291, all -> 0x023b }
            if (r0 == 0) goto L_0x018e
            org.jboss.netty.logging.InternalLogger r0 = logger     // Catch:{ Throwable -> 0x0291, all -> 0x023b }
            java.lang.String r9 = "Failed to open a temporary selector."
            r0.warn(r9, r4)     // Catch:{ Throwable -> 0x0291, all -> 0x023b }
        L_0x018e:
            r9 = r7
            if (r2 == 0) goto L_0x01a8
            r2.close()     // Catch:{ Throwable -> 0x0196 }
            goto L_0x01a8
        L_0x0196:
            r0 = move-exception
            r10 = r0
            r0 = r10
            org.jboss.netty.logging.InternalLogger r10 = logger
            boolean r10 = r10.isWarnEnabled()
            if (r10 == 0) goto L_0x01a8
            org.jboss.netty.logging.InternalLogger r10 = logger
            java.lang.String r11 = "Failed to close a temporary socket."
            r10.warn(r11, r0)
        L_0x01a8:
            if (r3 == 0) goto L_0x01d9
            r3.done = r8
            r1.shutdownNow()     // Catch:{ NullPointerException -> 0x01b0 }
            goto L_0x01b1
        L_0x01b0:
            r0 = move-exception
        L_0x01b1:
            java.nio.channels.Selector r0 = r3.selector     // Catch:{ Throwable -> 0x01c2 }
            r0.wakeup()     // Catch:{ Throwable -> 0x01c2 }
            java.util.concurrent.TimeUnit r0 = java.util.concurrent.TimeUnit.SECONDS     // Catch:{ InterruptedException -> 0x01c0 }
            boolean r0 = r1.awaitTermination(r5, r0)     // Catch:{ InterruptedException -> 0x01c0 }
            if (r0 == 0) goto L_0x01c1
            goto L_0x01c3
        L_0x01c0:
            r0 = move-exception
        L_0x01c1:
            goto L_0x01b1
        L_0x01c2:
            r0 = move-exception
        L_0x01c3:
            java.nio.channels.Selector r0 = r3.selector     // Catch:{ Throwable -> 0x01c9 }
            r0.close()     // Catch:{ Throwable -> 0x01c9 }
            goto L_0x01d9
        L_0x01c9:
            r0 = move-exception
            org.jboss.netty.logging.InternalLogger r5 = logger
            boolean r5 = r5.isWarnEnabled()
            if (r5 == 0) goto L_0x01d9
            org.jboss.netty.logging.InternalLogger r5 = logger
            java.lang.String r6 = "Failed to close a temporary selector."
            r5.warn(r6, r0)
        L_0x01d9:
            r5 = -1
            return r5
        L_0x01dc:
            r0 = move-exception
            r4 = r0
            org.jboss.netty.logging.InternalLogger r0 = logger     // Catch:{ Throwable -> 0x0291, all -> 0x023b }
            boolean r0 = r0.isWarnEnabled()     // Catch:{ Throwable -> 0x0291, all -> 0x023b }
            if (r0 == 0) goto L_0x01ed
            org.jboss.netty.logging.InternalLogger r0 = logger     // Catch:{ Throwable -> 0x0291, all -> 0x023b }
            java.lang.String r9 = "Failed to configure a temporary socket."
            r0.warn(r9, r4)     // Catch:{ Throwable -> 0x0291, all -> 0x023b }
        L_0x01ed:
            r9 = r7
            if (r2 == 0) goto L_0x0207
            r2.close()     // Catch:{ Throwable -> 0x01f5 }
            goto L_0x0207
        L_0x01f5:
            r0 = move-exception
            r10 = r0
            r0 = r10
            org.jboss.netty.logging.InternalLogger r10 = logger
            boolean r10 = r10.isWarnEnabled()
            if (r10 == 0) goto L_0x0207
            org.jboss.netty.logging.InternalLogger r10 = logger
            java.lang.String r11 = "Failed to close a temporary socket."
            r10.warn(r11, r0)
        L_0x0207:
            if (r3 == 0) goto L_0x0238
            r3.done = r8
            r1.shutdownNow()     // Catch:{ NullPointerException -> 0x020f }
            goto L_0x0210
        L_0x020f:
            r0 = move-exception
        L_0x0210:
            java.nio.channels.Selector r0 = r3.selector     // Catch:{ Throwable -> 0x0221 }
            r0.wakeup()     // Catch:{ Throwable -> 0x0221 }
            java.util.concurrent.TimeUnit r0 = java.util.concurrent.TimeUnit.SECONDS     // Catch:{ InterruptedException -> 0x021f }
            boolean r0 = r1.awaitTermination(r5, r0)     // Catch:{ InterruptedException -> 0x021f }
            if (r0 == 0) goto L_0x0220
            goto L_0x0222
        L_0x021f:
            r0 = move-exception
        L_0x0220:
            goto L_0x0210
        L_0x0221:
            r0 = move-exception
        L_0x0222:
            java.nio.channels.Selector r0 = r3.selector     // Catch:{ Throwable -> 0x0228 }
            r0.close()     // Catch:{ Throwable -> 0x0228 }
            goto L_0x0238
        L_0x0228:
            r0 = move-exception
            org.jboss.netty.logging.InternalLogger r5 = logger
            boolean r5 = r5.isWarnEnabled()
            if (r5 == 0) goto L_0x0238
            org.jboss.netty.logging.InternalLogger r5 = logger
            java.lang.String r6 = "Failed to close a temporary selector."
            r5.warn(r6, r0)
        L_0x0238:
            r5 = -1
            return r5
        L_0x023b:
            r0 = move-exception
            r10 = 0
        L_0x023d:
            r21 = r2
            r2 = r0
            r0 = r21
            r4 = r7
            r7 = r10
            r9 = r0
            if (r9 == 0) goto L_0x025d
            r9.close()     // Catch:{ Throwable -> 0x024b }
            goto L_0x025d
        L_0x024b:
            r0 = move-exception
            r10 = r0
            r0 = r10
            org.jboss.netty.logging.InternalLogger r10 = logger
            boolean r10 = r10.isWarnEnabled()
            if (r10 == 0) goto L_0x025d
            org.jboss.netty.logging.InternalLogger r10 = logger
            java.lang.String r11 = "Failed to close a temporary socket."
            r10.warn(r11, r0)
        L_0x025d:
            if (r3 == 0) goto L_0x028f
            r3.done = r8
            r1.shutdownNow()     // Catch:{ NullPointerException -> 0x0265 }
            goto L_0x0266
        L_0x0265:
            r0 = move-exception
        L_0x0266:
            java.nio.channels.Selector r0 = r3.selector     // Catch:{ Throwable -> 0x0278 }
            r0.wakeup()     // Catch:{ Throwable -> 0x0278 }
            java.util.concurrent.TimeUnit r0 = java.util.concurrent.TimeUnit.SECONDS     // Catch:{ InterruptedException -> 0x0276 }
            boolean r0 = r1.awaitTermination(r5, r0)     // Catch:{ InterruptedException -> 0x0276 }
            if (r0 != 0) goto L_0x0274
            goto L_0x0277
        L_0x0274:
            goto L_0x0279
        L_0x0276:
            r0 = move-exception
        L_0x0277:
            goto L_0x0266
        L_0x0278:
            r0 = move-exception
        L_0x0279:
            java.nio.channels.Selector r0 = r3.selector     // Catch:{ Throwable -> 0x027f }
            r0.close()     // Catch:{ Throwable -> 0x027f }
            goto L_0x028f
        L_0x027f:
            r0 = move-exception
            org.jboss.netty.logging.InternalLogger r5 = logger
            boolean r5 = r5.isWarnEnabled()
            if (r5 == 0) goto L_0x028f
            org.jboss.netty.logging.InternalLogger r5 = logger
            java.lang.String r6 = "Failed to close a temporary selector."
            r5.warn(r6, r0)
        L_0x028f:
            throw r2
        L_0x0291:
            r0 = move-exception
            r10 = 0
        L_0x0293:
            r4 = r0
            r9 = r10
            if (r2 == 0) goto L_0x02af
            r2.close()     // Catch:{ Throwable -> 0x029d }
            goto L_0x02af
        L_0x029d:
            r0 = move-exception
            r10 = r0
            r0 = r10
            org.jboss.netty.logging.InternalLogger r10 = logger
            boolean r10 = r10.isWarnEnabled()
            if (r10 == 0) goto L_0x02af
            org.jboss.netty.logging.InternalLogger r10 = logger
            java.lang.String r11 = "Failed to close a temporary socket."
            r10.warn(r11, r0)
        L_0x02af:
            if (r3 == 0) goto L_0x02e0
            r3.done = r8
            r1.shutdownNow()     // Catch:{ NullPointerException -> 0x02b7 }
            goto L_0x02b8
        L_0x02b7:
            r0 = move-exception
        L_0x02b8:
            java.nio.channels.Selector r0 = r3.selector     // Catch:{ Throwable -> 0x02c9 }
            r0.wakeup()     // Catch:{ Throwable -> 0x02c9 }
            java.util.concurrent.TimeUnit r0 = java.util.concurrent.TimeUnit.SECONDS     // Catch:{ InterruptedException -> 0x02c7 }
            boolean r0 = r1.awaitTermination(r5, r0)     // Catch:{ InterruptedException -> 0x02c7 }
            if (r0 == 0) goto L_0x02c8
            goto L_0x02ca
        L_0x02c7:
            r0 = move-exception
        L_0x02c8:
            goto L_0x02b8
        L_0x02c9:
            r0 = move-exception
        L_0x02ca:
            java.nio.channels.Selector r0 = r3.selector     // Catch:{ Throwable -> 0x02d0 }
            r0.close()     // Catch:{ Throwable -> 0x02d0 }
            goto L_0x02e0
        L_0x02d0:
            r0 = move-exception
            org.jboss.netty.logging.InternalLogger r5 = logger
            boolean r5 = r5.isWarnEnabled()
            if (r5 == 0) goto L_0x02e0
            org.jboss.netty.logging.InternalLogger r5 = logger
            java.lang.String r6 = "Failed to close a temporary selector."
            r5.warn(r6, r0)
        L_0x02e0:
            r5 = -1
            return r5
        */
        throw new UnsupportedOperationException("Method not decompiled: org.jboss.netty.channel.socket.nio.NioProviderMetadata.autodetect():int");
    }

    private static final class SelectorLoop implements Runnable {
        volatile boolean done;
        volatile boolean selecting;
        final Selector selector = Selector.open();

        SelectorLoop() throws IOException {
        }

        public void run() {
            while (!this.done) {
                synchronized (this) {
                }
                try {
                    this.selecting = true;
                    this.selector.select(1000);
                    this.selecting = false;
                    Set<SelectionKey> keys = this.selector.selectedKeys();
                    for (SelectionKey k : keys) {
                        k.interestOps(0);
                    }
                    keys.clear();
                } catch (IOException e) {
                    if (NioProviderMetadata.logger.isWarnEnabled()) {
                        NioProviderMetadata.logger.warn("Failed to wait for a temporary selector.", e);
                    }
                } catch (Throwable th) {
                    this.selecting = false;
                    throw th;
                }
            }
        }
    }

    public static void main(String[] args) throws Exception {
        for (Map.Entry<Object, Object> e : System.getProperties().entrySet()) {
            PrintStream printStream = System.out;
            printStream.println(e.getKey() + ": " + e.getValue());
        }
        System.out.println();
        PrintStream printStream2 = System.out;
        printStream2.println("Hard-coded Constraint Level: " + CONSTRAINT_LEVEL);
        PrintStream printStream3 = System.out;
        printStream3.println("Auto-detected Constraint Level: " + autodetect());
    }

    private NioProviderMetadata() {
    }
}
