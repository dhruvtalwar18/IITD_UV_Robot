package org.ros.node;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

public abstract class NativeNodeMain extends AbstractNodeMain {
    public static final int SUCCESS = 0;
    protected int executeReturnCode;
    /* access modifiers changed from: private */
    public String hostName;
    private String libName;
    private Log log;
    /* access modifiers changed from: private */
    public String masterUri;
    /* access modifiers changed from: private */
    public String nodeName;
    /* access modifiers changed from: private */
    public String[] remappingArguments;
    protected int shutdownReturnCode;
    /* access modifiers changed from: private */
    public boolean shuttingDown;

    /* access modifiers changed from: protected */
    public abstract int execute(String str, String str2, String str3, String[] strArr);

    /* access modifiers changed from: protected */
    public abstract int shutdown();

    public NativeNodeMain(String libName2, String[] remappings) {
        this.log = LogFactory.getLog(NativeNodeMain.class);
        this.masterUri = null;
        this.hostName = null;
        this.nodeName = null;
        this.shuttingDown = false;
        this.executeReturnCode = 0;
        this.shutdownReturnCode = 0;
        this.libName = libName2;
        if (remappings == null) {
            this.remappingArguments = new String[0];
        } else {
            this.remappingArguments = remappings;
        }
        Log log2 = this.log;
        log2.info("Trying to load native library '" + libName2 + "'...");
        try {
            System.loadLibrary(libName2);
        } catch (SecurityException e) {
            this.log.info("Error loading library! SecurityException");
        } catch (UnsatisfiedLinkError e2) {
            this.log.info("Error loading library! UnsatisfiedLinkError");
        } catch (NullPointerException e3) {
            this.log.info("Error loading library! NullPointerException");
        }
    }

    public NativeNodeMain(String libName2) {
        this(libName2, (String[]) null);
    }

    public void onStart(final ConnectedNode connectedNode) {
        this.masterUri = connectedNode.getMasterUri().toString();
        this.hostName = connectedNode.getUri().getHost();
        this.nodeName = getDefaultNodeName().toString();
        new Thread() {
            public void run() {
                NativeNodeMain.this.executeReturnCode = NativeNodeMain.this.execute(NativeNodeMain.this.masterUri, NativeNodeMain.this.hostName, NativeNodeMain.this.nodeName, NativeNodeMain.this.remappingArguments);
                if (NativeNodeMain.this.executeReturnCode != 0) {
                    NativeNodeMain nativeNodeMain = NativeNodeMain.this;
                    ConnectedNode connectedNode = connectedNode;
                    nativeNodeMain.onError(connectedNode, new Throwable(NativeNodeMain.this.nodeName + " execution error code " + NativeNodeMain.this.executeReturnCode));
                }
                if (!NativeNodeMain.this.shuttingDown) {
                    connectedNode.shutdown();
                }
            }
        }.start();
    }

    public void onShutdown(Node node) {
        this.shuttingDown = true;
        this.shutdownReturnCode = shutdown();
        if (this.shutdownReturnCode != 0) {
            onError(node, new Throwable(this.nodeName + " shutdown error code " + this.shutdownReturnCode));
        }
    }
}
