package org.ros.internal.node.server;

import java.io.IOException;
import java.net.InetSocketAddress;
import java.net.URI;
import java.util.concurrent.Callable;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;
import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;
import org.apache.xmlrpc.XmlRpcException;
import org.apache.xmlrpc.server.PropertyHandlerMapping;
import org.apache.xmlrpc.server.XmlRpcServerConfigImpl;
import org.apache.xmlrpc.webserver.WebServer;
import org.ros.address.AdvertiseAddress;
import org.ros.address.BindAddress;
import org.ros.exception.RosRuntimeException;
import org.ros.internal.node.xmlrpc.XmlRpcEndpoint;
import org.ros.internal.system.Process;

public class XmlRpcServer {
    private static final boolean DEBUG = false;
    private static final Log log = LogFactory.getLog(XmlRpcServer.class);
    private final AdvertiseAddress advertiseAddress;
    /* access modifiers changed from: private */
    public final WebServer server;
    private final CountDownLatch startLatch = new CountDownLatch(1);

    public XmlRpcServer(BindAddress bindAddress, AdvertiseAddress advertiseAddress2) {
        InetSocketAddress address = bindAddress.toInetSocketAddress();
        this.server = new WebServer(address.getPort(), address.getAddress());
        this.advertiseAddress = advertiseAddress2;
        this.advertiseAddress.setPortCallable(new Callable<Integer>() {
            public Integer call() throws Exception {
                return Integer.valueOf(XmlRpcServer.this.server.getPort());
            }
        });
    }

    public <T extends XmlRpcEndpoint> void start(Class<T> instanceClass, T instance) {
        org.apache.xmlrpc.server.XmlRpcServer xmlRpcServer = this.server.getXmlRpcServer();
        PropertyHandlerMapping phm = new PropertyHandlerMapping();
        phm.setRequestProcessorFactoryFactory(new NodeRequestProcessorFactoryFactory(instance));
        try {
            phm.addHandler("", instanceClass);
            xmlRpcServer.setHandlerMapping(phm);
            XmlRpcServerConfigImpl serverConfig = (XmlRpcServerConfigImpl) xmlRpcServer.getConfig();
            serverConfig.setEnabledForExtensions(false);
            serverConfig.setContentLengthOptional(false);
            try {
                this.server.start();
                this.startLatch.countDown();
            } catch (IOException e) {
                throw new RosRuntimeException((Throwable) e);
            }
        } catch (XmlRpcException e2) {
            throw new RosRuntimeException((Throwable) e2);
        }
    }

    public void shutdown() {
        this.server.shutdown();
    }

    public URI getUri() {
        return this.advertiseAddress.toUri("http");
    }

    public InetSocketAddress getAddress() {
        return this.advertiseAddress.toInetSocketAddress();
    }

    public AdvertiseAddress getAdvertiseAddress() {
        return this.advertiseAddress;
    }

    public void awaitStart() throws InterruptedException {
        this.startLatch.await();
    }

    public boolean awaitStart(long timeout, TimeUnit unit) throws InterruptedException {
        return this.startLatch.await(timeout, unit);
    }

    public int getPid() {
        return Process.getPid();
    }
}
