package org.ros;

import com.google.common.annotations.VisibleForTesting;
import java.net.URI;
import java.util.concurrent.TimeUnit;
import org.ros.address.AdvertiseAddress;
import org.ros.address.BindAddress;
import org.ros.internal.node.server.master.MasterServer;

public class RosCore {
    private final MasterServer masterServer;

    public static RosCore newPublic(String host, int port) {
        return new RosCore(BindAddress.newPublic(port), new AdvertiseAddress(host));
    }

    public static RosCore newPublic(int port) {
        return new RosCore(BindAddress.newPublic(port), AdvertiseAddress.newPublic());
    }

    public static RosCore newPublic() {
        return new RosCore(BindAddress.newPublic(), AdvertiseAddress.newPublic());
    }

    public static RosCore newPrivate(int port) {
        return new RosCore(BindAddress.newPrivate(port), AdvertiseAddress.newPrivate());
    }

    public static RosCore newPrivate() {
        return new RosCore(BindAddress.newPrivate(), AdvertiseAddress.newPrivate());
    }

    private RosCore(BindAddress bindAddress, AdvertiseAddress advertiseAddress) {
        this.masterServer = new MasterServer(bindAddress, advertiseAddress);
    }

    public void start() {
        this.masterServer.start();
    }

    public URI getUri() {
        return this.masterServer.getUri();
    }

    public void awaitStart() throws InterruptedException {
        this.masterServer.awaitStart();
    }

    public boolean awaitStart(long timeout, TimeUnit unit) throws InterruptedException {
        return this.masterServer.awaitStart(timeout, unit);
    }

    public void shutdown() {
        this.masterServer.shutdown();
    }

    @VisibleForTesting
    public MasterServer getMasterServer() {
        return this.masterServer;
    }
}
