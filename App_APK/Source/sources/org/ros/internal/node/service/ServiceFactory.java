package org.ros.internal.node.service;

import java.util.concurrent.ScheduledExecutorService;
import org.ros.exception.DuplicateServiceException;
import org.ros.internal.node.server.SlaveServer;
import org.ros.message.MessageDeserializer;
import org.ros.message.MessageFactory;
import org.ros.message.MessageSerializer;
import org.ros.namespace.GraphName;
import org.ros.node.service.ServiceResponseBuilder;

public class ServiceFactory {
    private final ScheduledExecutorService executorService;
    private final Object mutex = new Object();
    private final GraphName nodeName;
    private final ServiceManager serviceManager;
    private final SlaveServer slaveServer;

    public ServiceFactory(GraphName nodeName2, SlaveServer slaveServer2, ServiceManager serviceManager2, ScheduledExecutorService executorService2) {
        this.nodeName = nodeName2;
        this.slaveServer = slaveServer2;
        this.serviceManager = serviceManager2;
        this.executorService = executorService2;
    }

    public <T, S> DefaultServiceServer<T, S> newServer(ServiceDeclaration serviceDeclaration, ServiceResponseBuilder<T, S> responseBuilder, MessageDeserializer<T> deserializer, MessageSerializer<S> serializer, MessageFactory messageFactory) {
        DefaultServiceServer defaultServiceServer;
        GraphName name = serviceDeclaration.getName();
        synchronized (this.mutex) {
            if (!this.serviceManager.hasServer(name)) {
                defaultServiceServer = new DefaultServiceServer(serviceDeclaration, responseBuilder, this.slaveServer.getTcpRosAdvertiseAddress(), deserializer, serializer, messageFactory, this.executorService);
                this.serviceManager.addServer(defaultServiceServer);
            } else {
                throw new DuplicateServiceException(String.format("ServiceServer %s already exists.", new Object[]{name}));
            }
        }
        return defaultServiceServer;
    }

    public <T, S> DefaultServiceServer<T, S> getServer(GraphName name) {
        if (this.serviceManager.hasServer(name)) {
            return this.serviceManager.getServer(name);
        }
        return null;
    }

    /* JADX WARNING: Code restructure failed: missing block: B:11:0x0038, code lost:
        if (1 == 0) goto L_0x0041;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:12:0x003a, code lost:
        r3.connect(r11.getUri());
     */
    /* JADX WARNING: Code restructure failed: missing block: B:13:0x0041, code lost:
        return r3;
     */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public <T, S> org.ros.internal.node.service.DefaultServiceClient<T, S> newClient(org.ros.internal.node.service.ServiceDeclaration r11, org.ros.message.MessageSerializer<T> r12, org.ros.message.MessageDeserializer<S> r13, org.ros.message.MessageFactory r14) {
        /*
            r10 = this;
            java.net.URI r0 = r11.getUri()
            com.google.common.base.Preconditions.checkNotNull(r0)
            org.ros.namespace.GraphName r0 = r11.getName()
            r1 = 0
            java.lang.Object r2 = r10.mutex
            monitor-enter(r2)
            org.ros.internal.node.service.ServiceManager r3 = r10.serviceManager     // Catch:{ all -> 0x0042 }
            boolean r3 = r3.hasClient(r0)     // Catch:{ all -> 0x0042 }
            if (r3 == 0) goto L_0x0025
            org.ros.internal.node.service.ServiceManager r3 = r10.serviceManager     // Catch:{ all -> 0x0042 }
            org.ros.internal.node.service.DefaultServiceClient r3 = r3.getClient(r0)     // Catch:{ all -> 0x0042 }
            boolean r4 = r3.isConnected()     // Catch:{ all -> 0x0042 }
            if (r4 == 0) goto L_0x0025
            monitor-exit(r2)     // Catch:{ all -> 0x0042 }
            return r3
        L_0x0025:
            org.ros.namespace.GraphName r4 = r10.nodeName     // Catch:{ all -> 0x0042 }
            java.util.concurrent.ScheduledExecutorService r9 = r10.executorService     // Catch:{ all -> 0x0042 }
            r5 = r11
            r6 = r12
            r7 = r13
            r8 = r14
            org.ros.internal.node.service.DefaultServiceClient r3 = org.ros.internal.node.service.DefaultServiceClient.newDefault(r4, r5, r6, r7, r8, r9)     // Catch:{ all -> 0x0042 }
            org.ros.internal.node.service.ServiceManager r4 = r10.serviceManager     // Catch:{ all -> 0x0042 }
            r4.addClient(r3)     // Catch:{ all -> 0x0042 }
            r1 = 1
            monitor-exit(r2)     // Catch:{ all -> 0x0042 }
            if (r1 == 0) goto L_0x0041
            java.net.URI r2 = r11.getUri()
            r3.connect(r2)
        L_0x0041:
            return r3
        L_0x0042:
            r3 = move-exception
            monitor-exit(r2)     // Catch:{ all -> 0x0042 }
            throw r3
        */
        throw new UnsupportedOperationException("Method not decompiled: org.ros.internal.node.service.ServiceFactory.newClient(org.ros.internal.node.service.ServiceDeclaration, org.ros.message.MessageSerializer, org.ros.message.MessageDeserializer, org.ros.message.MessageFactory):org.ros.internal.node.service.DefaultServiceClient");
    }
}
