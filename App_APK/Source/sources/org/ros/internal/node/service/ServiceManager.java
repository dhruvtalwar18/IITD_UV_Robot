package org.ros.internal.node.service;

import com.google.common.collect.ImmutableList;
import com.google.common.collect.Maps;
import java.util.List;
import java.util.Map;
import org.ros.namespace.GraphName;

public class ServiceManager {
    private ServiceManagerListener listener;
    private final Map<GraphName, DefaultServiceClient<?, ?>> serviceClients = Maps.newConcurrentMap();
    private final Map<GraphName, DefaultServiceServer<?, ?>> serviceServers = Maps.newConcurrentMap();

    public void setListener(ServiceManagerListener listener2) {
        this.listener = listener2;
    }

    public boolean hasServer(GraphName name) {
        return this.serviceServers.containsKey(name);
    }

    public void addServer(DefaultServiceServer<?, ?> serviceServer) {
        this.serviceServers.put(serviceServer.getName(), serviceServer);
        if (this.listener != null) {
            this.listener.onServiceServerAdded(serviceServer);
        }
    }

    public void removeServer(DefaultServiceServer<?, ?> serviceServer) {
        this.serviceServers.remove(serviceServer.getName());
        if (this.listener != null) {
            this.listener.onServiceServerRemoved(serviceServer);
        }
    }

    public DefaultServiceServer<?, ?> getServer(GraphName name) {
        return this.serviceServers.get(name);
    }

    public boolean hasClient(GraphName name) {
        return this.serviceClients.containsKey(name);
    }

    public void addClient(DefaultServiceClient<?, ?> serviceClient) {
        this.serviceClients.put(serviceClient.getName(), serviceClient);
    }

    public void removeClient(DefaultServiceClient<?, ?> serviceClient) {
        this.serviceClients.remove(serviceClient.getName());
    }

    public DefaultServiceClient<?, ?> getClient(GraphName name) {
        return this.serviceClients.get(name);
    }

    public List<DefaultServiceServer<?, ?>> getServers() {
        return ImmutableList.copyOf(this.serviceServers.values());
    }

    public List<DefaultServiceClient<?, ?>> getClients() {
        return ImmutableList.copyOf(this.serviceClients.values());
    }
}
