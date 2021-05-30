package org.ros.master.client;

import java.net.URI;
import java.util.List;
import org.ros.internal.node.client.MasterClient;
import org.ros.internal.node.topic.TopicDeclaration;
import org.ros.node.Node;

public class MasterStateClient {
    private final Node caller;
    private final MasterClient masterClient;

    public MasterStateClient(Node caller2, URI masterUri) {
        this.caller = caller2;
        this.masterClient = new MasterClient(masterUri);
    }

    public URI lookupNode(String nodeName) {
        return this.masterClient.lookupNode(this.caller.getName(), nodeName).getResult();
    }

    public URI getUri() {
        return this.masterClient.getUri(this.caller.getName()).getResult();
    }

    public URI lookupService(String serviceName) {
        return this.masterClient.lookupService(this.caller.getName(), serviceName).getResult();
    }

    public List<TopicDeclaration> getPublishedTopics(String subgraph) {
        throw new UnsupportedOperationException();
    }

    public List<TopicType> getTopicTypes() {
        return this.masterClient.getTopicTypes(this.caller.getName()).getResult();
    }

    public SystemState getSystemState() {
        return this.masterClient.getSystemState(this.caller.getName()).getResult();
    }
}
