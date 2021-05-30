package org.ros.internal.node.topic;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;
import org.ros.exception.RemoteException;
import org.ros.internal.node.client.SlaveClient;
import org.ros.internal.node.response.Response;
import org.ros.internal.node.server.NodeIdentifier;
import org.ros.internal.node.xmlrpc.XmlRpcTimeoutException;
import org.ros.internal.transport.ProtocolDescription;
import org.ros.internal.transport.ProtocolNames;

class UpdatePublisherRunnable<MessageType> implements Runnable {
    private static final Log log = LogFactory.getLog(UpdatePublisherRunnable.class);
    private final NodeIdentifier nodeIdentifier;
    private final PublisherIdentifier publisherIdentifier;
    private final DefaultSubscriber<MessageType> subscriber;

    public UpdatePublisherRunnable(DefaultSubscriber<MessageType> subscriber2, NodeIdentifier nodeIdentifier2, PublisherIdentifier publisherIdentifier2) {
        this.subscriber = subscriber2;
        this.nodeIdentifier = nodeIdentifier2;
        this.publisherIdentifier = publisherIdentifier2;
    }

    public void run() {
        try {
            Response<ProtocolDescription> response = new SlaveClient(this.nodeIdentifier.getName(), this.publisherIdentifier.getNodeUri()).requestTopic(this.subscriber.getTopicName(), ProtocolNames.SUPPORTED);
            ProtocolDescription selected = response.getResult();
            if (ProtocolNames.SUPPORTED.contains(selected.getName())) {
                this.subscriber.addPublisher(this.publisherIdentifier, selected.getAddress());
                return;
            }
            Log log2 = log;
            log2.error("Publisher returned unsupported protocol selection: " + response);
        } catch (RemoteException e) {
            log.error(e);
        } catch (XmlRpcTimeoutException e2) {
            log.error(e2);
        } catch (RuntimeException e3) {
            log.error(e3);
        }
    }
}
