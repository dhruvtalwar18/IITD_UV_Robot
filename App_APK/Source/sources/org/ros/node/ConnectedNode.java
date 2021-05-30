package org.ros.node;

import java.net.URI;
import org.ros.exception.ServiceNotFoundException;
import org.ros.message.Time;
import org.ros.namespace.GraphName;
import org.ros.node.parameter.ParameterTree;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseBuilder;
import org.ros.node.service.ServiceServer;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

public interface ConnectedNode extends Node {
    Time getCurrentTime();

    ParameterTree getParameterTree();

    <T, S> ServiceServer<T, S> getServiceServer(String str);

    <T, S> ServiceServer<T, S> getServiceServer(GraphName graphName);

    URI lookupServiceUri(String str);

    URI lookupServiceUri(GraphName graphName);

    <T> Publisher<T> newPublisher(String str, String str2);

    <T> Publisher<T> newPublisher(GraphName graphName, String str);

    <T, S> ServiceClient<T, S> newServiceClient(String str, String str2) throws ServiceNotFoundException;

    <T, S> ServiceClient<T, S> newServiceClient(GraphName graphName, String str) throws ServiceNotFoundException;

    <T, S> ServiceServer<T, S> newServiceServer(String str, String str2, ServiceResponseBuilder<T, S> serviceResponseBuilder);

    <T, S> ServiceServer<T, S> newServiceServer(GraphName graphName, String str, ServiceResponseBuilder<T, S> serviceResponseBuilder);

    <T> Subscriber<T> newSubscriber(String str, String str2);

    <T> Subscriber<T> newSubscriber(GraphName graphName, String str);
}
