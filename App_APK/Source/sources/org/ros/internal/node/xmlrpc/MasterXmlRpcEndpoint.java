package org.ros.internal.node.xmlrpc;

import java.util.List;

public interface MasterXmlRpcEndpoint extends XmlRpcEndpoint {
    List<Object> getPid(String str);

    List<Object> getPublishedTopics(String str, String str2);

    List<Object> getSystemState(String str);

    List<Object> getTopicTypes(String str);

    List<Object> getUri(String str);

    List<Object> lookupNode(String str, String str2);

    List<Object> lookupService(String str, String str2);

    List<Object> registerPublisher(String str, String str2, String str3, String str4);

    List<Object> registerService(String str, String str2, String str3, String str4);

    List<Object> registerSubscriber(String str, String str2, String str3, String str4);

    List<Object> unregisterPublisher(String str, String str2, String str3);

    List<Object> unregisterService(String str, String str2, String str3);

    List<Object> unregisterSubscriber(String str, String str2, String str3);
}
