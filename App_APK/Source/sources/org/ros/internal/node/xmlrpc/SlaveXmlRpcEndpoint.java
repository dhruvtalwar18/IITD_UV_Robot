package org.ros.internal.node.xmlrpc;

import java.util.List;
import java.util.Map;
import java.util.Vector;

public interface SlaveXmlRpcEndpoint extends XmlRpcEndpoint {
    List<Object> getBusInfo(String str);

    List<Object> getBusStats(String str);

    List<Object> getMasterUri(String str);

    List<Object> getPid(String str);

    List<Object> getPublications(String str);

    List<Object> getSubscriptions(String str);

    List<Object> paramUpdate(String str, String str2, byte b);

    List<Object> paramUpdate(String str, String str2, char c);

    List<Object> paramUpdate(String str, String str2, double d);

    List<Object> paramUpdate(String str, String str2, int i);

    List<Object> paramUpdate(String str, String str2, String str3);

    List<Object> paramUpdate(String str, String str2, List<?> list);

    List<Object> paramUpdate(String str, String str2, Map<?, ?> map);

    List<Object> paramUpdate(String str, String str2, Vector<?> vector);

    List<Object> paramUpdate(String str, String str2, short s);

    List<Object> paramUpdate(String str, String str2, boolean z);

    List<Object> publisherUpdate(String str, String str2, Object[] objArr);

    List<Object> requestTopic(String str, String str2, Object[] objArr);

    List<Object> shutdown(String str, String str2);
}
