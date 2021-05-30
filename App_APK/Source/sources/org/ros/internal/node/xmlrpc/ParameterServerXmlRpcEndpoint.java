package org.ros.internal.node.xmlrpc;

import java.util.List;
import java.util.Map;

public interface ParameterServerXmlRpcEndpoint extends XmlRpcEndpoint {
    List<Object> deleteParam(String str, String str2);

    List<Object> getParam(String str, String str2);

    List<Object> getParamNames(String str);

    List<Object> hasParam(String str, String str2);

    List<Object> searchParam(String str, String str2);

    List<Object> setParam(String str, String str2, Boolean bool);

    List<Object> setParam(String str, String str2, Double d);

    List<Object> setParam(String str, String str2, Integer num);

    List<Object> setParam(String str, String str2, String str3);

    List<Object> setParam(String str, String str2, List<?> list);

    List<Object> setParam(String str, String str2, Map<?, ?> map);

    List<Object> subscribeParam(String str, String str2, String str3);

    List<Object> unsubscribeParam(String str, String str2, String str3);
}
