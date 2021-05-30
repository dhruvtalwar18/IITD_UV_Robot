package org.ros.internal.message.service;

import org.apache.commons.httpclient.cookie.CookieSpec;
import org.ros.internal.message.MessageInterfaceClassProvider;
import org.ros.internal.message.RawMessage;

public class ServiceResponseMessageInterfaceClassProvider implements MessageInterfaceClassProvider {
    public <T> Class<T> get(String messageType) {
        try {
            return getClass().getClassLoader().loadClass(messageType.replace(CookieSpec.PATH_DELIM, ".") + "$Response");
        } catch (ClassNotFoundException e) {
            return RawMessage.class;
        }
    }
}
