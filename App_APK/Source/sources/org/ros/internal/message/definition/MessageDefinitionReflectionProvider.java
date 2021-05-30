package org.ros.internal.message.definition;

import com.google.common.annotations.VisibleForTesting;
import com.google.common.collect.Maps;
import java.util.Collection;
import java.util.Map;
import org.apache.commons.httpclient.cookie.CookieSpec;
import org.ros.exception.RosMessageRuntimeException;
import org.ros.message.MessageDefinitionProvider;
import org.ros.message.MessageIdentifier;

public class MessageDefinitionReflectionProvider implements MessageDefinitionProvider {
    private static final String DEFINITION_FIELD = "_DEFINITION";
    private final Map<String, String> cache = Maps.newConcurrentMap();

    public String get(String messageType) {
        String messageDefinition = this.cache.get(messageType);
        if (messageDefinition != null) {
            return messageDefinition;
        }
        try {
            String messageDefinition2 = (String) getClass().getClassLoader().loadClass(messageType.replace(CookieSpec.PATH_DELIM, ".")).getDeclaredField(DEFINITION_FIELD).get((Object) null);
            this.cache.put(messageType, messageDefinition2);
            return messageDefinition2;
        } catch (Exception e) {
            throw new RosMessageRuntimeException((Throwable) e);
        }
    }

    public boolean has(String messageType) {
        try {
            get(messageType);
            return true;
        } catch (Exception e) {
            return false;
        }
    }

    public Collection<String> getPackages() {
        throw new UnsupportedOperationException();
    }

    public Collection<MessageIdentifier> getMessageIdentifiersByPackage(String pkg) {
        throw new UnsupportedOperationException();
    }

    @VisibleForTesting
    public void add(String messageType, String messageDefinition) {
        this.cache.put(messageType, messageDefinition);
    }
}
