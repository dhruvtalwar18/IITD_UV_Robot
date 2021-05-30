package org.ros.internal.message;

import com.google.common.annotations.VisibleForTesting;
import com.google.common.collect.Maps;
import java.util.Map;
import org.apache.commons.httpclient.cookie.CookieSpec;

public class DefaultMessageInterfaceClassProvider implements MessageInterfaceClassProvider {
    private final Map<String, Class<?>> cache = Maps.newConcurrentMap();

    public <T> Class<T> get(String messageType) {
        if (this.cache.containsKey(messageType)) {
            return this.cache.get(messageType);
        }
        try {
            Class<?> loadClass = getClass().getClassLoader().loadClass(messageType.replace(CookieSpec.PATH_DELIM, "."));
            this.cache.put(messageType, loadClass);
            return loadClass;
        } catch (ClassNotFoundException e) {
            return RawMessage.class;
        }
    }

    /* access modifiers changed from: package-private */
    @VisibleForTesting
    public <T> void add(String messageType, Class<T> messageInterfaceClass) {
        this.cache.put(messageType, messageInterfaceClass);
    }
}
