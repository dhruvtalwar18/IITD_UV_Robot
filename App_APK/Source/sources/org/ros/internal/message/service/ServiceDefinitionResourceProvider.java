package org.ros.internal.message.service;

import com.google.common.base.Preconditions;
import java.util.Collection;
import org.apache.commons.httpclient.cookie.CookieSpec;
import org.ros.internal.message.StringResourceProvider;
import org.ros.message.MessageDefinitionProvider;
import org.ros.message.MessageIdentifier;

public class ServiceDefinitionResourceProvider implements MessageDefinitionProvider {
    private final StringResourceProvider stringResourceProvider = new StringResourceProvider();

    private String serviceTypeToResourceName(String serviceType) {
        boolean contains = serviceType.contains(CookieSpec.PATH_DELIM);
        Preconditions.checkArgument(contains, "Service type must be fully qualified: " + serviceType);
        String[] packageAndType = serviceType.split(CookieSpec.PATH_DELIM, 2);
        return String.format("/%s/srv/%s.srv", new Object[]{packageAndType[0], packageAndType[1]});
    }

    public String get(String serviceType) {
        return this.stringResourceProvider.get(serviceTypeToResourceName(serviceType));
    }

    public boolean has(String serviceType) {
        return this.stringResourceProvider.has(serviceTypeToResourceName(serviceType));
    }

    public void add(String serviceType, String serviceDefinition) {
        this.stringResourceProvider.addStringToCache(serviceTypeToResourceName(serviceType), serviceDefinition);
    }

    public Collection<String> getPackages() {
        throw new UnsupportedOperationException();
    }

    public Collection<MessageIdentifier> getMessageIdentifiersByPackage(String pkg) {
        throw new UnsupportedOperationException();
    }
}
