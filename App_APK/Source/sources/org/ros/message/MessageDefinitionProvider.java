package org.ros.message;

import java.util.Collection;

public interface MessageDefinitionProvider {
    String get(String str);

    Collection<MessageIdentifier> getMessageIdentifiersByPackage(String str);

    Collection<String> getPackages();

    boolean has(String str);
}
