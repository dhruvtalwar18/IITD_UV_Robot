package org.ros.message;

public interface MessageFactoryProvider {
    MessageFactory get(MessageIdentifier messageIdentifier);

    boolean has(MessageIdentifier messageIdentifier);
}
