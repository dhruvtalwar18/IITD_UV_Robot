package org.ros.internal.message;

import org.ros.internal.message.service.ServiceRequestMessageFactory;
import org.ros.internal.message.service.ServiceResponseMessageFactory;
import org.ros.message.MessageDefinitionProvider;
import org.ros.message.MessageDeserializer;
import org.ros.message.MessageFactory;
import org.ros.message.MessageIdentifier;
import org.ros.message.MessageSerializationFactory;
import org.ros.message.MessageSerializer;

public class DefaultMessageSerializationFactory implements MessageSerializationFactory {
    private final ServiceRequestMessageFactory serviceRequestMessageFactory;
    private final ServiceResponseMessageFactory serviceResponseMessageFactory;
    private final MessageFactory topicMessageFactory;

    public DefaultMessageSerializationFactory(MessageDefinitionProvider messageDefinitionProvider) {
        this.topicMessageFactory = new DefaultMessageFactory(messageDefinitionProvider);
        this.serviceRequestMessageFactory = new ServiceRequestMessageFactory(messageDefinitionProvider);
        this.serviceResponseMessageFactory = new ServiceResponseMessageFactory(messageDefinitionProvider);
    }

    public <T> MessageSerializer<T> newMessageSerializer(String messageType) {
        return new DefaultMessageSerializer();
    }

    public <T> MessageDeserializer<T> newMessageDeserializer(String messageType) {
        return new DefaultMessageDeserializer(MessageIdentifier.of(messageType), this.topicMessageFactory);
    }

    public <T> MessageSerializer<T> newServiceRequestSerializer(String serviceType) {
        return new DefaultMessageSerializer();
    }

    public <T> MessageDeserializer<T> newServiceRequestDeserializer(String serviceType) {
        return new DefaultMessageDeserializer(MessageIdentifier.of(serviceType), this.serviceRequestMessageFactory);
    }

    public <T> MessageSerializer<T> newServiceResponseSerializer(String serviceType) {
        return new DefaultMessageSerializer();
    }

    public <T> MessageDeserializer<T> newServiceResponseDeserializer(String serviceType) {
        return new DefaultMessageDeserializer(MessageIdentifier.of(serviceType), this.serviceResponseMessageFactory);
    }
}
