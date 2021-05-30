package org.ros.internal.message.service;

import org.ros.internal.message.DefaultMessageFactory;
import org.ros.internal.message.DefaultMessageInterfaceClassProvider;
import org.ros.internal.message.MessageProxyFactory;
import org.ros.message.MessageDeclaration;
import org.ros.message.MessageDefinitionProvider;
import org.ros.message.MessageFactory;

public class ServiceRequestMessageFactory implements MessageFactory {
    private final MessageFactory messageFactory;
    private final MessageProxyFactory messageProxyFactory = new MessageProxyFactory(new DefaultMessageInterfaceClassProvider(), this.messageFactory);
    private final ServiceDescriptionFactory serviceDescriptionFactory;

    public ServiceRequestMessageFactory(MessageDefinitionProvider messageDefinitionProvider) {
        this.serviceDescriptionFactory = new ServiceDescriptionFactory(messageDefinitionProvider);
        this.messageFactory = new DefaultMessageFactory(messageDefinitionProvider);
    }

    public <T> T newFromType(String serviceType) {
        ServiceDescription serviceDescription = this.serviceDescriptionFactory.newFromType(serviceType);
        return this.messageProxyFactory.newMessageProxy(MessageDeclaration.of(serviceDescription.getRequestType(), serviceDescription.getRequestDefinition()));
    }
}
