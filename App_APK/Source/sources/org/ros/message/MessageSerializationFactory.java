package org.ros.message;

public interface MessageSerializationFactory {
    <T> MessageDeserializer<T> newMessageDeserializer(String str);

    <T> MessageSerializer<T> newMessageSerializer(String str);

    <T> MessageDeserializer<T> newServiceRequestDeserializer(String str);

    <T> MessageSerializer<T> newServiceRequestSerializer(String str);

    <T> MessageDeserializer<T> newServiceResponseDeserializer(String str);

    <T> MessageSerializer<T> newServiceResponseSerializer(String str);
}
