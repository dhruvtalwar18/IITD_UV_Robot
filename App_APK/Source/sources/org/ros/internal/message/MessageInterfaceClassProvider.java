package org.ros.internal.message;

public interface MessageInterfaceClassProvider {
    <T> Class<T> get(String str);
}
