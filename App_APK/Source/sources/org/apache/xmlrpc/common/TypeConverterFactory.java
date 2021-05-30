package org.apache.xmlrpc.common;

public interface TypeConverterFactory {
    TypeConverter getTypeConverter(Class cls);
}
