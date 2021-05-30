package org.apache.xmlrpc.common;

public interface TypeConverter {
    Object backConvert(Object obj);

    Object convert(Object obj);

    boolean isConvertable(Object obj);
}
