package org.apache.xmlrpc;

import java.util.TimeZone;

public interface XmlRpcConfig {
    TimeZone getTimeZone();

    boolean isEnabledForExtensions();
}
