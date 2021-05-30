package org.bytedeco.javacpp.tools;

import java.util.Properties;

public interface BuildEnabled {
    void init(Logger logger, Properties properties, String str);
}
