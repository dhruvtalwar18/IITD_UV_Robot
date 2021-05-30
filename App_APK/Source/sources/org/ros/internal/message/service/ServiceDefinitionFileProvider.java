package org.ros.internal.message.service;

import java.io.File;
import java.io.FileFilter;
import org.apache.commons.io.filefilter.FileFilterUtils;
import org.ros.internal.message.StringFileProvider;
import org.ros.internal.message.definition.MessageDefinitionFileProvider;

public class ServiceDefinitionFileProvider extends MessageDefinitionFileProvider {
    private static final String PARENT = "srv";
    private static final String SUFFIX = "srv";

    private static StringFileProvider newStringFileProvider() {
        return new StringFileProvider(FileFilterUtils.andFileFilter(FileFilterUtils.suffixFileFilter("srv"), FileFilterUtils.asFileFilter((FileFilter) new FileFilter() {
            public boolean accept(File file) {
                return ServiceDefinitionFileProvider.getParentBaseName(file.getAbsolutePath()).equals("srv");
            }
        })));
    }

    public ServiceDefinitionFileProvider() {
        super(newStringFileProvider());
    }
}
