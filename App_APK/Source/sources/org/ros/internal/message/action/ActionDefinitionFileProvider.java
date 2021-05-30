package org.ros.internal.message.action;

import java.io.File;
import java.io.FileFilter;
import org.apache.commons.io.filefilter.FileFilterUtils;
import org.ros.internal.message.StringFileProvider;
import org.ros.internal.message.definition.MessageDefinitionFileProvider;

public class ActionDefinitionFileProvider extends MessageDefinitionFileProvider {
    private static final String PARENT = "action";
    private static final String SUFFIX = "action";

    private static StringFileProvider newStringFileProvider() {
        return new StringFileProvider(FileFilterUtils.andFileFilter(FileFilterUtils.suffixFileFilter("action"), FileFilterUtils.asFileFilter((FileFilter) new FileFilter() {
            public boolean accept(File file) {
                return ActionDefinitionFileProvider.getParentBaseName(file.getAbsolutePath()).equals("action");
            }
        })));
    }

    public ActionDefinitionFileProvider() {
        super(newStringFileProvider());
    }
}
