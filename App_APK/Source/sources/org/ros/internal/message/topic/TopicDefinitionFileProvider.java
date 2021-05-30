package org.ros.internal.message.topic;

import android.support.v4.app.NotificationCompat;
import java.io.File;
import java.io.FileFilter;
import org.apache.commons.io.filefilter.FileFilterUtils;
import org.ros.internal.message.StringFileProvider;
import org.ros.internal.message.definition.MessageDefinitionFileProvider;

public class TopicDefinitionFileProvider extends MessageDefinitionFileProvider {
    private static final String PARENT = "msg";
    private static final String SUFFIX = "msg";

    private static StringFileProvider newStringFileProvider() {
        return new StringFileProvider(FileFilterUtils.andFileFilter(FileFilterUtils.suffixFileFilter(NotificationCompat.CATEGORY_MESSAGE), FileFilterUtils.asFileFilter((FileFilter) new FileFilter() {
            public boolean accept(File file) {
                return TopicDefinitionFileProvider.getParentBaseName(file.getAbsolutePath()).equals(NotificationCompat.CATEGORY_MESSAGE);
            }
        })));
    }

    public TopicDefinitionFileProvider() {
        super(newStringFileProvider());
    }
}
