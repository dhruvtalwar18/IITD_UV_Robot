package org.ros.internal.node.response;

import com.google.common.collect.Lists;
import java.util.List;
import org.ros.master.client.TopicType;

public class TopicTypeListResultFactory implements ResultFactory<List<TopicType>> {
    public List<TopicType> newFromValue(Object value) {
        List<TopicType> topics = Lists.newArrayList();
        for (Object pair : (Object[]) value) {
            topics.add(new TopicType((String) ((Object[]) pair)[0], (String) ((Object[]) pair)[1]));
        }
        return topics;
    }
}
