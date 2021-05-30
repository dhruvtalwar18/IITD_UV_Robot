package org.ros.internal.node.response;

import com.google.common.collect.Lists;
import java.util.Arrays;
import java.util.List;
import org.ros.internal.message.topic.TopicDescription;
import org.ros.internal.node.topic.TopicDeclaration;
import org.ros.namespace.GraphName;

public class TopicListResultFactory implements ResultFactory<List<TopicDeclaration>> {
    public List<TopicDeclaration> newFromValue(Object value) {
        List<TopicDeclaration> descriptions = Lists.newArrayList();
        for (Object topic : Arrays.asList((Object[]) value)) {
            descriptions.add(TopicDeclaration.newFromTopicName(GraphName.of((String) ((Object[]) topic)[0]), new TopicDescription((String) ((Object[]) topic)[1], (String) null, (String) null)));
        }
        return descriptions;
    }
}
