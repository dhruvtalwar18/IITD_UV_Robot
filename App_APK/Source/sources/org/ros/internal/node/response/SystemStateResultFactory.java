package org.ros.internal.node.response;

import com.google.common.collect.Maps;
import com.google.common.collect.Sets;
import java.util.Arrays;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;
import org.ros.master.client.SystemState;
import org.ros.master.client.TopicSystemState;

public class SystemStateResultFactory implements ResultFactory<SystemState> {
    public SystemState newFromValue(Object value) {
        Object[] vals = (Object[]) value;
        Map<String, Set<String>> publisherMap = getPublishers(vals[0]);
        Map<String, Set<String>> subscriberMap = getSubscribers(vals[1]);
        Map<String, TopicSystemState> topics = Maps.newHashMap();
        for (Map.Entry<String, Set<String>> publisherData : publisherMap.entrySet()) {
            String topicName = publisherData.getKey();
            Set<String> subscriberNodes = subscriberMap.remove(topicName);
            if (subscriberNodes == null) {
                subscriberNodes = Sets.newHashSet();
            }
            topics.put(topicName, new TopicSystemState(topicName, publisherData.getValue(), subscriberNodes));
        }
        for (Map.Entry<String, Set<String>> subscriberData : subscriberMap.entrySet()) {
            HashSet<String> noPublishers = Sets.newHashSet();
            String topicName2 = subscriberData.getKey();
            topics.put(topicName2, new TopicSystemState(topicName2, noPublishers, subscriberData.getValue()));
        }
        return new SystemState(topics.values());
    }

    private Map<String, Set<String>> getPublishers(Object pubPairs) {
        Map<String, Set<String>> topicToPublishers = Maps.newHashMap();
        for (Object topicData : Arrays.asList((Object[]) pubPairs)) {
            String topicName = (String) ((Object[]) topicData)[0];
            Set<String> publishers = Sets.newHashSet();
            for (Object publisher : (Object[]) ((Object[]) topicData)[1]) {
                publishers.add(publisher.toString());
            }
            topicToPublishers.put(topicName, publishers);
        }
        return topicToPublishers;
    }

    private Map<String, Set<String>> getSubscribers(Object subPairs) {
        Map<String, Set<String>> topicToSubscribers = Maps.newHashMap();
        for (Object topicData : Arrays.asList((Object[]) subPairs)) {
            String topicName = (String) ((Object[]) topicData)[0];
            Set<String> subscribers = Sets.newHashSet();
            for (Object subscriber : (Object[]) ((Object[]) topicData)[1]) {
                subscribers.add(subscriber.toString());
            }
            topicToSubscribers.put(topicName, subscribers);
        }
        return topicToSubscribers;
    }
}
