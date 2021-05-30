package com.github.robotics_in_concert.rocon_rosjava_core.rosjava_utils;

import java.util.ArrayList;
import java.util.List;
import org.bytedeco.javacpp.opencv_stitching;
import org.ros.exception.RosRuntimeException;
import org.ros.master.client.MasterStateClient;
import org.ros.master.client.TopicType;
import org.ros.node.Node;

public class RosTopicInfo {
    private MasterStateClient master;

    public RosTopicInfo(Node caller) {
        this.master = new MasterStateClient(caller, caller.getMasterUri());
    }

    public List<String> findTopics(String type, double timeout) {
        double duration = opencv_stitching.Stitcher.ORIG_RESOL;
        new ArrayList();
        List<String> topic_names = new ArrayList<>();
        do {
            for (TopicType topic : this.master.getTopicTypes()) {
                if (topic.getMessageType().equals(type)) {
                    topic_names.add(topic.getName());
                }
            }
            if (topic_names.size() > 0) {
                return topic_names;
            }
            try {
                Thread.sleep((long) ((int) (0.1d * 1000.0d)));
                duration += 1000.0d * 0.1d;
            } catch (Exception e) {
                throw new RosRuntimeException((Throwable) e);
            }
        } while (duration <= timeout);
        throw new RosRuntimeException("timed out looking for topics of type [" + type + "]");
    }

    public String findTopic(String type) {
        List<String> topic_names = findTopics(type, 15.0d);
        if (topic_names.size() == 1) {
            return topic_names.get(0);
        }
        throw new RosRuntimeException("couldn't find a unique topic of type  [" + type + "]");
    }
}
