package org.ros.internal.message.action;

import org.ros.internal.message.MessageGenerationTemplate;

public class ActionGenerationTemplateActionFeedback implements MessageGenerationTemplate {
    public String applyTemplate(String messageSource) {
        return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\nHeader header\nactionlib_msgs/GoalStatus status\n" + messageSource + "Feedback feedback";
    }
}
