package org.ros.internal.message;

import com.google.common.collect.Lists;
import com.google.common.collect.Sets;
import java.io.File;
import java.io.IOException;
import java.util.Collection;
import java.util.List;
import java.util.ListIterator;
import org.apache.commons.io.FileUtils;
import org.ros.exception.RosMessageRuntimeException;
import org.ros.internal.message.action.ActionDefinitionFileProvider;
import org.ros.internal.message.action.ActionGenerationTemplateActionFeedback;
import org.ros.internal.message.action.ActionGenerationTemplateActionGoal;
import org.ros.internal.message.action.ActionGenerationTemplateActionResult;
import org.ros.internal.message.action.ActionGenerationTemplateFeedback;
import org.ros.internal.message.action.ActionGenerationTemplateGoal;
import org.ros.internal.message.action.ActionGenerationTemplateResult;
import org.ros.internal.message.definition.MessageDefinitionProviderChain;
import org.ros.internal.message.definition.MessageDefinitionTupleParser;
import org.ros.internal.message.service.ServiceDefinitionFileProvider;
import org.ros.internal.message.topic.TopicDefinitionFileProvider;
import org.ros.message.MessageDeclaration;
import org.ros.message.MessageFactory;
import org.ros.message.MessageIdentifier;

public class GenerateInterfaces {
    private static final String ROS_PACKAGE_PATH = "ROS_PACKAGE_PATH";
    private final ActionDefinitionFileProvider actionDefinitionFileProvider;
    private final MessageGenerationTemplate actionGenerationTemplateActionFeedback = new ActionGenerationTemplateActionFeedback();
    private final MessageGenerationTemplate actionGenerationTemplateActionGoal = new ActionGenerationTemplateActionGoal();
    private final MessageGenerationTemplate actionGenerationTemplateActionResult = new ActionGenerationTemplateActionResult();
    private final MessageGenerationTemplate actionGenerationTemplateFeedback = new ActionGenerationTemplateFeedback();
    private final MessageGenerationTemplate actionGenerationTemplateGoal = new ActionGenerationTemplateGoal();
    private final MessageGenerationTemplate actionGenerationTemplateResult = new ActionGenerationTemplateResult();
    private final MessageDefinitionProviderChain messageDefinitionProviderChain = new MessageDefinitionProviderChain();
    private final MessageFactory messageFactory;
    private final ServiceDefinitionFileProvider serviceDefinitionFileProvider;
    private final TopicDefinitionFileProvider topicDefinitionFileProvider = new TopicDefinitionFileProvider();

    public GenerateInterfaces() {
        this.messageDefinitionProviderChain.addMessageDefinitionProvider(this.topicDefinitionFileProvider);
        this.serviceDefinitionFileProvider = new ServiceDefinitionFileProvider();
        this.messageDefinitionProviderChain.addMessageDefinitionProvider(this.serviceDefinitionFileProvider);
        this.actionDefinitionFileProvider = new ActionDefinitionFileProvider();
        this.messageDefinitionProviderChain.addMessageDefinitionProvider(this.actionDefinitionFileProvider);
        this.messageFactory = new DefaultMessageFactory(this.messageDefinitionProviderChain);
    }

    private void writeTopicInterfaces(File outputDirectory, Collection<String> packages) throws IOException {
        Collection<MessageIdentifier> topicTypes = Sets.newHashSet();
        if (packages.size() == 0) {
            packages = this.topicDefinitionFileProvider.getPackages();
        }
        for (String pkg : packages) {
            Collection<MessageIdentifier> messageIdentifiers = this.topicDefinitionFileProvider.getMessageIdentifiersByPackage(pkg);
            if (messageIdentifiers != null) {
                topicTypes.addAll(messageIdentifiers);
            }
        }
        for (MessageIdentifier topicType : topicTypes) {
            writeInterface(new MessageDeclaration(topicType, this.messageDefinitionProviderChain.get(topicType.getType())), outputDirectory, true);
        }
    }

    private void writeServiceInterfaces(File outputDirectory, Collection<String> packages) throws IOException {
        Collection<MessageIdentifier> serviceTypes = Sets.newHashSet();
        if (packages.size() == 0) {
            packages = this.serviceDefinitionFileProvider.getPackages();
        }
        for (String pkg : packages) {
            Collection<MessageIdentifier> messageIdentifiers = this.serviceDefinitionFileProvider.getMessageIdentifiersByPackage(pkg);
            if (messageIdentifiers != null) {
                serviceTypes.addAll(messageIdentifiers);
            }
        }
        for (MessageIdentifier serviceType : serviceTypes) {
            String definition = this.messageDefinitionProviderChain.get(serviceType.getType());
            writeInterface(MessageDeclaration.of(serviceType.getType(), definition), outputDirectory, false);
            List<String> requestAndResponse = MessageDefinitionTupleParser.parse(definition, 2);
            MessageDeclaration requestDeclaration = MessageDeclaration.of(serviceType.getType() + "Request", requestAndResponse.get(0));
            MessageDeclaration responseDeclaration = MessageDeclaration.of(serviceType.getType() + "Response", requestAndResponse.get(1));
            writeInterface(requestDeclaration, outputDirectory, true);
            writeInterface(responseDeclaration, outputDirectory, true);
        }
    }

    private void writeActionInterfaces(File outputDirectory, Collection<String> packages) throws IOException {
        Collection<String> packages2;
        File file = outputDirectory;
        Collection<MessageIdentifier> actionTypes = Sets.newHashSet();
        if (packages.size() == 0) {
            packages2 = this.actionDefinitionFileProvider.getPackages();
        } else {
            packages2 = packages;
        }
        for (String pkg : packages2) {
            Collection<MessageIdentifier> messageIdentifiers = this.actionDefinitionFileProvider.getMessageIdentifiersByPackage(pkg);
            if (messageIdentifiers != null) {
                actionTypes.addAll(messageIdentifiers);
            }
        }
        for (MessageIdentifier actionType : actionTypes) {
            String definition = this.messageDefinitionProviderChain.get(actionType.getType());
            writeInterface(MessageDeclaration.of(actionType.getType(), definition), file, false);
            List<String> goalResultAndFeedback = MessageDefinitionTupleParser.parse(definition, 3);
            MessageDeclaration goalDeclaration = MessageDeclaration.of(actionType.getType() + "Goal", this.actionGenerationTemplateGoal.applyTemplate(goalResultAndFeedback.get(0)));
            MessageDeclaration resultDeclaration = MessageDeclaration.of(actionType.getType() + "Result", this.actionGenerationTemplateResult.applyTemplate(goalResultAndFeedback.get(1)));
            MessageDeclaration feedbackDeclaration = MessageDeclaration.of(actionType.getType() + "Feedback", this.actionGenerationTemplateFeedback.applyTemplate(goalResultAndFeedback.get(2)));
            MessageDeclaration actionGoalDeclaration = MessageDeclaration.of(actionType.getType() + "ActionGoal", this.actionGenerationTemplateActionGoal.applyTemplate(actionType.getType()));
            MessageDeclaration actionResultDeclaration = MessageDeclaration.of(actionType.getType() + "ActionResult", this.actionGenerationTemplateActionResult.applyTemplate(actionType.getType()));
            Collection<MessageIdentifier> actionTypes2 = actionTypes;
            MessageDeclaration actionFeedbackDeclaration = MessageDeclaration.of(actionType.getType() + "ActionFeedback", this.actionGenerationTemplateActionFeedback.applyTemplate(actionType.getType()));
            writeInterface(goalDeclaration, file, true);
            writeInterface(resultDeclaration, file, true);
            writeInterface(feedbackDeclaration, file, true);
            writeInterface(actionGoalDeclaration, file, true);
            writeInterface(actionResultDeclaration, file, true);
            writeInterface(actionFeedbackDeclaration, file, true);
            actionTypes = actionTypes2;
        }
    }

    private void writeInterface(MessageDeclaration messageDeclaration, File outputDirectory, boolean addConstantsAndMethods) {
        MessageInterfaceBuilder builder = new MessageInterfaceBuilder();
        builder.setPackageName(messageDeclaration.getPackage());
        builder.setInterfaceName(messageDeclaration.getName());
        builder.setMessageDeclaration(messageDeclaration);
        builder.setAddConstantsAndMethods(addConstantsAndMethods);
        try {
            String content = builder.build(this.messageFactory);
            FileUtils.writeStringToFile(new File(outputDirectory, messageDeclaration.getType() + ".java"), content);
        } catch (Exception e) {
            System.out.printf("Failed to generate interface for %s.\n", new Object[]{messageDeclaration.getType()});
            e.printStackTrace();
        }
    }

    public void generate(File outputDirectory, Collection<String> packages, Collection<File> packagePath) {
        for (File directory : packagePath) {
            this.topicDefinitionFileProvider.addDirectory(directory);
            this.serviceDefinitionFileProvider.addDirectory(directory);
            this.actionDefinitionFileProvider.addDirectory(directory);
        }
        this.topicDefinitionFileProvider.update();
        this.serviceDefinitionFileProvider.update();
        this.actionDefinitionFileProvider.update();
        try {
            writeTopicInterfaces(outputDirectory, packages);
            writeServiceInterfaces(outputDirectory, packages);
            writeActionInterfaces(outputDirectory, packages);
        } catch (IOException e) {
            throw new RosMessageRuntimeException((Throwable) e);
        }
    }

    public static void main(String[] args) {
        List<String> arguments = Lists.newArrayList((E[]) args);
        if (arguments.size() == 0) {
            arguments.add(".");
        }
        String rosPackagePath = System.getenv("ROS_PACKAGE_PATH");
        ListIterator<String> iter = arguments.listIterator();
        while (true) {
            if (!iter.hasNext()) {
                break;
            }
            String arg = iter.next();
            if (arg.contains("--package-path=")) {
                rosPackagePath = arg.replace("--package-path=", "");
                iter.remove();
                break;
            }
        }
        Collection<File> packagePath = Lists.newArrayList();
        for (String path : rosPackagePath.split(File.pathSeparator)) {
            File packageDirectory = new File(path);
            if (packageDirectory.exists()) {
                packagePath.add(packageDirectory);
            }
        }
        new GenerateInterfaces().generate(new File(arguments.remove(0)), arguments, packagePath);
    }
}
