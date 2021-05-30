package org.ros.internal.message;

import android.support.v4.app.NotificationCompat;
import com.google.common.collect.Lists;
import java.io.File;
import java.io.IOException;
import java.io.PrintStream;
import java.util.List;
import org.apache.commons.io.FileUtils;
import org.apache.commons.io.FilenameUtils;
import org.ros.exception.RosMessageRuntimeException;
import org.ros.internal.message.definition.MessageDefinitionReflectionProvider;
import org.ros.internal.message.definition.MessageDefinitionTupleParser;
import org.ros.message.MessageDeclaration;
import org.ros.message.MessageFactory;
import org.ros.message.MessageIdentifier;

public class GenerateInterface {
    private static void writeInterface(MessageDeclaration messageDeclaration, File outputDirectory, boolean addConstantsAndMethods, MessageFactory messageFactory) {
        MessageInterfaceBuilder builder = new MessageInterfaceBuilder();
        builder.setPackageName(messageDeclaration.getPackage());
        builder.setInterfaceName(messageDeclaration.getName());
        builder.setMessageDeclaration(messageDeclaration);
        builder.setAddConstantsAndMethods(addConstantsAndMethods);
        try {
            String content = builder.build(messageFactory);
            File file = new File(outputDirectory, messageDeclaration.getType() + ".java");
            PrintStream printStream = System.out;
            printStream.println("Output File: " + file.getAbsolutePath());
            FileUtils.writeStringToFile(file, content);
        } catch (Exception e) {
            System.out.printf("Failed to generate interface for %s.\n", new Object[]{messageDeclaration.getType()});
            e.printStackTrace();
        }
    }

    public static void main(String[] args) {
        List<String> arguments = Lists.newArrayList((E[]) args);
        if (arguments.size() != 3) {
            System.out.println("Incorrect usage, please provide two args: _output_directory_, _pkg_ and _path_to_msg/srv_file_");
            System.exit(1);
        }
        File outputDirectory = new File(arguments.remove(0));
        String pkg = arguments.remove(0);
        File file = new File(arguments.remove(0));
        PrintStream printStream = System.out;
        printStream.println("Output Directory: " + outputDirectory.getAbsolutePath());
        PrintStream printStream2 = System.out;
        printStream2.println("Package: " + pkg);
        PrintStream printStream3 = System.out;
        printStream3.println("Message: " + file.getAbsolutePath());
        String name = FilenameUtils.getBaseName(file.getName());
        String extension = FilenameUtils.getExtension(file.getName());
        PrintStream printStream4 = System.out;
        printStream4.println("  Name: " + name);
        PrintStream printStream5 = System.out;
        printStream5.println("  Extension: " + extension);
        try {
            String definition = FileUtils.readFileToString(file, "US-ASCII");
            MessageIdentifier messageIdentifier = MessageIdentifier.of(pkg, name);
            MessageDeclaration messageDeclaration = new MessageDeclaration(messageIdentifier, definition);
            MessageDefinitionReflectionProvider messageDefinitionProvider = new MessageDefinitionReflectionProvider();
            messageDefinitionProvider.add(messageIdentifier.getType(), definition);
            MessageFactory messageFactory = new DefaultMessageFactory(messageDefinitionProvider);
            if (extension.equals(NotificationCompat.CATEGORY_MESSAGE)) {
                writeInterface(messageDeclaration, outputDirectory, true, messageFactory);
            } else if (extension.equals("srv")) {
                writeInterface(messageDeclaration, outputDirectory, false, messageFactory);
                List<String> requestAndResponse = MessageDefinitionTupleParser.parse(definition, 2);
                MessageDeclaration requestDeclaration = MessageDeclaration.of(messageIdentifier.getType() + "Request", requestAndResponse.get(0));
                MessageDeclaration responseDeclaration = MessageDeclaration.of(messageIdentifier.getType() + "Response", requestAndResponse.get(1));
                writeInterface(requestDeclaration, outputDirectory, true, messageFactory);
                writeInterface(responseDeclaration, outputDirectory, true, messageFactory);
            }
        } catch (IOException e) {
            throw new RosMessageRuntimeException((Throwable) e);
        }
    }
}
