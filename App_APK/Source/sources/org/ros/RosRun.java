package org.ros;

import com.google.common.base.Preconditions;
import com.google.common.collect.Lists;
import java.io.PrintStream;
import org.ros.exception.RosRuntimeException;
import org.ros.internal.loader.CommandLineLoader;
import org.ros.node.DefaultNodeMainExecutor;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMain;

public class RosRun {
    public static void printUsage() {
        System.err.println("Usage: java -jar my_package.jar com.example.MyNodeMain [args]");
    }

    public static void main(String[] argv) throws Exception {
        boolean z = true;
        if (argv.length == 0) {
            printUsage();
            System.exit(1);
        }
        CommandLineLoader loader = new CommandLineLoader(Lists.newArrayList((E[]) argv));
        String nodeClassName = loader.getNodeClassName();
        PrintStream printStream = System.out;
        printStream.println("Loading node class: " + loader.getNodeClassName());
        NodeConfiguration nodeConfiguration = loader.build();
        try {
            NodeMain nodeMain = loader.loadClass(nodeClassName);
            if (nodeMain == null) {
                z = false;
            }
            Preconditions.checkState(z);
            DefaultNodeMainExecutor.newDefault().execute(nodeMain, nodeConfiguration);
        } catch (ClassNotFoundException e) {
            throw new RosRuntimeException("Unable to locate node: " + nodeClassName, e);
        } catch (InstantiationException e2) {
            throw new RosRuntimeException("Unable to instantiate node: " + nodeClassName, e2);
        } catch (IllegalAccessException e3) {
            throw new RosRuntimeException("Unable to instantiate node: " + nodeClassName, e3);
        }
    }
}
