package org.ros.internal.loader;

import com.google.common.base.Preconditions;
import com.google.common.collect.Lists;
import com.google.common.collect.Maps;
import java.io.File;
import java.net.URI;
import java.net.URISyntaxException;
import java.util.Collections;
import java.util.List;
import java.util.Map;
import org.ros.CommandLineVariables;
import org.ros.EnvironmentVariables;
import org.ros.address.InetAddressFactory;
import org.ros.exception.RosRuntimeException;
import org.ros.namespace.GraphName;
import org.ros.namespace.NameResolver;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMain;

public class CommandLineLoader {
    private final List<String> argv;
    private final Map<String, String> environment;
    private final List<String> nodeArguments;
    private String nodeClassName;
    private final List<String> remappingArguments;
    private final Map<GraphName, GraphName> remappings;
    private final Map<String, String> specialRemappings;

    public CommandLineLoader(List<String> argv2) {
        this(argv2, System.getenv());
    }

    public CommandLineLoader(List<String> argv2, Map<String, String> environment2) {
        Preconditions.checkArgument(argv2.size() > 0);
        this.argv = argv2;
        this.environment = environment2;
        this.nodeArguments = Lists.newArrayList();
        this.remappingArguments = Lists.newArrayList();
        this.remappings = Maps.newHashMap();
        this.specialRemappings = Maps.newHashMap();
        parseArgv();
    }

    private void parseArgv() {
        this.nodeClassName = this.argv.get(0);
        for (String argument : this.argv.subList(1, this.argv.size())) {
            if (argument.contains(":=")) {
                this.remappingArguments.add(argument);
            } else {
                this.nodeArguments.add(argument);
            }
        }
    }

    public String getNodeClassName() {
        return this.nodeClassName;
    }

    public List<String> getNodeArguments() {
        return Collections.unmodifiableList(this.nodeArguments);
    }

    public NodeConfiguration build() {
        parseRemappingArguments();
        NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(getHost());
        nodeConfiguration.setParentResolver(buildParentResolver());
        nodeConfiguration.setRosRoot(getRosRoot());
        nodeConfiguration.setRosPackagePath(getRosPackagePath());
        nodeConfiguration.setMasterUri(getMasterUri());
        if (this.specialRemappings.containsKey(CommandLineVariables.NODE_NAME)) {
            nodeConfiguration.setNodeName(this.specialRemappings.get(CommandLineVariables.NODE_NAME));
        }
        return nodeConfiguration;
    }

    private void parseRemappingArguments() {
        for (String remapping : this.remappingArguments) {
            Preconditions.checkState(remapping.contains(":="));
            String[] remap = remapping.split(":=");
            if (remap.length > 2) {
                throw new IllegalArgumentException("Invalid remapping argument: " + remapping);
            } else if (remapping.startsWith("__")) {
                this.specialRemappings.put(remap[0], remap[1]);
            } else {
                this.remappings.put(GraphName.of(remap[0]), GraphName.of(remap[1]));
            }
        }
    }

    private NameResolver buildParentResolver() {
        GraphName namespace = GraphName.root();
        if (this.specialRemappings.containsKey(CommandLineVariables.ROS_NAMESPACE)) {
            namespace = GraphName.of(this.specialRemappings.get(CommandLineVariables.ROS_NAMESPACE)).toGlobal();
        } else if (this.environment.containsKey(EnvironmentVariables.ROS_NAMESPACE)) {
            namespace = GraphName.of(this.environment.get(EnvironmentVariables.ROS_NAMESPACE)).toGlobal();
        }
        return new NameResolver(namespace, this.remappings);
    }

    private String getHost() {
        String host = InetAddressFactory.newLoopback().getHostAddress();
        if (this.specialRemappings.containsKey(CommandLineVariables.ROS_IP)) {
            return this.specialRemappings.get(CommandLineVariables.ROS_IP);
        }
        if (this.environment.containsKey(EnvironmentVariables.ROS_IP)) {
            return this.environment.get(EnvironmentVariables.ROS_IP);
        }
        if (this.environment.containsKey(EnvironmentVariables.ROS_HOSTNAME)) {
            return this.environment.get(EnvironmentVariables.ROS_HOSTNAME);
        }
        return host;
    }

    private URI getMasterUri() {
        URI uri = NodeConfiguration.DEFAULT_MASTER_URI;
        try {
            if (this.specialRemappings.containsKey(CommandLineVariables.ROS_MASTER_URI)) {
                return new URI(this.specialRemappings.get(CommandLineVariables.ROS_MASTER_URI));
            }
            if (this.environment.containsKey(EnvironmentVariables.ROS_MASTER_URI)) {
                return new URI(this.environment.get(EnvironmentVariables.ROS_MASTER_URI));
            }
            return uri;
        } catch (URISyntaxException e) {
            throw new RosRuntimeException("Invalid master URI: " + uri);
        }
    }

    private File getRosRoot() {
        if (this.environment.containsKey(EnvironmentVariables.ROS_ROOT)) {
            return new File(this.environment.get(EnvironmentVariables.ROS_ROOT));
        }
        return null;
    }

    private List<File> getRosPackagePath() {
        if (!this.environment.containsKey(EnvironmentVariables.ROS_PACKAGE_PATH)) {
            return Lists.newArrayList();
        }
        List<File> paths = Lists.newArrayList();
        for (String path : this.environment.get(EnvironmentVariables.ROS_PACKAGE_PATH).split(File.pathSeparator)) {
            paths.add(new File(path));
        }
        return paths;
    }

    public NodeMain loadClass(String name) throws ClassNotFoundException, InstantiationException, IllegalAccessException {
        return NodeMain.class.cast(getClass().getClassLoader().loadClass(name).newInstance());
    }
}
