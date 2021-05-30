package com.github.robotics_in_concert.rocon_rosjava_core.rocon_interactions;

import com.github.robotics_in_concert.rocon_rosjava_core.rosjava_utils.BlockingServiceClientNode;
import com.github.robotics_in_concert.rocon_rosjava_core.rosjava_utils.RosTopicInfo;
import com.google.common.collect.Lists;
import java.io.IOException;
import java.io.PrintStream;
import java.util.List;
import org.ros.exception.RosRuntimeException;
import org.ros.exception.ServiceNotFoundException;
import org.ros.internal.loader.CommandLineLoader;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.DefaultNodeMainExecutor;
import org.ros.node.NodeConfiguration;
import rocon_interaction_msgs.GetRoles;
import rocon_interaction_msgs.GetRolesRequest;
import rocon_interaction_msgs.GetRolesResponse;
import rocon_interaction_msgs.InteractiveClients;

public class RoconInteractions extends AbstractNodeMain {
    private BlockingServiceClientNode<GetRolesRequest, GetRolesResponse> getRolesClient = null;
    private GraphName namespace;
    private String roconURI;

    public RoconInteractions(String roconURI2) {
        this.roconURI = roconURI2;
    }

    public void onStart(ConnectedNode connectedNode) {
        try {
            this.namespace = GraphName.of(new RosTopicInfo(connectedNode).findTopic(InteractiveClients._TYPE)).getParent();
            try {
                GetRolesRequest request = (GetRolesRequest) connectedNode.getServiceRequestMessageFactory().newFromType(GetRoles._TYPE);
                request.setUri(this.roconURI);
                synchronized (this) {
                    this.getRolesClient = new BlockingServiceClientNode<>(connectedNode, this.namespace.toString() + "/get_roles", GetRoles._TYPE, request);
                    this.getRolesClient.waitForResponse();
                    notifyAll();
                }
            } catch (ServiceNotFoundException e) {
            }
        } catch (RosRuntimeException e2) {
        }
    }

    public void waitForResponse() throws InteractionsException {
        synchronized (this) {
            try {
                if (this.getRolesClient == null) {
                    wait(4000);
                }
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    public GraphName getDefaultNodeName() {
        return GraphName.of("rocon_rosjava_interactions");
    }

    public List<String> getRoles() throws InteractionsException {
        waitForResponse();
        return this.getRolesClient.getResponse().getRoles();
    }

    public String getNamespace() {
        return this.namespace.toString();
    }

    public static void main(String[] argv) throws IOException {
        NodeConfiguration nodeConfiguration = new CommandLineLoader(Lists.newArrayList((E[]) new String[]{"com.github.rocon_rosjava_core.rocon_interactions.RoconInteractions"})).build();
        RoconInteractions interactions = new RoconInteractions("rocon:/*/*/hydro|indigo/ice_cream_sandwich|jellybean|chrome");
        DefaultNodeMainExecutor.newDefault().execute(interactions, nodeConfiguration);
        try {
            for (String role : interactions.getRoles()) {
                PrintStream printStream = System.out;
                printStream.println("Interactions : found role '" + role + "'");
            }
        } catch (InteractionsException e) {
            PrintStream printStream2 = System.out;
            printStream2.println("Interactions : error getting roles [" + e.getMessage() + "]");
        }
    }
}
