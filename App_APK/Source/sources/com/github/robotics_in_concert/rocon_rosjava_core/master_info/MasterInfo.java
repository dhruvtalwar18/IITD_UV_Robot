package com.github.robotics_in_concert.rocon_rosjava_core.master_info;

import com.github.robotics_in_concert.rocon_rosjava_core.rosjava_utils.ListenerException;
import com.github.robotics_in_concert.rocon_rosjava_core.rosjava_utils.ListenerNode;
import com.github.robotics_in_concert.rocon_rosjava_core.rosjava_utils.RosTopicInfo;
import com.google.common.collect.Lists;
import java.io.IOException;
import java.io.PrintStream;
import java.util.concurrent.TimeoutException;
import org.ros.internal.loader.CommandLineLoader;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.DefaultNodeMainExecutor;
import org.ros.node.NodeConfiguration;
import rocon_std_msgs.Icon;

public class MasterInfo extends AbstractNodeMain {
    private ListenerNode<rocon_std_msgs.MasterInfo> masterInfoListener = new ListenerNode<>();

    public void onStart(ConnectedNode connectedNode) {
        this.masterInfoListener.connect(connectedNode, new RosTopicInfo(connectedNode).findTopic(rocon_std_msgs.MasterInfo._TYPE), rocon_std_msgs.MasterInfo._TYPE);
    }

    public void waitForResponse() throws MasterInfoException {
        try {
            this.masterInfoListener.waitForResponse();
        } catch (ListenerException e) {
            throw new MasterInfoException(e.getMessage());
        } catch (TimeoutException e2) {
            throw new MasterInfoException(e2.getMessage());
        }
    }

    public GraphName getDefaultNodeName() {
        return GraphName.of("rocon_rosjava_master_info");
    }

    public String getName() throws MasterInfoException {
        try {
            if (this.masterInfoListener.getMessage() == null) {
                this.masterInfoListener.waitForResponse();
            }
            return this.masterInfoListener.getMessage().getName();
        } catch (ListenerException e) {
            throw new MasterInfoException(e.getMessage());
        } catch (TimeoutException e2) {
            throw new MasterInfoException(e2.getMessage());
        }
    }

    public String getDescription() throws MasterInfoException {
        try {
            if (this.masterInfoListener.getMessage() == null) {
                this.masterInfoListener.waitForResponse();
            }
            return this.masterInfoListener.getMessage().getDescription();
        } catch (ListenerException e) {
            throw new MasterInfoException(e.getMessage());
        } catch (TimeoutException e2) {
            throw new MasterInfoException(e2.getMessage());
        }
    }

    public String getIconResourceName() throws MasterInfoException {
        try {
            if (this.masterInfoListener.getMessage() == null) {
                this.masterInfoListener.waitForResponse();
            }
            return this.masterInfoListener.getMessage().getIcon().getResourceName();
        } catch (ListenerException e) {
            throw new MasterInfoException(e.getMessage());
        } catch (TimeoutException e2) {
            throw new MasterInfoException(e2.getMessage());
        }
    }

    public String getIconFormat() throws MasterInfoException {
        try {
            if (this.masterInfoListener.getMessage() == null) {
                this.masterInfoListener.waitForResponse();
            }
            return this.masterInfoListener.getMessage().getIcon().getFormat();
        } catch (ListenerException e) {
            throw new MasterInfoException(e.getMessage());
        } catch (TimeoutException e2) {
            throw new MasterInfoException(e2.getMessage());
        }
    }

    public Icon getIcon() throws MasterInfoException {
        try {
            if (this.masterInfoListener.getMessage() == null) {
                this.masterInfoListener.waitForResponse();
            }
            return this.masterInfoListener.getMessage().getIcon();
        } catch (ListenerException e) {
            throw new MasterInfoException(e.getMessage());
        } catch (TimeoutException e2) {
            throw new MasterInfoException(e2.getMessage());
        }
    }

    public static void main(String[] argv) throws IOException {
        NodeConfiguration nodeConfiguration = new CommandLineLoader(Lists.newArrayList((E[]) new String[]{"com.github.robotics_in_concert.rocon_rosjava_core.master_info.MasterInfo"})).build();
        MasterInfo masterInfo = new MasterInfo();
        DefaultNodeMainExecutor.newDefault().execute(masterInfo, nodeConfiguration);
        try {
            masterInfo.waitForResponse();
            PrintStream printStream = System.out;
            printStream.println("MasterInfo : retrieved information [" + masterInfo.getName() + "]");
        } catch (MasterInfoException e) {
            PrintStream printStream2 = System.out;
            printStream2.println("Interactions : error getting roles [" + e.getMessage() + "]");
        }
    }
}
