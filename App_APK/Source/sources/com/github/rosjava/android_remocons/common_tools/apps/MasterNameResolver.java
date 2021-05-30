package com.github.rosjava.android_remocons.common_tools.apps;

import android.util.Log;
import com.github.rosjava.android_remocons.common_tools.master.MasterDescription;
import java.util.Iterator;
import org.ros.master.client.MasterStateClient;
import org.ros.master.client.TopicSystemState;
import org.ros.namespace.GraphName;
import org.ros.namespace.NameResolver;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;

public class MasterNameResolver extends AbstractNodeMain {
    private ConnectedNode connectedNode;
    private MasterDescription currentMaster;
    private GraphName masterName;
    private NameResolver masterNameResolver;
    private boolean resolved = false;

    public void setMaster(MasterDescription currentMaster2) {
        this.currentMaster = currentMaster2;
    }

    public GraphName getDefaultNodeName() {
        return null;
    }

    public void setMasterName(String name) {
        this.masterName = GraphName.of(name);
    }

    public String getMasterName() {
        return this.masterName.toString();
    }

    public void resetMasterName(String name) {
        this.masterNameResolver = this.connectedNode.getResolver().newChild(name);
    }

    public NameResolver getMasterNameSpace() {
        return this.masterNameResolver;
    }

    public void waitForResolver() {
        while (!this.resolved) {
            try {
                Thread.sleep(100);
            } catch (Exception e) {
                Log.w("MasterRemocon", "Master name waitForResolver caught an arbitrary exception");
            }
        }
    }

    public void onStart(ConnectedNode connectedNode2) {
        this.connectedNode = connectedNode2;
        if (this.currentMaster == null) {
            Iterator<TopicSystemState> it = new MasterStateClient(this.connectedNode, this.connectedNode.getMasterUri()).getSystemState().getTopics().iterator();
            while (true) {
                if (!it.hasNext()) {
                    break;
                }
                GraphName graph_name = GraphName.of(it.next().getTopicName());
                if (graph_name.getBasename().toString().equals("app_list")) {
                    this.masterName = graph_name.getParent().toRelative();
                    Log.i("ApplicationManagement", "Configuring master namespace resolver [" + this.masterName + "]");
                    break;
                }
            }
        } else {
            this.masterName = GraphName.of(this.currentMaster.getAppsNameSpace());
        }
        this.masterNameResolver = connectedNode2.getResolver().newChild(this.masterName);
        this.resolved = true;
    }
}
