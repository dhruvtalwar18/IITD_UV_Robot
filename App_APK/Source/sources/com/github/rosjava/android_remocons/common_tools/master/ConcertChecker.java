package com.github.rosjava.android_remocons.common_tools.master;

import android.util.Log;
import com.github.robotics_in_concert.rocon_rosjava_core.master_info.MasterInfo;
import com.github.robotics_in_concert.rocon_rosjava_core.master_info.MasterInfoException;
import com.github.robotics_in_concert.rocon_rosjava_core.rocon_interactions.InteractionsException;
import com.github.robotics_in_concert.rocon_rosjava_core.rocon_interactions.RoconInteractions;
import com.github.rosjava.android_remocons.common_tools.rocon.Constants;
import java.lang.Thread;
import java.net.URI;
import java.net.URISyntaxException;
import java.util.Date;
import org.ros.address.InetAddressFactory;
import org.ros.android.NodeMainExecutorService;
import org.ros.internal.node.client.ParameterClient;
import org.ros.internal.node.server.NodeIdentifier;
import org.ros.internal.node.xmlrpc.XmlRpcTimeoutException;
import org.ros.namespace.GraphName;
import org.ros.node.NodeConfiguration;

public class ConcertChecker {
    private CheckerThread checkerThread;
    /* access modifiers changed from: private */
    public FailureHandler failureCallback;
    /* access modifiers changed from: private */
    public ConcertDescriptionReceiver foundConcertCallback;

    public interface ConcertDescriptionReceiver {
        void receive(RoconDescription roconDescription);
    }

    public interface FailureHandler {
        void handleFailure(String str);
    }

    public ConcertChecker(ConcertDescriptionReceiver foundConcertCallback2, FailureHandler failureCallback2) {
        this.foundConcertCallback = foundConcertCallback2;
        this.failureCallback = failureCallback2;
    }

    public void beginChecking(MasterId masterId) {
        stopChecking();
        if (masterId.getMasterUri() == null) {
            this.failureCallback.handleFailure("empty concert URI");
            return;
        }
        try {
            this.checkerThread = new CheckerThread(masterId, new URI(masterId.getMasterUri()));
            this.checkerThread.start();
        } catch (URISyntaxException e) {
            this.failureCallback.handleFailure("invalid concert URI");
        }
    }

    public void stopChecking() {
        if (this.checkerThread != null && this.checkerThread.isAlive()) {
            this.checkerThread.interrupt();
        }
    }

    private class CheckerThread extends Thread {
        private URI concertUri;
        private MasterId masterId;

        public CheckerThread(MasterId masterId2, URI concertUri2) {
            this.concertUri = concertUri2;
            this.masterId = masterId2;
            setDaemon(true);
            setUncaughtExceptionHandler(new Thread.UncaughtExceptionHandler(ConcertChecker.this) {
                public void uncaughtException(Thread thread, Throwable ex) {
                    FailureHandler access$000 = ConcertChecker.this.failureCallback;
                    access$000.handleFailure("exception: " + ex.getMessage());
                }
            });
        }

        public void run() {
            try {
                Log.i("Remocon", "r ros master found [" + ((String) new ParameterClient(NodeIdentifier.forNameAndUri("/concert_checker", this.concertUri.toString()), this.concertUri).getParam(GraphName.of("/rosversion")).getResult()) + "]");
                NodeMainExecutorService nodeMainExecutorService = new NodeMainExecutorService();
                NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress(), this.concertUri);
                MasterInfo masterInfo = new MasterInfo();
                RoconInteractions roconInteractions = new RoconInteractions(Constants.ANDROID_PLATFORM_INFO.getUri());
                nodeMainExecutorService.execute(masterInfo, nodeConfiguration.setNodeName("master_info_node"));
                masterInfo.waitForResponse();
                Log.i("Remocon", "master info found");
                nodeMainExecutorService.execute(roconInteractions, nodeConfiguration.setNodeName("rocon_interactions_node"));
                roconInteractions.waitForResponse();
                Log.i("Remocon", "rocon interactions found");
                RoconDescription roconDescription = new RoconDescription(this.masterId, masterInfo.getName(), masterInfo.getDescription(), masterInfo.getIcon(), roconInteractions.getNamespace(), new Date());
                roconDescription.setConnectionStatus(MasterDescription.OK);
                roconDescription.setUserRoles(roconInteractions.getRoles());
                ConcertChecker.this.foundConcertCallback.receive(roconDescription);
                nodeMainExecutorService.shutdownNodeMain(masterInfo);
                nodeMainExecutorService.shutdownNodeMain(roconInteractions);
            } catch (XmlRpcTimeoutException e) {
                Log.w("Remocon", "timed out trying to connect to the master [" + this.concertUri + "][" + e.toString() + "]");
                ConcertChecker.this.failureCallback.handleFailure("Timed out trying to connect to the master. Is your network interface up?");
            } catch (RuntimeException e2) {
                Log.w("Remocon", "connection refused. Is the master running? [" + this.concertUri + "][" + e2.toString() + "]");
                ConcertChecker.this.failureCallback.handleFailure("Connection refused. Is the master running?");
            } catch (InteractionsException e3) {
                Log.w("Remocon", "rocon interactions unavailable [" + this.concertUri + "][" + e3.toString() + "]");
                FailureHandler access$000 = ConcertChecker.this.failureCallback;
                StringBuilder sb = new StringBuilder();
                sb.append("Rocon interactions unavailable [");
                sb.append(e3.toString());
                sb.append("]");
                access$000.handleFailure(sb.toString());
            } catch (MasterInfoException e4) {
                Log.w("Remocon", "master info unavailable [" + this.concertUri + "][" + e4.toString() + "]");
                ConcertChecker.this.failureCallback.handleFailure("Rocon master info unavailable. Is your ROS_IP set? Is the rocon_master_info node running?");
            } catch (Throwable e5) {
                Log.w("Remocon", "exception while creating node in concert checker for URI " + this.concertUri, e5);
                FailureHandler access$0002 = ConcertChecker.this.failureCallback;
                access$0002.handleFailure("unknown exception in the rocon checker [" + e5.toString() + "]");
            }
        }
    }
}
