package com.github.rosjava.android_remocons.common_tools.rocon;

import android.os.AsyncTask;
import android.util.Log;
import com.github.rosjava.android_remocons.common_tools.master.MasterId;
import java.lang.Thread;
import java.net.URI;
import java.net.URISyntaxException;
import org.ros.address.InetAddressFactory;
import org.ros.android.NodeMainExecutorService;
import org.ros.exception.RosRuntimeException;
import org.ros.exception.ServiceNotFoundException;
import org.ros.internal.node.client.ParameterClient;
import org.ros.internal.node.server.NodeIdentifier;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeConfiguration;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;
import rocon_interaction_msgs.GetInteraction;
import rocon_interaction_msgs.GetInteractionRequest;
import rocon_interaction_msgs.GetInteractionResponse;
import rocon_interaction_msgs.GetInteractions;
import rocon_interaction_msgs.GetInteractionsRequest;
import rocon_interaction_msgs.GetInteractionsResponse;
import rocon_interaction_msgs.Interaction;
import rocon_interaction_msgs.RequestInteraction;
import rocon_interaction_msgs.RequestInteractionRequest;
import rocon_interaction_msgs.RequestInteractionResponse;

public class InteractionsManager extends AbstractNodeMain {
    private Action action = Action.NONE;
    private Interaction app;
    private ServiceResponseListener<GetInteractionResponse> appInfoServiceResponseListener;
    private int app_hash;
    private ConnectNodeThread connectThread;
    private ConnectedNode connectedNode;
    /* access modifiers changed from: private */
    public FailureHandler failureCallback;
    private ServiceResponseListener<GetInteractionsResponse> getAppsServiceResponseListener;
    private String interactionsNamespace;
    /* access modifiers changed from: private */
    public NodeMainExecutorService nodeMainExecutorService;
    private String remocon_name;
    private ServiceResponseListener<RequestInteractionResponse> requestServiceResponseListener;
    private String role;

    public enum Action {
        NONE,
        GET_INTERACTIONS_FOR_ROLE,
        GET_INTERACTION_INFO,
        REQUEST_INTERACTION_USE
    }

    public interface FailureHandler {
        void handleFailure(String str);
    }

    public InteractionsManager(FailureHandler failureCallback2) {
        this.failureCallback = failureCallback2;
    }

    public void init(String interactionsNamespace2) {
        this.interactionsNamespace = interactionsNamespace2;
    }

    public void setRemoconName(String remocon_name2) {
        this.remocon_name = remocon_name2;
    }

    public void setupRequestService(ServiceResponseListener<RequestInteractionResponse> serviceResponseListener) {
        this.requestServiceResponseListener = serviceResponseListener;
    }

    public void setupGetInteractionsService(ServiceResponseListener<GetInteractionsResponse> serviceResponseListener) {
        this.getAppsServiceResponseListener = serviceResponseListener;
    }

    public void setupAppInfoService(ServiceResponseListener<GetInteractionResponse> serviceResponseListener) {
        this.appInfoServiceResponseListener = serviceResponseListener;
    }

    /* access modifiers changed from: protected */
    public void finalize() throws Throwable {
        super.finalize();
        shutdown();
    }

    public void shutdown() {
        if (this.nodeMainExecutorService != null) {
            this.nodeMainExecutorService.shutdownNodeMain(this);
        } else {
            Log.i("InteractionsMng", "Shutting down an uninitialized apps manager");
        }
    }

    public void getAppsForRole(MasterId masterId, String role2) {
        this.action = Action.GET_INTERACTIONS_FOR_ROLE;
        this.role = role2;
        if (this.connectedNode == null) {
            Log.i("InteractionsMng", "First action requested (" + this.action + "). Starting node...");
            new ConnectNodeThread(masterId).start();
            return;
        }
        new AsyncTask<Void, Void, Void>() {
            /* access modifiers changed from: protected */
            public Void doInBackground(Void... params) {
                InteractionsManager.this.getAppsForRole();
                return null;
            }
        }.execute(new Void[0]);
    }

    public void requestAppUse(MasterId masterId, String role2, Interaction app2) {
        this.action = Action.REQUEST_INTERACTION_USE;
        this.role = role2;
        this.app = app2;
        this.app_hash = app2.getHash();
        if (this.connectedNode == null) {
            Log.i("InteractionsMng", "First action requested (" + this.action + "). Starting node...");
            new ConnectNodeThread(masterId).start();
            return;
        }
        new AsyncTask<Void, Void, Void>() {
            /* access modifiers changed from: protected */
            public Void doInBackground(Void... params) {
                InteractionsManager.this.requestAppUse();
                return null;
            }
        }.execute(new Void[0]);
    }

    public void getAppInfo(MasterId masterId, int hash) {
        this.action = Action.GET_INTERACTION_INFO;
        this.app_hash = hash;
        if (this.connectedNode == null) {
            Log.i("InteractionsMng", "First action requested (" + this.action + "). Starting node...");
            new ConnectNodeThread(masterId).start();
            return;
        }
        new AsyncTask<Void, Void, Void>() {
            /* access modifiers changed from: protected */
            public Void doInBackground(Void... params) {
                InteractionsManager.this.getAppInfo();
                return null;
            }
        }.execute(new Void[0]);
    }

    /* access modifiers changed from: private */
    public void getAppsForRole() {
        String serviceName = this.interactionsNamespace + "/get_interactions";
        try {
            Log.i("InteractionsMng", "List interactions service client created [" + serviceName + "]");
            ServiceClient<GetInteractionsRequest, GetInteractionsResponse> srvClient = this.connectedNode.newServiceClient(serviceName, GetInteractions._TYPE);
            GetInteractionsRequest request = srvClient.newMessage();
            request.getRoles().add(this.role);
            request.setUri(Constants.ANDROID_PLATFORM_INFO.getUri());
            srvClient.call(request, this.getAppsServiceResponseListener);
            Log.i("InteractionsMng", "List interactions service call done [" + serviceName + "]");
        } catch (ServiceNotFoundException e) {
            Log.i("InteractionsMng", "List interactions service not found [" + serviceName + "]");
            throw new RosRuntimeException((Throwable) e);
        }
    }

    /* access modifiers changed from: private */
    public void requestAppUse() {
        String serviceName = this.interactionsNamespace + "/request_interaction";
        try {
            Log.i("InteractionsMng", "Request app service client created [" + serviceName + "]");
            ServiceClient<RequestInteractionRequest, RequestInteractionResponse> srvClient = this.connectedNode.newServiceClient(serviceName, RequestInteraction._TYPE);
            RequestInteractionRequest request = srvClient.newMessage();
            request.setRemocon(this.remocon_name);
            request.setHash(this.app.getHash());
            srvClient.call(request, this.requestServiceResponseListener);
            Log.i("InteractionsMng", "Request app service call done [" + serviceName + "]");
        } catch (ServiceNotFoundException e) {
            Log.i("InteractionsMng", "Request app service not found [" + serviceName + "]");
            throw new RosRuntimeException((Throwable) e);
        }
    }

    /* access modifiers changed from: private */
    public void getAppInfo() {
        String serviceName = this.interactionsNamespace + "/get_interaction";
        try {
            Log.i("InteractionsMng", "Get app info service client created [" + serviceName + "]");
            ServiceClient<GetInteractionRequest, GetInteractionResponse> srvClient = this.connectedNode.newServiceClient(serviceName, GetInteraction._TYPE);
            GetInteractionRequest request = srvClient.newMessage();
            request.setHash(this.app_hash);
            srvClient.call(request, this.appInfoServiceResponseListener);
            Log.i("InteractionsMng", "Get app info service call done [" + serviceName + "]");
        } catch (ServiceNotFoundException e) {
            Log.i("InteractionsMng", "Get app info not found [" + serviceName + "]");
            throw new RosRuntimeException((Throwable) e);
        }
    }

    private class ConnectNodeThread extends Thread {
        private MasterId masterId;

        public ConnectNodeThread(MasterId masterId2) {
            this.masterId = masterId2;
            setDaemon(true);
            setUncaughtExceptionHandler(new Thread.UncaughtExceptionHandler(InteractionsManager.this) {
                public void uncaughtException(Thread thread, Throwable ex) {
                    FailureHandler access$300 = InteractionsManager.this.failureCallback;
                    access$300.handleFailure("exception: " + ex.getMessage());
                }
            });
        }

        public void run() {
            try {
                URI concertUri = new URI(this.masterId.getMasterUri());
                Log.i("Remocon", "Concert " + ((String) new ParameterClient(NodeIdentifier.forNameAndUri("/concert_checker", concertUri.toString()), concertUri).getParam(GraphName.of("/rosversion")).getResult()) + " found; retrieve additional information");
                NodeMainExecutorService unused = InteractionsManager.this.nodeMainExecutorService = new NodeMainExecutorService();
                InteractionsManager.this.nodeMainExecutorService.execute(InteractionsManager.this, NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress(), concertUri).setNodeName("apps_manager_node"));
            } catch (URISyntaxException e) {
                Log.i("InteractionsMng", "invalid concert URI [" + this.masterId.getMasterUri() + "][" + e.toString() + "]");
                InteractionsManager.this.failureCallback.handleFailure("invalid concert URI");
            } catch (RuntimeException e2) {
                Log.i("InteractionsMng", "could not find concert [" + this.masterId.getMasterUri() + "][" + e2.toString() + "]");
                InteractionsManager.this.failureCallback.handleFailure(e2.toString());
            } catch (Throwable e3) {
                Log.i("InteractionsMng", "exception while creating node in concert checker for URI " + this.masterId.getMasterUri(), e3);
                InteractionsManager.this.failureCallback.handleFailure(e3.toString());
            }
        }
    }

    public GraphName getDefaultNodeName() {
        return null;
    }

    public void onStart(ConnectedNode connectedNode2) {
        if (this.connectedNode != null) {
            Log.e("InteractionsMng", "App manager re-started before previous shutdown; ignoring...");
            return;
        }
        this.connectedNode = connectedNode2;
        Log.i("InteractionsMng", "onStart() - " + this.action);
        switch (this.action) {
            case NONE:
                Log.i("InteractionsMng", "Node started without specifying an action");
                break;
            case REQUEST_INTERACTION_USE:
                requestAppUse();
                break;
            case GET_INTERACTIONS_FOR_ROLE:
                getAppsForRole();
                break;
            case GET_INTERACTION_INFO:
                getAppInfo();
                break;
            default:
                Log.i("InteractionsMng", "Unrecognised action requested: " + this.action);
                break;
        }
        Log.i("InteractionsMng", "Done");
    }

    public void onShutdown(Node node) {
        Log.i("InteractionsMng", "Shutdown connected node...");
        super.onShutdown(node);
        this.connectedNode = null;
        Log.i("InteractionsMng", "Done; shutdown apps manager node main");
    }

    public void onShutdownComplete(Node node) {
        super.onShutdownComplete(node);
    }

    public void onError(Node node, Throwable throwable) {
        super.onError(node, throwable);
        Log.e("InteractionsMng", node.getName().toString() + " node error: " + throwable.getMessage());
        FailureHandler failureHandler = this.failureCallback;
        failureHandler.handleFailure(node.getName().toString() + " node error: " + throwable.toString());
    }
}
