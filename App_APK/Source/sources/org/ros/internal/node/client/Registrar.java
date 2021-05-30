package org.ros.internal.node.client;

import com.google.common.base.Preconditions;
import java.net.URI;
import java.util.Collection;
import java.util.List;
import java.util.concurrent.Callable;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;
import org.ros.concurrent.Holder;
import org.ros.concurrent.RetryingExecutorService;
import org.ros.exception.RosRuntimeException;
import org.ros.internal.node.response.Response;
import org.ros.internal.node.server.NodeIdentifier;
import org.ros.internal.node.service.DefaultServiceServer;
import org.ros.internal.node.service.ServiceManagerListener;
import org.ros.internal.node.topic.DefaultPublisher;
import org.ros.internal.node.topic.DefaultSubscriber;
import org.ros.internal.node.topic.PublisherIdentifier;
import org.ros.internal.node.topic.TopicParticipantManagerListener;

public class Registrar implements TopicParticipantManagerListener, ServiceManagerListener {
    private static final boolean DEBUG = true;
    private static final int SHUTDOWN_TIMEOUT = 5;
    private static final TimeUnit SHUTDOWN_TIMEOUT_UNITS = TimeUnit.SECONDS;
    private static final Log log = LogFactory.getLog(Registrar.class);
    private final ScheduledExecutorService executorService;
    /* access modifiers changed from: private */
    public final MasterClient masterClient;
    /* access modifiers changed from: private */
    public NodeIdentifier nodeIdentifier = null;
    private final RetryingExecutorService retryingExecutorService;
    private boolean running = false;

    public Registrar(MasterClient masterClient2, ScheduledExecutorService executorService2) {
        this.masterClient = masterClient2;
        this.executorService = executorService2;
        this.retryingExecutorService = new RetryingExecutorService(executorService2);
        Log log2 = log;
        log2.info("MasterXmlRpcEndpoint URI: " + masterClient2.getRemoteUri());
    }

    public void setRetryDelay(long delay, TimeUnit unit) {
        this.retryingExecutorService.setRetryDelay(delay, unit);
    }

    private boolean submit(Callable<Boolean> callable) {
        if (this.running) {
            this.retryingExecutorService.submit(callable);
            return true;
        }
        log.warn("Registrar no longer running, request ignored.");
        return false;
    }

    /* access modifiers changed from: private */
    public <T> boolean callMaster(Callable<Response<T>> callable) {
        Preconditions.checkNotNull(this.nodeIdentifier, "Registrar not started.");
        try {
            Response<T> response = callable.call();
            log.info(response);
            return response.isSuccess();
        } catch (Exception e) {
            log.error("Exception caught while communicating with master.", e);
            return false;
        }
    }

    public void onPublisherAdded(final DefaultPublisher<?> publisher) {
        Log log2 = log;
        log2.info("Registering publisher: " + publisher);
        if (!submit(new Callable<Boolean>() {
            public Boolean call() throws Exception {
                boolean success = Registrar.this.callMaster(new Callable<Response<List<URI>>>() {
                    public Response<List<URI>> call() throws Exception {
                        return Registrar.this.masterClient.registerPublisher(publisher.toDeclaration());
                    }
                });
                if (success) {
                    publisher.signalOnMasterRegistrationSuccess();
                } else {
                    publisher.signalOnMasterRegistrationFailure();
                }
                return Boolean.valueOf(!success);
            }
        })) {
            this.executorService.execute(new Runnable() {
                public void run() {
                    publisher.signalOnMasterRegistrationFailure();
                }
            });
        }
    }

    public void onPublisherRemoved(final DefaultPublisher<?> publisher) {
        Log log2 = log;
        log2.info("Unregistering publisher: " + publisher);
        if (!submit(new Callable<Boolean>() {
            public Boolean call() throws Exception {
                boolean success = Registrar.this.callMaster(new Callable<Response<Integer>>() {
                    public Response<Integer> call() throws Exception {
                        return Registrar.this.masterClient.unregisterPublisher(publisher.getIdentifier());
                    }
                });
                if (success) {
                    publisher.signalOnMasterUnregistrationSuccess();
                } else {
                    publisher.signalOnMasterUnregistrationFailure();
                }
                return Boolean.valueOf(!success);
            }
        })) {
            this.executorService.execute(new Runnable() {
                public void run() {
                    publisher.signalOnMasterUnregistrationFailure();
                }
            });
        }
    }

    public void onSubscriberAdded(final DefaultSubscriber<?> subscriber) {
        Log log2 = log;
        log2.info("Registering subscriber: " + subscriber);
        if (!submit(new Callable<Boolean>() {
            public Boolean call() throws Exception {
                final Holder<Response<List<URI>>> holder = Holder.newEmpty();
                boolean success = Registrar.this.callMaster(new Callable<Response<List<URI>>>() {
                    public Response<List<URI>> call() throws Exception {
                        return (Response) holder.set(Registrar.this.masterClient.registerSubscriber(Registrar.this.nodeIdentifier, subscriber));
                    }
                });
                if (success) {
                    subscriber.updatePublishers(PublisherIdentifier.newCollectionFromUris((Collection) holder.get().getResult(), subscriber.getTopicDeclaration()));
                    subscriber.signalOnMasterRegistrationSuccess();
                } else {
                    subscriber.signalOnMasterRegistrationFailure();
                }
                return Boolean.valueOf(!success);
            }
        })) {
            this.executorService.execute(new Runnable() {
                public void run() {
                    subscriber.signalOnMasterRegistrationFailure();
                }
            });
        }
    }

    public void onSubscriberRemoved(final DefaultSubscriber<?> subscriber) {
        Log log2 = log;
        log2.info("Unregistering subscriber: " + subscriber);
        if (!submit(new Callable<Boolean>() {
            public Boolean call() throws Exception {
                boolean success = Registrar.this.callMaster(new Callable<Response<Integer>>() {
                    public Response<Integer> call() throws Exception {
                        return Registrar.this.masterClient.unregisterSubscriber(Registrar.this.nodeIdentifier, subscriber);
                    }
                });
                if (success) {
                    subscriber.signalOnMasterUnregistrationSuccess();
                } else {
                    subscriber.signalOnMasterUnregistrationFailure();
                }
                return Boolean.valueOf(!success);
            }
        })) {
            this.executorService.execute(new Runnable() {
                public void run() {
                    subscriber.signalOnMasterUnregistrationFailure();
                }
            });
        }
    }

    public void onServiceServerAdded(final DefaultServiceServer<?, ?> serviceServer) {
        Log log2 = log;
        log2.info("Registering service: " + serviceServer);
        if (!submit(new Callable<Boolean>() {
            public Boolean call() throws Exception {
                boolean success = Registrar.this.callMaster(new Callable<Response<Void>>() {
                    public Response<Void> call() throws Exception {
                        return Registrar.this.masterClient.registerService(Registrar.this.nodeIdentifier, serviceServer);
                    }
                });
                if (success) {
                    serviceServer.signalOnMasterRegistrationSuccess();
                } else {
                    serviceServer.signalOnMasterRegistrationFailure();
                }
                return Boolean.valueOf(!success);
            }
        })) {
            this.executorService.execute(new Runnable() {
                public void run() {
                    serviceServer.signalOnMasterRegistrationFailure();
                }
            });
        }
    }

    public void onServiceServerRemoved(final DefaultServiceServer<?, ?> serviceServer) {
        Log log2 = log;
        log2.info("Unregistering service: " + serviceServer);
        if (!submit(new Callable<Boolean>() {
            public Boolean call() throws Exception {
                boolean success = Registrar.this.callMaster(new Callable<Response<Integer>>() {
                    public Response<Integer> call() throws Exception {
                        return Registrar.this.masterClient.unregisterService(Registrar.this.nodeIdentifier, serviceServer);
                    }
                });
                if (success) {
                    serviceServer.signalOnMasterUnregistrationSuccess();
                } else {
                    serviceServer.signalOnMasterUnregistrationFailure();
                }
                return Boolean.valueOf(!success);
            }
        })) {
            this.executorService.execute(new Runnable() {
                public void run() {
                    serviceServer.signalOnMasterUnregistrationFailure();
                }
            });
        }
    }

    public void start(NodeIdentifier nodeIdentifier2) {
        Preconditions.checkNotNull(nodeIdentifier2);
        Preconditions.checkState(this.nodeIdentifier == null, "Registrar already started.");
        this.nodeIdentifier = nodeIdentifier2;
        this.running = true;
    }

    public void shutdown() {
        if (this.running) {
            this.running = false;
            try {
                this.retryingExecutorService.shutdown(5, SHUTDOWN_TIMEOUT_UNITS);
            } catch (InterruptedException e) {
                throw new RosRuntimeException((Throwable) e);
            }
        }
    }
}
