package org.jboss.netty.channel.socket;

import java.io.IOException;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.MulticastSocket;
import java.net.NetworkInterface;
import java.net.SocketException;
import org.jboss.netty.channel.ChannelException;
import org.jboss.netty.channel.DefaultChannelConfig;
import org.jboss.netty.channel.FixedReceiveBufferSizePredictorFactory;
import org.jboss.netty.channel.ReceiveBufferSizePredictor;
import org.jboss.netty.channel.ReceiveBufferSizePredictorFactory;
import org.jboss.netty.util.internal.ConversionUtil;

public class DefaultDatagramChannelConfig extends DefaultChannelConfig implements DatagramChannelConfig {
    private static final ReceiveBufferSizePredictorFactory DEFAULT_PREDICTOR_FACTORY = new FixedReceiveBufferSizePredictorFactory(768);
    private volatile ReceiveBufferSizePredictor predictor;
    private volatile ReceiveBufferSizePredictorFactory predictorFactory = DEFAULT_PREDICTOR_FACTORY;
    private final DatagramSocket socket;

    public DefaultDatagramChannelConfig(DatagramSocket socket2) {
        if (socket2 != null) {
            this.socket = socket2;
            return;
        }
        throw new NullPointerException("socket");
    }

    public boolean setOption(String key, Object value) {
        if (super.setOption(key, value)) {
            return true;
        }
        if (key.equals("broadcast")) {
            setBroadcast(ConversionUtil.toBoolean(value));
        } else if (key.equals("receiveBufferSize")) {
            setReceiveBufferSize(ConversionUtil.toInt(value));
        } else if (key.equals("sendBufferSize")) {
            setSendBufferSize(ConversionUtil.toInt(value));
        } else if (key.equals("receiveBufferSizePredictorFactory")) {
            setReceiveBufferSizePredictorFactory((ReceiveBufferSizePredictorFactory) value);
        } else if (key.equals("receiveBufferSizePredictor")) {
            setReceiveBufferSizePredictor((ReceiveBufferSizePredictor) value);
        } else if (key.equals("reuseAddress")) {
            setReuseAddress(ConversionUtil.toBoolean(value));
        } else if (key.equals("loopbackModeDisabled")) {
            setLoopbackModeDisabled(ConversionUtil.toBoolean(value));
        } else if (key.equals("interface")) {
            setInterface((InetAddress) value);
        } else if (key.equals("networkInterface")) {
            setNetworkInterface((NetworkInterface) value);
        } else if (key.equals("timeToLive")) {
            setTimeToLive(ConversionUtil.toInt(value));
        } else if (!key.equals("trafficClass")) {
            return false;
        } else {
            setTrafficClass(ConversionUtil.toInt(value));
        }
        return true;
    }

    public boolean isBroadcast() {
        try {
            return this.socket.getBroadcast();
        } catch (SocketException e) {
            throw new ChannelException((Throwable) e);
        }
    }

    public void setBroadcast(boolean broadcast) {
        try {
            this.socket.setBroadcast(broadcast);
        } catch (SocketException e) {
            throw new ChannelException((Throwable) e);
        }
    }

    public InetAddress getInterface() {
        if (this.socket instanceof MulticastSocket) {
            try {
                return ((MulticastSocket) this.socket).getInterface();
            } catch (SocketException e) {
                throw new ChannelException((Throwable) e);
            }
        } else {
            throw new UnsupportedOperationException();
        }
    }

    public void setInterface(InetAddress interfaceAddress) {
        if (this.socket instanceof MulticastSocket) {
            try {
                ((MulticastSocket) this.socket).setInterface(interfaceAddress);
            } catch (SocketException e) {
                throw new ChannelException((Throwable) e);
            }
        } else {
            throw new UnsupportedOperationException();
        }
    }

    public boolean isLoopbackModeDisabled() {
        if (this.socket instanceof MulticastSocket) {
            try {
                return ((MulticastSocket) this.socket).getLoopbackMode();
            } catch (SocketException e) {
                throw new ChannelException((Throwable) e);
            }
        } else {
            throw new UnsupportedOperationException();
        }
    }

    public void setLoopbackModeDisabled(boolean loopbackModeDisabled) {
        if (this.socket instanceof MulticastSocket) {
            try {
                ((MulticastSocket) this.socket).setLoopbackMode(loopbackModeDisabled);
            } catch (SocketException e) {
                throw new ChannelException((Throwable) e);
            }
        } else {
            throw new UnsupportedOperationException();
        }
    }

    public NetworkInterface getNetworkInterface() {
        if (this.socket instanceof MulticastSocket) {
            try {
                return ((MulticastSocket) this.socket).getNetworkInterface();
            } catch (SocketException e) {
                throw new ChannelException((Throwable) e);
            }
        } else {
            throw new UnsupportedOperationException();
        }
    }

    public void setNetworkInterface(NetworkInterface networkInterface) {
        if (this.socket instanceof MulticastSocket) {
            try {
                ((MulticastSocket) this.socket).setNetworkInterface(networkInterface);
            } catch (SocketException e) {
                throw new ChannelException((Throwable) e);
            }
        } else {
            throw new UnsupportedOperationException();
        }
    }

    public boolean isReuseAddress() {
        try {
            return this.socket.getReuseAddress();
        } catch (SocketException e) {
            throw new ChannelException((Throwable) e);
        }
    }

    public void setReuseAddress(boolean reuseAddress) {
        try {
            this.socket.setReuseAddress(reuseAddress);
        } catch (SocketException e) {
            throw new ChannelException((Throwable) e);
        }
    }

    public int getReceiveBufferSize() {
        try {
            return this.socket.getReceiveBufferSize();
        } catch (SocketException e) {
            throw new ChannelException((Throwable) e);
        }
    }

    public void setReceiveBufferSize(int receiveBufferSize) {
        try {
            this.socket.setReceiveBufferSize(receiveBufferSize);
        } catch (SocketException e) {
            throw new ChannelException((Throwable) e);
        }
    }

    public int getSendBufferSize() {
        try {
            return this.socket.getSendBufferSize();
        } catch (SocketException e) {
            throw new ChannelException((Throwable) e);
        }
    }

    public void setSendBufferSize(int sendBufferSize) {
        try {
            this.socket.setSendBufferSize(sendBufferSize);
        } catch (SocketException e) {
            throw new ChannelException((Throwable) e);
        }
    }

    public int getTimeToLive() {
        if (this.socket instanceof MulticastSocket) {
            try {
                return ((MulticastSocket) this.socket).getTimeToLive();
            } catch (IOException e) {
                throw new ChannelException((Throwable) e);
            }
        } else {
            throw new UnsupportedOperationException();
        }
    }

    public void setTimeToLive(int ttl) {
        if (this.socket instanceof MulticastSocket) {
            try {
                ((MulticastSocket) this.socket).setTimeToLive(ttl);
            } catch (IOException e) {
                throw new ChannelException((Throwable) e);
            }
        } else {
            throw new UnsupportedOperationException();
        }
    }

    public int getTrafficClass() {
        try {
            return this.socket.getTrafficClass();
        } catch (SocketException e) {
            throw new ChannelException((Throwable) e);
        }
    }

    public void setTrafficClass(int trafficClass) {
        try {
            this.socket.setTrafficClass(trafficClass);
        } catch (SocketException e) {
            throw new ChannelException((Throwable) e);
        }
    }

    public ReceiveBufferSizePredictor getReceiveBufferSizePredictor() {
        ReceiveBufferSizePredictor predictor2 = this.predictor;
        if (predictor2 != null) {
            return predictor2;
        }
        try {
            ReceiveBufferSizePredictor predictor3 = getReceiveBufferSizePredictorFactory().getPredictor();
            ReceiveBufferSizePredictor predictor4 = predictor3;
            this.predictor = predictor3;
            return predictor4;
        } catch (Exception e) {
            throw new ChannelException("Failed to create a new " + ReceiveBufferSizePredictor.class.getSimpleName() + '.', e);
        }
    }

    public void setReceiveBufferSizePredictor(ReceiveBufferSizePredictor predictor2) {
        if (predictor2 != null) {
            this.predictor = predictor2;
            return;
        }
        throw new NullPointerException("predictor");
    }

    public ReceiveBufferSizePredictorFactory getReceiveBufferSizePredictorFactory() {
        return this.predictorFactory;
    }

    public void setReceiveBufferSizePredictorFactory(ReceiveBufferSizePredictorFactory predictorFactory2) {
        if (predictorFactory2 != null) {
            this.predictorFactory = predictorFactory2;
            return;
        }
        throw new NullPointerException("predictorFactory");
    }
}
