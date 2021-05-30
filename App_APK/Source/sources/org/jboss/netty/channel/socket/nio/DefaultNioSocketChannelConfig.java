package org.jboss.netty.channel.socket.nio;

import java.net.Socket;
import java.util.Map;
import org.jboss.netty.channel.AdaptiveReceiveBufferSizePredictorFactory;
import org.jboss.netty.channel.ChannelException;
import org.jboss.netty.channel.ReceiveBufferSizePredictor;
import org.jboss.netty.channel.ReceiveBufferSizePredictorFactory;
import org.jboss.netty.channel.socket.DefaultSocketChannelConfig;
import org.jboss.netty.logging.InternalLogger;
import org.jboss.netty.logging.InternalLoggerFactory;
import org.jboss.netty.util.internal.ConversionUtil;

class DefaultNioSocketChannelConfig extends DefaultSocketChannelConfig implements NioSocketChannelConfig {
    private static final ReceiveBufferSizePredictorFactory DEFAULT_PREDICTOR_FACTORY = new AdaptiveReceiveBufferSizePredictorFactory();
    private static final InternalLogger logger = InternalLoggerFactory.getInstance((Class<?>) DefaultNioSocketChannelConfig.class);
    private volatile ReceiveBufferSizePredictor predictor;
    private volatile ReceiveBufferSizePredictorFactory predictorFactory = DEFAULT_PREDICTOR_FACTORY;
    private volatile int writeBufferHighWaterMark = 65536;
    private volatile int writeBufferLowWaterMark = 32768;
    private volatile int writeSpinCount = 16;

    DefaultNioSocketChannelConfig(Socket socket) {
        super(socket);
    }

    public void setOptions(Map<String, Object> options) {
        super.setOptions(options);
        if (getWriteBufferHighWaterMark() < getWriteBufferLowWaterMark()) {
            setWriteBufferLowWaterMark0(getWriteBufferHighWaterMark() >>> 1);
            if (logger.isWarnEnabled()) {
                logger.warn("writeBufferLowWaterMark cannot be greater than writeBufferHighWaterMark; setting to the half of the writeBufferHighWaterMark.");
            }
        }
    }

    public boolean setOption(String key, Object value) {
        if (super.setOption(key, value)) {
            return true;
        }
        if (key.equals("writeBufferHighWaterMark")) {
            setWriteBufferHighWaterMark0(ConversionUtil.toInt(value));
        } else if (key.equals("writeBufferLowWaterMark")) {
            setWriteBufferLowWaterMark0(ConversionUtil.toInt(value));
        } else if (key.equals("writeSpinCount")) {
            setWriteSpinCount(ConversionUtil.toInt(value));
        } else if (key.equals("receiveBufferSizePredictorFactory")) {
            setReceiveBufferSizePredictorFactory((ReceiveBufferSizePredictorFactory) value);
        } else if (!key.equals("receiveBufferSizePredictor")) {
            return false;
        } else {
            setReceiveBufferSizePredictor((ReceiveBufferSizePredictor) value);
        }
        return true;
    }

    public int getWriteBufferHighWaterMark() {
        return this.writeBufferHighWaterMark;
    }

    public void setWriteBufferHighWaterMark(int writeBufferHighWaterMark2) {
        if (writeBufferHighWaterMark2 >= getWriteBufferLowWaterMark()) {
            setWriteBufferHighWaterMark0(writeBufferHighWaterMark2);
            return;
        }
        throw new IllegalArgumentException("writeBufferHighWaterMark cannot be less than writeBufferLowWaterMark (" + getWriteBufferLowWaterMark() + "): " + writeBufferHighWaterMark2);
    }

    private void setWriteBufferHighWaterMark0(int writeBufferHighWaterMark2) {
        if (writeBufferHighWaterMark2 >= 0) {
            this.writeBufferHighWaterMark = writeBufferHighWaterMark2;
            return;
        }
        throw new IllegalArgumentException("writeBufferHighWaterMark: " + writeBufferHighWaterMark2);
    }

    public int getWriteBufferLowWaterMark() {
        return this.writeBufferLowWaterMark;
    }

    public void setWriteBufferLowWaterMark(int writeBufferLowWaterMark2) {
        if (writeBufferLowWaterMark2 <= getWriteBufferHighWaterMark()) {
            setWriteBufferLowWaterMark0(writeBufferLowWaterMark2);
            return;
        }
        throw new IllegalArgumentException("writeBufferLowWaterMark cannot be greater than writeBufferHighWaterMark (" + getWriteBufferHighWaterMark() + "): " + writeBufferLowWaterMark2);
    }

    private void setWriteBufferLowWaterMark0(int writeBufferLowWaterMark2) {
        if (writeBufferLowWaterMark2 >= 0) {
            this.writeBufferLowWaterMark = writeBufferLowWaterMark2;
            return;
        }
        throw new IllegalArgumentException("writeBufferLowWaterMark: " + writeBufferLowWaterMark2);
    }

    public int getWriteSpinCount() {
        return this.writeSpinCount;
    }

    public void setWriteSpinCount(int writeSpinCount2) {
        if (writeSpinCount2 > 0) {
            this.writeSpinCount = writeSpinCount2;
            return;
        }
        throw new IllegalArgumentException("writeSpinCount must be a positive integer.");
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
