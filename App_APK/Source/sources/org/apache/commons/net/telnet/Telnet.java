package org.apache.commons.net.telnet;

import com.google.common.base.Ascii;
import java.io.BufferedInputStream;
import java.io.BufferedOutputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.io.PrintStream;
import org.apache.commons.net.SocketClient;

class Telnet extends SocketClient {
    static final int DEFAULT_PORT = 23;
    protected static final int TERMINAL_TYPE = 24;
    protected static final int TERMINAL_TYPE_IS = 0;
    protected static final int TERMINAL_TYPE_SEND = 1;
    static final byte[] _COMMAND_AYT = {-1, -10};
    static final byte[] _COMMAND_DO = {-1, -3};
    static final byte[] _COMMAND_DONT = {-1, -2};
    static final byte[] _COMMAND_IS = {Ascii.CAN, 0};
    static final byte[] _COMMAND_SB = {-1, -6};
    static final byte[] _COMMAND_SE = {-1, -16};
    static final byte[] _COMMAND_WILL = {-1, -5};
    static final byte[] _COMMAND_WONT = {-1, -4};
    static final int _DO_MASK = 2;
    static final int _REQUESTED_DO_MASK = 8;
    static final int _REQUESTED_WILL_MASK = 4;
    static final int _WILL_MASK = 1;
    static final boolean debug = false;
    static final boolean debugoptions = false;
    private TelnetNotificationHandler __notifhand = null;
    int[] _doResponse;
    int[] _options;
    int[] _willResponse;
    private boolean aytFlag = true;
    private Object aytMonitor = new Object();
    private TelnetOptionHandler[] optionHandlers;
    private OutputStream spyStream = null;
    private String terminalType = null;

    Telnet() {
        setDefaultPort(23);
        this._doResponse = new int[256];
        this._willResponse = new int[256];
        this._options = new int[256];
        this.optionHandlers = new TelnetOptionHandler[256];
    }

    Telnet(String termtype) {
        setDefaultPort(23);
        this._doResponse = new int[256];
        this._willResponse = new int[256];
        this._options = new int[256];
        this.terminalType = termtype;
        this.optionHandlers = new TelnetOptionHandler[256];
    }

    /* access modifiers changed from: package-private */
    public boolean _stateIsWill(int option) {
        return (this._options[option] & 1) != 0;
    }

    /* access modifiers changed from: package-private */
    public boolean _stateIsWont(int option) {
        return !_stateIsWill(option);
    }

    /* access modifiers changed from: package-private */
    public boolean _stateIsDo(int option) {
        return (this._options[option] & 2) != 0;
    }

    /* access modifiers changed from: package-private */
    public boolean _stateIsDont(int option) {
        return !_stateIsDo(option);
    }

    /* access modifiers changed from: package-private */
    public boolean _requestedWill(int option) {
        return (this._options[option] & 4) != 0;
    }

    /* access modifiers changed from: package-private */
    public boolean _requestedWont(int option) {
        return !_requestedWill(option);
    }

    /* access modifiers changed from: package-private */
    public boolean _requestedDo(int option) {
        return (this._options[option] & 8) != 0;
    }

    /* access modifiers changed from: package-private */
    public boolean _requestedDont(int option) {
        return !_requestedDo(option);
    }

    /* access modifiers changed from: package-private */
    public void _setWill(int option) {
        int[] iArr = this._options;
        iArr[option] = iArr[option] | 1;
        if (_requestedWill(option) && this.optionHandlers[option] != null) {
            this.optionHandlers[option].setWill(true);
            int[] subneg = this.optionHandlers[option].startSubnegotiationLocal();
            if (subneg != null) {
                try {
                    _sendSubnegotiation(subneg);
                } catch (IOException e) {
                    PrintStream printStream = System.err;
                    printStream.println("Exception in option subnegotiation" + e.getMessage());
                }
            }
        }
    }

    /* access modifiers changed from: package-private */
    public void _setDo(int option) {
        int[] iArr = this._options;
        iArr[option] = iArr[option] | 2;
        if (_requestedDo(option) && this.optionHandlers[option] != null) {
            this.optionHandlers[option].setDo(true);
            int[] subneg = this.optionHandlers[option].startSubnegotiationRemote();
            if (subneg != null) {
                try {
                    _sendSubnegotiation(subneg);
                } catch (IOException e) {
                    PrintStream printStream = System.err;
                    printStream.println("Exception in option subnegotiation" + e.getMessage());
                }
            }
        }
    }

    /* access modifiers changed from: package-private */
    public void _setWantWill(int option) {
        int[] iArr = this._options;
        iArr[option] = iArr[option] | 4;
    }

    /* access modifiers changed from: package-private */
    public void _setWantDo(int option) {
        int[] iArr = this._options;
        iArr[option] = iArr[option] | 8;
    }

    /* access modifiers changed from: package-private */
    public void _setWont(int option) {
        int[] iArr = this._options;
        iArr[option] = iArr[option] & -2;
        if (this.optionHandlers[option] != null) {
            this.optionHandlers[option].setWill(false);
        }
    }

    /* access modifiers changed from: package-private */
    public void _setDont(int option) {
        int[] iArr = this._options;
        iArr[option] = iArr[option] & -3;
        if (this.optionHandlers[option] != null) {
            this.optionHandlers[option].setDo(false);
        }
    }

    /* access modifiers changed from: package-private */
    public void _setWantWont(int option) {
        int[] iArr = this._options;
        iArr[option] = iArr[option] & -5;
    }

    /* access modifiers changed from: package-private */
    public void _setWantDont(int option) {
        int[] iArr = this._options;
        iArr[option] = iArr[option] & -9;
    }

    /* access modifiers changed from: package-private */
    public void _processDo(int option) throws IOException {
        if (this.__notifhand != null) {
            this.__notifhand.receivedNegotiation(1, option);
        }
        boolean acceptNewState = false;
        if (this.optionHandlers[option] != null) {
            acceptNewState = this.optionHandlers[option].getAcceptLocal();
        } else if (option == 24 && this.terminalType != null && this.terminalType.length() > 0) {
            acceptNewState = true;
        }
        if (this._willResponse[option] > 0) {
            int[] iArr = this._willResponse;
            iArr[option] = iArr[option] - 1;
            if (this._willResponse[option] > 0 && _stateIsWill(option)) {
                int[] iArr2 = this._willResponse;
                iArr2[option] = iArr2[option] - 1;
            }
        }
        if (this._willResponse[option] == 0 && _requestedWont(option)) {
            if (acceptNewState) {
                _setWantWill(option);
                _sendWill(option);
            } else {
                int[] iArr3 = this._willResponse;
                iArr3[option] = iArr3[option] + 1;
                _sendWont(option);
            }
        }
        _setWill(option);
    }

    /* access modifiers changed from: package-private */
    public void _processDont(int option) throws IOException {
        if (this.__notifhand != null) {
            this.__notifhand.receivedNegotiation(2, option);
        }
        if (this._willResponse[option] > 0) {
            int[] iArr = this._willResponse;
            iArr[option] = iArr[option] - 1;
            if (this._willResponse[option] > 0 && _stateIsWont(option)) {
                int[] iArr2 = this._willResponse;
                iArr2[option] = iArr2[option] - 1;
            }
        }
        if (this._willResponse[option] == 0 && _requestedWill(option)) {
            if (_stateIsWill(option) || _requestedWill(option)) {
                _sendWont(option);
            }
            _setWantWont(option);
        }
        _setWont(option);
    }

    /* access modifiers changed from: package-private */
    public void _processWill(int option) throws IOException {
        if (this.__notifhand != null) {
            this.__notifhand.receivedNegotiation(3, option);
        }
        boolean acceptNewState = false;
        if (this.optionHandlers[option] != null) {
            acceptNewState = this.optionHandlers[option].getAcceptRemote();
        }
        if (this._doResponse[option] > 0) {
            int[] iArr = this._doResponse;
            iArr[option] = iArr[option] - 1;
            if (this._doResponse[option] > 0 && _stateIsDo(option)) {
                int[] iArr2 = this._doResponse;
                iArr2[option] = iArr2[option] - 1;
            }
        }
        if (this._doResponse[option] == 0 && _requestedDont(option)) {
            if (acceptNewState) {
                _setWantDo(option);
                _sendDo(option);
            } else {
                int[] iArr3 = this._doResponse;
                iArr3[option] = iArr3[option] + 1;
                _sendDont(option);
            }
        }
        _setDo(option);
    }

    /* access modifiers changed from: package-private */
    public void _processWont(int option) throws IOException {
        if (this.__notifhand != null) {
            this.__notifhand.receivedNegotiation(4, option);
        }
        if (this._doResponse[option] > 0) {
            int[] iArr = this._doResponse;
            iArr[option] = iArr[option] - 1;
            if (this._doResponse[option] > 0 && _stateIsDont(option)) {
                int[] iArr2 = this._doResponse;
                iArr2[option] = iArr2[option] - 1;
            }
        }
        if (this._doResponse[option] == 0 && _requestedDo(option)) {
            if (_stateIsDo(option) || _requestedDo(option)) {
                _sendDont(option);
            }
            _setWantDont(option);
        }
        _setDont(option);
    }

    /* access modifiers changed from: package-private */
    public void _processSuboption(int[] suboption, int suboptionLength) throws IOException {
        if (suboptionLength <= 0) {
            return;
        }
        if (this.optionHandlers[suboption[0]] != null) {
            _sendSubnegotiation(this.optionHandlers[suboption[0]].answerSubnegotiation(suboption, suboptionLength));
        } else if (suboptionLength > 1 && suboption[0] == 24 && suboption[1] == 1) {
            _sendTerminalType();
        }
    }

    /* access modifiers changed from: package-private */
    public final synchronized void _sendTerminalType() throws IOException {
        if (this.terminalType != null) {
            this._output_.write(_COMMAND_SB);
            this._output_.write(_COMMAND_IS);
            this._output_.write(this.terminalType.getBytes());
            this._output_.write(_COMMAND_SE);
            this._output_.flush();
        }
    }

    /* access modifiers changed from: package-private */
    public final synchronized void _sendSubnegotiation(int[] subn) throws IOException {
        if (subn != null) {
            byte[] byteresp = new byte[subn.length];
            for (int ii = 0; ii < subn.length; ii++) {
                byteresp[ii] = (byte) subn[ii];
            }
            this._output_.write(_COMMAND_SB);
            this._output_.write(byteresp);
            this._output_.write(_COMMAND_SE);
            this._output_.flush();
        }
    }

    /* access modifiers changed from: package-private */
    public final synchronized void _processAYTResponse() {
        if (!this.aytFlag) {
            synchronized (this.aytMonitor) {
                this.aytFlag = true;
                try {
                    this.aytMonitor.notifyAll();
                } catch (IllegalMonitorStateException e) {
                    PrintStream printStream = System.err;
                    printStream.println("Exception notifying:" + e.getMessage());
                }
            }
        }
    }

    /* access modifiers changed from: protected */
    public void _connectAction_() throws IOException {
        for (int ii = 0; ii < 256; ii++) {
            this._doResponse[ii] = 0;
            this._willResponse[ii] = 0;
            this._options[ii] = 0;
            if (this.optionHandlers[ii] != null) {
                this.optionHandlers[ii].setDo(false);
                this.optionHandlers[ii].setWill(false);
            }
        }
        super._connectAction_();
        this._input_ = new BufferedInputStream(this._input_);
        this._output_ = new BufferedOutputStream(this._output_);
        for (int ii2 = 0; ii2 < 256; ii2++) {
            if (this.optionHandlers[ii2] != null) {
                if (this.optionHandlers[ii2].getInitLocal()) {
                    try {
                        _requestWill(this.optionHandlers[ii2].getOptionCode());
                    } catch (IOException e) {
                        PrintStream printStream = System.err;
                        printStream.println("Exception while initializing option: " + e.getMessage());
                    }
                }
                if (this.optionHandlers[ii2].getInitRemote()) {
                    try {
                        _requestDo(this.optionHandlers[ii2].getOptionCode());
                    } catch (IOException e2) {
                        PrintStream printStream2 = System.err;
                        printStream2.println("Exception while initializing option: " + e2.getMessage());
                    }
                }
            }
        }
    }

    /* access modifiers changed from: package-private */
    public final synchronized void _sendDo(int option) throws IOException {
        this._output_.write(_COMMAND_DO);
        this._output_.write(option);
        this._output_.flush();
    }

    /* access modifiers changed from: package-private */
    /* JADX WARNING: Code restructure failed: missing block: B:9:0x0014, code lost:
        return;
     */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public final synchronized void _requestDo(int r3) throws java.io.IOException {
        /*
            r2 = this;
            monitor-enter(r2)
            int[] r0 = r2._doResponse     // Catch:{ all -> 0x0025 }
            r0 = r0[r3]     // Catch:{ all -> 0x0025 }
            if (r0 != 0) goto L_0x000d
            boolean r0 = r2._stateIsDo(r3)     // Catch:{ all -> 0x0025 }
            if (r0 != 0) goto L_0x0013
        L_0x000d:
            boolean r0 = r2._requestedDo(r3)     // Catch:{ all -> 0x0025 }
            if (r0 == 0) goto L_0x0015
        L_0x0013:
            monitor-exit(r2)
            return
        L_0x0015:
            r2._setWantDo(r3)     // Catch:{ all -> 0x0025 }
            int[] r0 = r2._doResponse     // Catch:{ all -> 0x0025 }
            r1 = r0[r3]     // Catch:{ all -> 0x0025 }
            int r1 = r1 + 1
            r0[r3] = r1     // Catch:{ all -> 0x0025 }
            r2._sendDo(r3)     // Catch:{ all -> 0x0025 }
            monitor-exit(r2)
            return
        L_0x0025:
            r3 = move-exception
            monitor-exit(r2)
            throw r3
        */
        throw new UnsupportedOperationException("Method not decompiled: org.apache.commons.net.telnet.Telnet._requestDo(int):void");
    }

    /* access modifiers changed from: package-private */
    public final synchronized void _sendDont(int option) throws IOException {
        this._output_.write(_COMMAND_DONT);
        this._output_.write(option);
        this._output_.flush();
    }

    /* access modifiers changed from: package-private */
    /* JADX WARNING: Code restructure failed: missing block: B:9:0x0014, code lost:
        return;
     */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public final synchronized void _requestDont(int r3) throws java.io.IOException {
        /*
            r2 = this;
            monitor-enter(r2)
            int[] r0 = r2._doResponse     // Catch:{ all -> 0x0025 }
            r0 = r0[r3]     // Catch:{ all -> 0x0025 }
            if (r0 != 0) goto L_0x000d
            boolean r0 = r2._stateIsDont(r3)     // Catch:{ all -> 0x0025 }
            if (r0 != 0) goto L_0x0013
        L_0x000d:
            boolean r0 = r2._requestedDont(r3)     // Catch:{ all -> 0x0025 }
            if (r0 == 0) goto L_0x0015
        L_0x0013:
            monitor-exit(r2)
            return
        L_0x0015:
            r2._setWantDont(r3)     // Catch:{ all -> 0x0025 }
            int[] r0 = r2._doResponse     // Catch:{ all -> 0x0025 }
            r1 = r0[r3]     // Catch:{ all -> 0x0025 }
            int r1 = r1 + 1
            r0[r3] = r1     // Catch:{ all -> 0x0025 }
            r2._sendDont(r3)     // Catch:{ all -> 0x0025 }
            monitor-exit(r2)
            return
        L_0x0025:
            r3 = move-exception
            monitor-exit(r2)
            throw r3
        */
        throw new UnsupportedOperationException("Method not decompiled: org.apache.commons.net.telnet.Telnet._requestDont(int):void");
    }

    /* access modifiers changed from: package-private */
    public final synchronized void _sendWill(int option) throws IOException {
        this._output_.write(_COMMAND_WILL);
        this._output_.write(option);
        this._output_.flush();
    }

    /* access modifiers changed from: package-private */
    /* JADX WARNING: Code restructure failed: missing block: B:9:0x0014, code lost:
        return;
     */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public final synchronized void _requestWill(int r3) throws java.io.IOException {
        /*
            r2 = this;
            monitor-enter(r2)
            int[] r0 = r2._willResponse     // Catch:{ all -> 0x0025 }
            r0 = r0[r3]     // Catch:{ all -> 0x0025 }
            if (r0 != 0) goto L_0x000d
            boolean r0 = r2._stateIsWill(r3)     // Catch:{ all -> 0x0025 }
            if (r0 != 0) goto L_0x0013
        L_0x000d:
            boolean r0 = r2._requestedWill(r3)     // Catch:{ all -> 0x0025 }
            if (r0 == 0) goto L_0x0015
        L_0x0013:
            monitor-exit(r2)
            return
        L_0x0015:
            r2._setWantWill(r3)     // Catch:{ all -> 0x0025 }
            int[] r0 = r2._doResponse     // Catch:{ all -> 0x0025 }
            r1 = r0[r3]     // Catch:{ all -> 0x0025 }
            int r1 = r1 + 1
            r0[r3] = r1     // Catch:{ all -> 0x0025 }
            r2._sendWill(r3)     // Catch:{ all -> 0x0025 }
            monitor-exit(r2)
            return
        L_0x0025:
            r3 = move-exception
            monitor-exit(r2)
            throw r3
        */
        throw new UnsupportedOperationException("Method not decompiled: org.apache.commons.net.telnet.Telnet._requestWill(int):void");
    }

    /* access modifiers changed from: package-private */
    public final synchronized void _sendWont(int option) throws IOException {
        this._output_.write(_COMMAND_WONT);
        this._output_.write(option);
        this._output_.flush();
    }

    /* access modifiers changed from: package-private */
    /* JADX WARNING: Code restructure failed: missing block: B:9:0x0014, code lost:
        return;
     */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public final synchronized void _requestWont(int r3) throws java.io.IOException {
        /*
            r2 = this;
            monitor-enter(r2)
            int[] r0 = r2._willResponse     // Catch:{ all -> 0x0025 }
            r0 = r0[r3]     // Catch:{ all -> 0x0025 }
            if (r0 != 0) goto L_0x000d
            boolean r0 = r2._stateIsWont(r3)     // Catch:{ all -> 0x0025 }
            if (r0 != 0) goto L_0x0013
        L_0x000d:
            boolean r0 = r2._requestedWont(r3)     // Catch:{ all -> 0x0025 }
            if (r0 == 0) goto L_0x0015
        L_0x0013:
            monitor-exit(r2)
            return
        L_0x0015:
            r2._setWantWont(r3)     // Catch:{ all -> 0x0025 }
            int[] r0 = r2._doResponse     // Catch:{ all -> 0x0025 }
            r1 = r0[r3]     // Catch:{ all -> 0x0025 }
            int r1 = r1 + 1
            r0[r3] = r1     // Catch:{ all -> 0x0025 }
            r2._sendWont(r3)     // Catch:{ all -> 0x0025 }
            monitor-exit(r2)
            return
        L_0x0025:
            r3 = move-exception
            monitor-exit(r2)
            throw r3
        */
        throw new UnsupportedOperationException("Method not decompiled: org.apache.commons.net.telnet.Telnet._requestWont(int):void");
    }

    /* access modifiers changed from: package-private */
    public final synchronized void _sendByte(int b) throws IOException {
        this._output_.write(b);
        _spyWrite(b);
    }

    /* access modifiers changed from: package-private */
    public final boolean _sendAYT(long timeout) throws IOException, IllegalArgumentException, InterruptedException {
        boolean retValue = false;
        synchronized (this.aytMonitor) {
            synchronized (this) {
                this.aytFlag = false;
                this._output_.write(_COMMAND_AYT);
                this._output_.flush();
            }
            try {
                this.aytMonitor.wait(timeout);
                if (!this.aytFlag) {
                    retValue = false;
                    this.aytFlag = true;
                } else {
                    retValue = true;
                }
            } catch (IllegalMonitorStateException e) {
                PrintStream printStream = System.err;
                printStream.println("Exception processing AYT:" + e.getMessage());
            }
        }
        return retValue;
    }

    /* access modifiers changed from: package-private */
    public void addOptionHandler(TelnetOptionHandler opthand) throws InvalidTelnetOptionException {
        int optcode = opthand.getOptionCode();
        if (!TelnetOption.isValidOption(optcode)) {
            throw new InvalidTelnetOptionException("Invalid Option Code", optcode);
        } else if (this.optionHandlers[optcode] == null) {
            this.optionHandlers[optcode] = opthand;
            if (isConnected()) {
                if (opthand.getInitLocal()) {
                    try {
                        _requestWill(optcode);
                    } catch (IOException e) {
                        PrintStream printStream = System.err;
                        printStream.println("Exception while initializing option: " + e.getMessage());
                    }
                }
                if (opthand.getInitRemote()) {
                    try {
                        _requestDo(optcode);
                    } catch (IOException e2) {
                        PrintStream printStream2 = System.err;
                        printStream2.println("Exception while initializing option: " + e2.getMessage());
                    }
                }
            }
        } else {
            throw new InvalidTelnetOptionException("Already registered option", optcode);
        }
    }

    /* access modifiers changed from: package-private */
    public void deleteOptionHandler(int optcode) throws InvalidTelnetOptionException {
        if (!TelnetOption.isValidOption(optcode)) {
            throw new InvalidTelnetOptionException("Invalid Option Code", optcode);
        } else if (this.optionHandlers[optcode] != null) {
            TelnetOptionHandler opthand = this.optionHandlers[optcode];
            this.optionHandlers[optcode] = null;
            if (opthand.getWill()) {
                try {
                    _requestWont(optcode);
                } catch (IOException e) {
                    PrintStream printStream = System.err;
                    printStream.println("Exception while turning off option: " + e.getMessage());
                }
            }
            if (opthand.getDo()) {
                try {
                    _requestDont(optcode);
                } catch (IOException e2) {
                    PrintStream printStream2 = System.err;
                    printStream2.println("Exception while turning off option: " + e2.getMessage());
                }
            }
        } else {
            throw new InvalidTelnetOptionException("Unregistered option", optcode);
        }
    }

    /* access modifiers changed from: package-private */
    public void _registerSpyStream(OutputStream spystream) {
        this.spyStream = spystream;
    }

    /* access modifiers changed from: package-private */
    public void _stopSpyStream() {
        this.spyStream = null;
    }

    /* access modifiers changed from: package-private */
    public void _spyRead(int ch) {
        if (this.spyStream != null && ch != 13) {
            try {
                this.spyStream.write(ch);
                if (ch == 10) {
                    this.spyStream.write(13);
                }
                this.spyStream.flush();
            } catch (IOException e) {
                this.spyStream = null;
            }
        }
    }

    /* access modifiers changed from: package-private */
    public void _spyWrite(int ch) {
        if ((!_stateIsDo(1) || !_requestedDo(1)) && this.spyStream != null) {
            try {
                this.spyStream.write(ch);
                this.spyStream.flush();
            } catch (IOException e) {
                this.spyStream = null;
            }
        }
    }

    public void registerNotifHandler(TelnetNotificationHandler notifhand) {
        this.__notifhand = notifhand;
    }

    public void unregisterNotifHandler() {
        this.__notifhand = null;
    }
}
