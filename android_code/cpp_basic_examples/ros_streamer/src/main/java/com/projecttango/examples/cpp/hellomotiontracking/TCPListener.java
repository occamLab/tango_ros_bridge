package com.projecttango.examples.cpp.hellomotiontracking;

/**
 * Created by parallels on 6/15/17.
 */

public interface TCPListener {
    public void onTCPMessageRecieved(String message);
    public void onTCPConnectionStatusChanged(boolean isConnectedNow);
}
