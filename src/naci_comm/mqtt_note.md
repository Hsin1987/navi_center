MQTT User Note
===
## Document
- [ MQTT-v3.1.1 PDF](http://docs.oasis-open.org/mqtt/mqtt/v3.1.1/os/mqtt-v3.1.1-os.pdf)
- [MQTT Document (Chinese Version)](https://mcxiaoke.gitbooks.io/mqtt-cn/content/mqtt/01-Introduction.html)


## Key Parts
- MQTT Packet Components
    1. Fixed header
    2. Variable header
        * Packet Identifier:
            * for QoS >0,
    3. Payload
- 14 MQTT Control Packet Type
    1. **CONNECT**, C to S: Client request to connect to server.
    2. **CONNACK**, S to C: Connect Acknowledgment.
    =============================
    3. **PUBLISH**, C to S: Publish message.
    4. **PUBACK**, S to C: Publish Acknowledgement
    5. **PUBREC**, C to S to C: Publish received (assured delivery part 1)
    6. **PUBREL**, C to S to C: Publish release (assured delivery part 2)
    7. **PUBCOMP**, C to S to C: Publish complete (assured delivery part 3)
    =============================
    8. **SUBSCRIBE**, C to S: Client subscribe request
    9. **SUBACK**, S to S, Subscribe acknowledgement
    10. **UNSUBSCRIBE**, C to S: Unsubscribe request
    11. **UNSUBACK**, S to C: Unsubcribe acknowledgment
    =============================
    12. **PINGREQ**, C to S: PING request
    13. **PINGRESP**, S to C: PING response
    14. **DISCONNECT**, C to S: Client is disconnecting

## Control Packet
1. **CONNECT**
    * It must be the 1st packet from C to S.
    * 2nd or more connect request will be regarded as **protocol violation**. Will be disconnected.
    * Connect Flags
        * **Clean Session Flag**: This bit specifies the handling of the Session state.
            - The Client and Server can store Session state to **enable reliable messaging** to continue across a sequence of Network Connections.
            - Set "0": Server MUST Resume communication based on state from the current session.
            - Set "1": Client and Server MUST discard any previous session and start a new one.
            - **The Session State in the Client** consists of:
              - QoS 1 & QoS 2 have been sent to S, but have not been acknowledged.
              - QoS 2 message which have been received from the S, but have not been completely acknowledged.
            - **The Session in the Server** consists of:
                - The existence of a session.
                - The Client's subscriptions.
                - QoS 1, 2 messages have been sent to the C, but have not been acknowledged.
                - QoS 1, 2 messages pending transmission to the C.
                - QoS 1, 2 have been sent received from the C, but have not been acknowledged.
            - Note:
                - To ensure that you do not lose messages while disconnected, use QoS 1 or QoS 2 with **CleanSession** set to 0.
                - When Client want to disconnect, it should reset **CleanSession**  to 1 before disconnection.
        * **Will Flag**: The will message MUST be published when the Network Connection is subsequently closed.
            - to be continued. **Could be helpful when communication network error occurs.**
        * **User Name Flag**:
            - set "0": user name **MUST NOT** be present in the payload.
            - Set "1": user name **MUST** be present in the payload.
        * **Password Flag**
            - to be continued.
        * **Keep Alive**:  it is the maximum time interval between the point at which the Client sent two Control Packet.
            - In the absence of sending any control packet, the Client MUST send a **PINGREQ** Packet.
            - The Server MUST disconnect the Client as if the network had failed once it pass 1.5x setting period.
            - Note: setting **Keep Alive** as "0" means turn off the keep alive mechanism. Then, server is not required to disconnect the client of inactivity.


