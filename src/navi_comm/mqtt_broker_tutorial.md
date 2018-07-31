1. Host a accessible MQTT broker
    * [Mosquitto MQTT Broker on Linux (www.steves-internet-guide)](http://www.steves-internet-guide.com/install-mosquitto-linux/)
    * Install the Mosquitto Broker.
        ```bash
        sudo apt-add-repository ppa:mosquitto-dev/mosquitto-ppa
        sudo apt-get update
        sudo apt-get install mosquitto
        sudo apt-get install mosquitto-clients
        ```
    * Check Installation Complete
        Check **/etc/init.d/mosquitto** exist
        Use the command below to check mosquitto running:
        ```
        ps ax | grep mosquitto
        ```
        # You should get something like this:
        > 15662 ?        S      0:00 /usr/sbin/mosquitto -c /etc/mosquitto/mosquitto.conf

    * Start and Stop the service
        ```bash
        sudo /etc/init.d/mosquitto start
        sudo /etc/init.d/mosquitto stop
        ```
    * Seeing is believing. Start the mosquitto broker and see the control message on console.
        # If you want to see the control messages on the console then you need to start the mosquitto broker manually from a command line.
        ```bash
        mosquitto -v
        ```
2.  Starting Mosquitto Using a **Configuration file**.
    * You can find the **mosquitto.conf** template file in the **/etc/mosquitto/** folder.
    * Start with the config, it's better to save a file on directory which need no root permision.
    ```bash
    mosquitto -c <filename>
    mosquitto -c /home/ubuntu/mqtt/mosquitto.conf #where I put the config
    ```
   * Config:
        * Setting the Logging File Location
        * Others



3. Client Side
    * Install Dependency
        1. PAHO
        ```
        sudo pip install paho-mqtt
        ```

rostopic pub -r 1 my_topic std_msgs/String "hello there"


## Note:


