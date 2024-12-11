#include <WiFi.h>
#include <PubSubClient.h>

// WiFi credentials
const char *ssid = "Inflight_WiFi"; ///< Wi-Fi SSID (network name)
const char *password = "12345678";  ///< Wi-Fi password

// MQTT Broker credentials
const char *mqtt_broker = "broker.emqx.io"; ///< MQTT broker address
const char *topic = "emqx/esp32"; ///< MQTT topic to subscribe/publish to
const char *mqtt_username = "emqx"; ///< MQTT username
const char *mqtt_password = "public"; ///< MQTT password
const int mqtt_port = 1883; ///< MQTT port (default for non-SSL connections)

WiFiClient espClient; ///< Wi-Fi client object for connecting to the network
PubSubClient client(espClient); ///< MQTT client using the Wi-Fi client for communication

/**
 * @brief Callback function that handles incoming messages from the MQTT broker.
 * 
 * This function is called when a new message is received on the subscribed MQTT topic.
 * It prints the message content and topic to the serial monitor for debugging.
 * 
 * @param topic The topic the message was received from.
 * @param payload The message payload (data).
 * @param length The length of the payload in bytes.
 */
void callback(char *topic, byte *payload, unsigned int length) {
    Serial.print("Message arrived in topic: ");
    Serial.println(topic);

    Serial.print("Message: ");
    for (int i = 0; i < length; i++) {
        Serial.print((char)payload[i]);  ///< Prints each byte of the payload as a character
    }
    Serial.println();
    Serial.println("-----------------------");
}

/**
 * @brief Initializes the ESP32, connects to Wi-Fi, and sets up MQTT broker connection.
 * 
 * This function connects the ESP32 to a Wi-Fi network and a public MQTT broker (EMQX),
 * and sets up a callback for receiving messages. Once connected, it publishes a greeting
 * message to the MQTT topic and subscribes to it for future messages.
 */

void setupMQTT() {
    // Set software serial baud to 115200 for debugging
    Serial.begin(115200);

    // Connecting to WiFi network
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.println("Connecting to WiFi..");
    }
    Serial.println("Connected to the Wi-Fi network");

    // Connecting to MQTT broker
    client.setServer(mqtt_broker, mqtt_port);  ///< Sets the MQTT broker address and port
    client.setCallback(callback); ///< Sets the callback function for handling incoming messages
    
    while (!client.connected()) {
        String client_id = "esp32-client-"; 
        client_id += String(WiFi.macAddress()); ///< Creates a unique client ID using the MAC address
        Serial.printf("The client %s connects to the public MQTT broker\n", client_id.c_str());

        // Attempt to connect to MQTT broker
        if (client.connect(client_id.c_str(), mqtt_username, mqtt_password)) {
            Serial.println("Public EMQX MQTT broker connected");
        } else {
            Serial.print("Failed with state ");
            Serial.print(client.state());
            delay(2000); ///< Wait before retrying connection
        }
    }

    // Publish a message to the topic and subscribe to it
    client.publish(topic, "Hi, the code you wrote wowrks ^^");
    client.subscribe(topic);
}



/**
 * @brief Main loop function that keeps the MQTT client alive and processes incoming messages.
 * 
 * This function continuously calls the MQTT client's loop function to ensure it can receive and process
 * any messages from the broker.
 */
void loopMGTT() {
    client.loop();  ///< Keeps the MQTT connection alive and checks for new messages
}
