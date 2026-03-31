#include <WiFi.h>

// Replace with your network credentials
const char* ssid     = "ESP32-Access-Point";
const char* password = "123456789";
int setTemperature = 70;
bool currentMode = false; // false = OFF, true = ON

// Set web server port number to 80
WiFiServer server(80);

// Function to generate HTML page
String getHTML(int setTemperature, bool mode, int currentTemp) {
  // Default colors
  String offColor = "#555555";   // default ON
  String onColor = "#cccccc";    // default inactive

  if (mode == false) {
    offColor = "#555555";  // dark grey
    onColor = "#cccccc";  // light grey
  }
  else if (mode == true) {
    onColor = "#555555";  // dark grey
    offColor = "#cccccc";  // light grey
  }

  String html = "<!DOCTYPE html><html>";
  html += "<head>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<title>Thermostat</title>";
  html += "</head>";

  html += "<body style='margin:0; background-color:#f0f0f0; font-family:Arial; display:flex; flex-direction:column; justify-content:center; align-items:center; height:100vh;'>";
  // ===== CURRENT TEMP TITLE =====
  html += "<div style='font-size:20px; margin-bottom:5px;'>Current Temperature</div>";

  // ===== CURRENT TEMP BOX (TOP) =====
  html += "<div style='width:150px; height:150px; border:3px solid black; display:flex; align-items:center; justify-content:center; font-size:48px; margin-bottom:20px; background-color:white;'>";
  html += String(currentTemp);
  html += "</div>";

  // ===== FORM =====
  html += "<form action='/setTemp' method='GET' style='display:flex; flex-direction:column; align-items:center;'>";

  // Row container
  html += "<div style='display:flex; align-items:center; justify-content:center;'>";

  // LEFT SIDE: title + input (centered together)
  html += "<div style='display:flex; flex-direction:column; align-items:center; margin-right:0px;'>";

  html += "<div style='font-size:20px; margin-bottom:5px; text-align:center;'>Set Temperature</div>";

  html += "<input type='number' name='temp' value='";
  html += String(setTemperature);
  html += "' style='width:150px; height:150px; text-align:center; font-size:48px; border:3px solid black;'>";

  html += "</div>";

  html += "</div>";

  // Spacer
  html += "<div style='height:40px;'></div>";

  // ===== MODE BUTTONS =====

  html += "<div style='display:flex; justify-content:center; gap:20px;'>";

  html += "<a href='/on'><button type = 'button' style='padding:15px 25px; font-size:18px; background-color:";
  html += onColor;
  html += "; color:white;'>ON</button></a>";

  html += "<a href='/off'><button type = 'button' style='padding:15px 25px; font-size:18px; background-color:";
  html += offColor;
  html += "; color:white;'>OFF</button></a>";

  html += "</div>";

  html += "</body></html>";

  return html;
}

void generateServer() {
  Serial.begin(115200);
  
  // Connect to Wi-Fi network with SSID and password
  Serial.println("Setting AP (Access Point)…");
  // Remove the password parameter, if you want the AP (Access Point) to be open
  WiFi.softAP(ssid, password);

  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);
  
  server.begin();
}

void manageServer(int currentTemp){
  WiFiClient client = server.available();
  if (client) {
    Serial.println("New Client");

    String request = "";
    unsigned long timeout = millis();

    while (client.connected() && millis() - timeout < 2000) {
      if (client.available()) {
        char c = client.read();
        request += c;

        // End of HTTP request (blank line)
        if (c == '\n' && request.endsWith("\r\n\r\n")) {
          break;
        }
      }
    }

    // Check request
    if (request.indexOf("GET /setTemp?temp=") >= 0) {
        int start = request.indexOf("temp=") + 5;
        int end = request.indexOf(" ", start);
        String tempStr = request.substring(start, end);
        setTemperature = tempStr.toInt();
        Serial.println("Set Temperature updated to: " + String(setTemperature));
    }   
    if (request.indexOf("GET /on") >= 0) {
      Serial.println("Mode set to ON");
      currentMode = true;
    }
    if (request.indexOf("GET /off") >= 0) {
      Serial.println("Mode set to OFF");
      currentMode = false;
    }

    // Send HTTP response
    client.println("HTTP/1.1 200 OK");
    client.println("Content-type:text/html");
    client.println("Connection: close");
    client.println();

    client.println(getHTML(setTemperature, currentMode, currentTemp));
    client.println();

    delay(1);
    client.stop();
    Serial.println("Client disconnected");
  }
}

bool setControls() {
  // This function can be expanded to include more complex logic for controlling the AC and heating based on the current temperature, set temperature, and mode.
  // For now, it simply returns true if the mode is ON and false if the mode is OFF.
  return currentMode;
}