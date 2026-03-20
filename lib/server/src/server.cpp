#include <WiFi.h>

// Replace with your network credentials
const char* ssid     = "ESP32-Access-Point";
const char* password = "123456789";
int setTemperature = 70;
String mode = "OFF";

// Set web server port number to 80
WiFiServer server(80);

// Function to generate HTML page
String getHTML(int setTemperature, String mode, int currentTemp) {
  // Default colors
  String offColor = "#555555";   // dark grey (default ON)
  String acColor = "#cccccc";    // default inactive
  String heatColor = "#cccccc";  // default inactive

  // Apply mode-based styling
  if (mode == "AC") {
    acColor = "#87CEFA";   // light blue
    offColor = "#cccccc";  // light grey
  }
  else if (mode == "HEAT") {
    heatColor = "#FF7F7F"; // light red
    offColor = "#cccccc";  // light grey
  }
  else if (mode == "OFF") {
    offColor = "#555555";  // dark grey
    acColor = "#cccccc";
    heatColor = "#cccccc";
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
  html += "<div style='display:flex; flex-direction:column; align-items:center; margin-right:20px;'>";

  html += "<div style='font-size:20px; margin-bottom:5px; text-align:center;'>Set Temperature</div>";

  html += "<input type='number' name='temp' value='";
  html += String(setTemperature);
  html += "' style='width:150px; height:150px; text-align:center; font-size:48px; border:3px solid black;'>";

  html += "</div>";

  // RIGHT SIDE: +/- buttons
  html += "<div style='display:flex; flex-direction:column; justify-content:space-between; height:150px;'>";
  html += "<a href='/plus'><button type='button' style='width:60px; height:70px; font-size:24px;'>+</button></a>";
  html += "<a href='/minus'><button type='button' style='width:60px; height:70px; font-size:24px;'>-</button></a>";
  html += "</div>";

  html += "</div>";

  // Spacer
  html += "<div style='height:40px;'></div>";

  // ===== MODE BUTTONS =====
  html += "<div style='display:flex; justify-content:center; gap:20px;'>";

  html += "<a href='/ac'><button style='padding:15px 25px; font-size:18px; background-color:";
  html += acColor;
  html += ";'>AC</button></a>";

  html += "<a href='/off'><button style='padding:15px 25px; font-size:18px; background-color:";
  html += offColor;
  html += "; color:white;'>OFF</button></a>";

  html += "<a href='/heat'><button style='padding:15px 25px; font-size:18px; background-color:";
  html += heatColor;
  html += ";'>HEAT</button></a>";

  html += "</div>";

  html += "</body></html>";

  return html;
}

void generateServer() {
  Serial.begin(115200);
  
  // Connect to Wi-Fi network with SSID and password
  Serial.print("Setting AP (Access Point)…");
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
    }   
    else if (request.indexOf("GET /plus") >= 0) {
        setTemperature++;
    }
    else if (request.indexOf("GET /minus") >= 0) {
        setTemperature--;
    }
    else if (request.indexOf("GET /ac") >= 0) {
        mode = "AC";
    }
    else if (request.indexOf("GET /heat") >= 0) {
        mode = "HEAT";
    }
    else if (request.indexOf("GET /off") >= 0) {
        mode = "OFF";
    }

    // Send HTTP response
    client.println("HTTP/1.1 200 OK");
    client.println("Content-type:text/html");
    client.println("Connection: close");
    client.println();

    client.println(getHTML(setTemperature, mode, currentTemp));
    client.println();

    delay(1);
    client.stop();
    Serial.println("Client disconnected");

    // Close the connection
    client.stop();
    Serial.println("Client disconnected.");
    Serial.println("");
  }
}