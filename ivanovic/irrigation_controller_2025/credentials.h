#ifndef CREDENTIALS_H
#define CREDENTIALS_H

// Structure for WiFi credentials
struct WiFiCred {
  const char* SECRET_SSID;
  const char* SECRET_PASS;
};

// List of known WiFi networks
const WiFiCred knownNetworks[] = {
  {"IntelliSys Air", "intel_cool@2025!"}, 
  {"irrikit_Cloud", "IntelliSys@2025!"},
  {"IntelliSys Online", "qwerty0976_"},
  {"IntelliSys Pro 2025", "intel_cool@2025"},
  {"iPaul", "longpassword"}
};
const int knownCount = sizeof(knownNetworks) / sizeof(knownNetworks[0]);

#endif
