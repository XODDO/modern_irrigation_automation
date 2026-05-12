#ifndef CREDENTIALS_H
#define CREDENTIALS_H

// Structure for WiFi credentials
struct WiFiCred {
  const char* SECRET_SSID;
  const char* SECRET_PASS;
};

// List of known WiFi networks
const WiFiCred knownNetworks[] = {
  {"MUARIK_DRYER_ONLINE", "MUARIK@2026"},
  {"IntelliSys Cloud", "IntelliSys@2025!"},
  {"IntelliSys Air", "intel_cool@2025!"},
  {"irrikit_Cloud", "IntelliSys@2025!"},
  {"IntelliSys Online", "qwerty0976_"},
  {"IntelliSys Pro 2025", "intel_cool@2025"},
  {"iPaul", "longpassword"},
  {"irrikit_Cloud", "IntelliSys@2025!"}
};
const int knownCount = sizeof(knownNetworks) / sizeof(knownNetworks[0]);

struct OTA {
    const char OTA_PASSWORD[7] = "12!34";
};


class OTA_Credentials {
public: 
    const char secure_ota_password[7] = "1A2B";

};



#endif
