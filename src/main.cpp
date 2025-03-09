// Signal K application template file.
//
// This application demonstrates core SensESP concepts in a very
// concise manner. You can build and upload the application as is
// and observe the value changes on the serial port monitor.
//
// You can use this source file as a basis for your own projects.
// Remove the parts that are not relevant to you, and add your own code
// for external hardware libraries.

#include <memory>
#include <iostream>
#include <sstream>
#include <string>

#include "sensesp.h"
#include "sensesp/sensors/analog_input.h"
#include "sensesp/sensors/digital_input.h"
#include "sensesp/sensors/digital_output.h"
#include "sensesp/sensors/sensor.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp/system/lambda_consumer.h"
#include "sensesp_app_builder.h"
#include "sensesp/transforms/linear.h"
#include "sensesp/signalk/signalk_value_listener.h"
#include "sensesp/signalk/signalk_listener.h"
#include "sensesp/system/rgb_led.h"
#include "sensesp/controllers/smart_switch_controller.h"
#include "sensesp/transforms/click_type.h"
#include "sensesp/transforms/debounce.h"
#include "sensesp/transforms/press_repeater.h"
#include "sensesp/transforms/repeat.h"
#include "sensesp/signalk/signalk_put_request_listener.h"

#define LED_ON_COLOR 0x004700
#define LED_OFF_COLOR 0x261900

#define STDBUTTON_PIN D10
#define STD_LED_R_PIN A1
#define STD_LED_G_PIN A2
#define STD_LED_B_PIN A3
#define PWRBUTTON_PIN D3
#define PWR_LED_R_PIN D1
#define PWR_LED_G_PIN D2
#define PWR_LED_B_PIN D3
#define POTI_PIN A0
#define BUZZER_PIN D4

#define PIN_RELAY D5
// PIN D5 is free



using namespace sensesp;

// The setup function performs one-time application initialization.
void setup() {
  SetupLogging(ESP_LOG_DEBUG);

  // Construct the global SensESPApp() object
  SensESPAppBuilder builder;
  sensesp_app = (&builder)
                    // Set a custom hostname for the app.
                    ->set_hostname("cockpit-1")
                    // Optionally, hard-code the WiFi and Signal K server
                    // settings. This is normally not needed.
                    //->set_wifi_client("My WiFi SSID", "my_wifi_password")
                    //->set_wifi_access_point("My AP SSID", "my_ap_password")
                    //->set_sk_server("192.168.10.3", 80)
                    ->get_app();


  ////////////////POTI////////////////
  //analogSetAttenuation(ADC_11db);
  const char* poti_path = "environment.inside.engineRoom.throttle";
  const char* poti_sk_path = "/engine/throttle/analog_in";
  const char* linear_config_path = "/engine/throttle/linear";
  unsigned int read_delay = 5000;

  auto input_calibration = new Linear(0.00137, 1023, linear_config_path);
  auto* poti_input = new AnalogInput(POTI_PIN, read_delay, poti_sk_path);

  ConfigItem(poti_input)
      ->set_title("Throttle input")
      ->set_description("Throttle read interval")
      ->set_sort_order(100);

  ConfigItem(input_calibration)
      ->set_title("Throttle calibration")
      ->set_description("Throttle calibration")
      ->set_sort_order(101);

  poti_input->connect_to(input_calibration);
  input_calibration->connect_to(new SKOutputFloat(poti_path, poti_sk_path, new SKMetadata("ratio", "Throttle Level")));



  ////////////////LEDs////////////////
  // STD BUTTON LED
  // Define the SK Path of the device that controls the load that this
  // switch should control remotely.  Clicking the virtual switch will
  // send a PUT request on this path to the main device. This virtual
  // switch will also subscribe to this path, and will set its state
  // internally to match any value reported on this path.
  // To find valid Signal K Paths that fits your need you look at this link:
  // https://signalk.org/specification/1.4.0/doc/vesselsBranch.html
  const char* sk_path = "electrical.switches.lights.engineroom.state";

  // "Configuration paths" are combined with "/config" to formulate a URL
  // used by the RESTful API for retrieving or setting configuration data.
  // It is ALSO used to specify a path to the SPIFFS file system
  // where configuration data is saved on the MCU board.  It should
  // ALWAYS start with a forward slash if specified.  If left blank,
  // that indicates a sensor or transform does not have any
  // configuration to save.
  const char* config_path_button_c = "/button/clicktime";
  const char* config_path_status_light = "/button/statusLight";
  const char* config_path_sk_output = "/signalk/path";
  const char* config_path_repeat = "/signalk/repeat";

  // An led that represents the current state of the switch.
  auto* led = new RgbLed(STD_LED_R_PIN, STD_LED_G_PIN, STD_LED_B_PIN,
                         config_path_status_light, LED_ON_COLOR, LED_OFF_COLOR);

  // Create a switch controller to handle the user press logic and
  // connect it a server PUT request...
  SmartSwitchController* controller = new SmartSwitchController(false);
  controller->connect_to(new BoolSKPutRequest(sk_path));

  // Also connect the controller to an onboard LED...
  controller->connect_to(led->on_off_consumer_);

  // Connect a physical button that will feed manual click types into the
  // controller...
  DigitalInputState* btn = new DigitalInputState(STDBUTTON_PIN, INPUT, 100);
  PressRepeater* pr = new PressRepeater();
  btn->connect_to(pr);

  auto click_type = new ClickType(config_path_button_c);

  ConfigItem(click_type)->set_title("Click Type")->set_sort_order(1000);

  pr->connect_to(click_type)->connect_to(controller->click_consumer_);

  // In addition to the manual button "click types", a
  // SmartSwitchController accepts explicit state settings via
  // any boolean producer or various "truth" values in human readable
  // format via a String producer.
  // Since this device is NOT the device that directly controls the
  // load, we don't want to respond to PUT requests. Instead, we
  // let the "real" switch respond to the PUT requests, and
  // here we just listen to the published values that are
  // sent across the Signal K network when the controlling device
  // confirms it has made the change in state.
  auto* sk_listener = new SKValueListener<bool>(sk_path);
  sk_listener->connect_to(controller->swich_consumer_);

  // To avoid garbage collecting all shared pointers created in setup(),
  // loop from here.
  while (true) {
    loop();
  }
}

void loop() { event_loop()->tick(); }
