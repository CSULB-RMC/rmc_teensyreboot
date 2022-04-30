// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>
#include <usb.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("teensy_reset_node"), libusb_teensy_handle(NULL)
  {
    subscription_ = this->create_subscription<std_msgs::msg::Empty>(
      "teensy_reset", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const std_msgs::msg::Empty::SharedPtr msg)
  {
    teensy_open();
    soft_reboot();
    teensy_close();
    
  }
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr subscription_;
  usb_dev_handle *libusb_teensy_handle;
  usb_dev_handle * open_usb_device(int vid, int pid)
  {
    struct usb_bus *bus;
    struct usb_device *dev;
    usb_dev_handle *h;
    char buf[128];
    int r;

    usb_init();
    usb_find_busses();
    usb_find_devices();
    //printf_verbose("\nSearching for USB device:\n");
    for (bus = usb_get_busses(); bus; bus = bus->next) {
      for (dev = bus->devices; dev; dev = dev->next) {
        //printf_verbose("bus \"%s\", device \"%s\" vid=%04X, pid=%04X\n",
        //	bus->dirname, dev->filename,
        //	dev->descriptor.idVendor,
        //	dev->descriptor.idProduct
        //);
        if (dev->descriptor.idVendor != vid) continue;
        if (dev->descriptor.idProduct != pid) continue;
        h = usb_open(dev);
        if (!h) {
          RCLCPP_INFO(this->get_logger(), "Found device but unable to open\n");
          continue;
        }
        #ifdef LIBUSB_HAS_GET_DRIVER_NP
        r = usb_get_driver_np(h, 0, buf, sizeof(buf));
        if (r >= 0) {
          r = usb_detach_kernel_driver_np(h, 0);
          if (r < 0) {
            usb_close(h);
            RCLCPP_INFO(this->get_logger(), "Device is in use by \"%s\" driver\n", buf);
            continue;
          }
        }
        #endif
        // Mac OS-X - removing this call to usb_claim_interface() might allow
        // this to work, even though it is a clear misuse of the libusb API.
        // normally Apple's IOKit should be used on Mac OS-X
        #if !defined(MACOSX)
        r = usb_claim_interface(h, 0);
        if (r < 0) {
          usb_close(h);
          RCLCPP_INFO(this->get_logger(), "Unable to claim interface, check USB permissions\n");
          continue;
        }
        #endif

        return h;
      }
    }
    return NULL;
  }
  void teensy_close(void)
  {
    if (!libusb_teensy_handle) return;
    usb_release_interface(libusb_teensy_handle, 0);
    usb_close(libusb_teensy_handle);
    libusb_teensy_handle = NULL;
  }
  int teensy_open(void)
  {
    teensy_close();
    libusb_teensy_handle = open_usb_device(0x16C0, 0x0478);
    if (libusb_teensy_handle) return 1;
    return 0;
  }
  int soft_reboot(void)
  {
    usb_dev_handle *serial_handle = NULL;

    serial_handle = open_usb_device(0x16C0, 0x0483);
    if (!serial_handle) {
      char *error = usb_strerror();
      RCLCPP_INFO(this->get_logger(), "Error opening USB device: %s\n", error);
      return 0;
    }

    unsigned char reboot_command[] = {0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08};
    int response = usb_control_msg(serial_handle, 0x21, 0x20, 0, 0, (char *)reboot_command, sizeof reboot_command, 10000);

    usb_release_interface(serial_handle, 0);
    usb_close(serial_handle);

    if (response < 0) {
      char *error = usb_strerror();
      RCLCPP_INFO(this->get_logger(), "Unable to soft reboot with USB error: %s\n", error);
      return 0;
    }

    return 1;
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
