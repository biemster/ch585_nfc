# ch585_nfc
CH585 NFC demos (PCD and PICC) using ch32fun

## TODO
A couple things need to be taken care of for ch585 support in ch32fun before this works properly
- [ ] Not all clock settings work, for example the RTC does not work at 78MHz main freq. This needs investigating and fixing
- [ ] Probably a proper .high_code section should be added for critical functions, running from flash is slow
- [ ] Debug printf seems to be hanging on something
- [ ] USBFS should be tested, USBHS needs to be implemented
- [ ] A BLE advertisement with the detected tag would be nice, but iSLER is still unstable on ch585

In parallel the NFC blob needs reversing, starting with
- [ ] Determine which NFC/RFID tags are supported by the blob
- [ ] Implement a ch32fun demo with the blob that can do all the protocols supported by it
- [x] Reverse above demo to do the protocol init and APDU's and stuff without the blob
- [ ] Determine if support for extra protocols is viable
