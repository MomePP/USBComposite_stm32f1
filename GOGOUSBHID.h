/* Copyright (c) 2011, Peter Barrett  
**  
** Permission to use, copy, modify, and/or distribute this software for  
** any purpose with or without fee is hereby granted, provided that the  
** above copyright notice and this permission notice appear in all copies.  
** 
** THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL  
** WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED  
** WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR  
** BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES  
** OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS,  
** WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION,  
** ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS  
** SOFTWARE.  
*/

#ifndef _GOGOUSBHID_H_
#define _GOGOUSBHID_H_

#include <Print.h>
#include <boards.h>
#include "Stream.h"
#include "gogo_usb_hid.h"
#include "USBHID.h"

class GoGoHIDReporter;

class GOGOUSBHID {
private:
    bool autoRegister = true;
	bool enabledHID = false;
    uint32 rxPacketSize = 64;
    uint32 txPacketSize = 64;
    struct usb_chunk* chunkList;
    // baseChunk holds any explicitly specified report descriptor that
    // overrides any report descriptors from the chain of registered profiles
    struct usb_chunk baseChunk = { 0, 0, 0 };
    GoGoHIDReporter* profiles;
public:
	static bool init(GOGOUSBHID* me);
    // add a report to the list ; if always is false, then it only works if autoRegister is true
    void addReport(GoGoHIDReporter* r, bool always=true);
    void clear();
	bool registerComponent();
	void setReportDescriptor(const uint8_t* report_descriptor, uint16_t report_descriptor_length);
	void setReportDescriptor(const HIDReportDescriptor* reportDescriptor=NULL);
    // All the strings are zero-terminated ASCII strings. Use NULL for defaults.
    void begin(const uint8_t* report_descriptor, uint16_t length);
    void begin(const HIDReportDescriptor* reportDescriptor = NULL);
    void begin(USBCompositeSerial serial, const uint8_t* report_descriptor, uint16_t length);
    void begin(USBCompositeSerial serial, const HIDReportDescriptor* reportDescriptor = NULL);
    void setBuffers(uint8_t buffers, volatile HIDBuffer_t* fb=NULL, int count=0); // type = HID_REPORT_TYPE_FEATURE or HID_REPORT_TYPE_OUTPUT
    bool addBuffer(uint8_t type, volatile HIDBuffer_t* buffer);
	void clearBuffers(uint8_t type);
	void clearBuffers();
    inline bool addFeatureBuffer(volatile HIDBuffer_t* buffer) {
        return addBuffer(HID_REPORT_TYPE_FEATURE, buffer);
    }
    inline bool addOutputBuffer(volatile HIDBuffer_t* buffer) {
        return addBuffer(HID_REPORT_TYPE_OUTPUT, buffer);
    }
    inline void setFeatureBuffers(volatile HIDBuffer_t* fb=NULL, int count=0) {
        setBuffers(HID_REPORT_TYPE_FEATURE, fb, count);
    }        
    inline void setOutputBuffers(volatile HIDBuffer_t* fb=NULL, int count=0) {
        setBuffers(HID_REPORT_TYPE_OUTPUT, fb, count);
    }     
    void end(void);
    void setRXPacketSize(uint32 size=64) {
        rxPacketSize = size;
    }
    void setTXPacketSize(uint32 size=64) {
        txPacketSize = size;
    }
    GOGOUSBHID(bool _autoRegister=true) {
        autoRegister = _autoRegister;
    }
};

class GoGoHIDReporter {
    private:
        uint8_t* reportBuffer;
        uint8_t reportID;
        uint8_t userSuppliedReportID;
        bool forceUserSuppliedReportID;
        uint16_t bufferSize;
        HIDReportDescriptor reportDescriptor;
        struct usb_chunk reportChunks[3];
        class GoGoHIDReporter* next;
        friend class GOGOUSBHID;

    protected:
        GOGOUSBHID& HID;
        
    public:
        void sendReport(); 
        // if you use this init function, the buffer starts with a reportID, even if the reportID is zero,
        // and bufferSize includes the reportID; if reportID is zero, sendReport() will skip the initial
        // reportID byte
        GoGoHIDReporter(GOGOUSBHID& _HID, const HIDReportDescriptor* r, uint8_t* _buffer, unsigned _size, uint8_t _reportID, bool forceReportID=false);
        // if you use this init function, the buffer has no reportID byte in it
        GoGoHIDReporter(GOGOUSBHID& _HID, const HIDReportDescriptor* r, uint8_t* _buffer, unsigned _size);
        uint16_t getFeature(uint8_t* out=NULL, uint8_t poll=1);
        uint16_t getOutput(uint8_t* out=NULL, uint8_t poll=1);
        uint16_t getData(uint8_t type, uint8_t* out, uint8_t poll=1); // type = HID_REPORT_TYPE_FEATURE or HID_REPORT_TYPE_OUTPUT
        void setFeature(uint8_t* feature);
        void registerProfile(bool always=true);
        uint32 available(void);
        uint32 readByte(void);
        uint32 readBytes(void *buf, uint32_t len);
};

#endif
        		