/******************************************************************************
 * The MIT License
 *
 * Copyright (c) 2011 LeafLabs LLC.
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy,
 * modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *****************************************************************************/

/**
 * @file libmaple/usb/stm32f1/usb_hid.c
 * @brief USB HID (human interface device) support
 *
 * FIXME: this works on the STM32F1 USB peripherals, and probably no
 * place else. Nonportable bits really need to be factored out, and
 * the result made cleaner.
 */

#include "usb_generic.h"
#include "gogo_usb_hid.h"
#include <string.h>

uint16 GetEPTxAddr(uint8 /*bEpNum*/);

/* usb_lib headers */
#include "usb_type.h"
#include "usb_core.h"
#include "usb_def.h"


#include <libmaple/gpio.h>
#include <board/board.h>

typedef enum _HID_REQUESTS
{
 
  GET_REPORT = 1,
  GET_IDLE,
  GET_PROTOCOL,
 
  SET_REPORT = 9,
  SET_IDLE,
  SET_PROTOCOL
 
} HID_REQUESTS;

static uint32 ProtocolValue = 0;
static uint32 txEPSize = 64;
static uint32 rxEPSize = 64;
static volatile int8 transmitting;
static struct usb_chunk* reportDescriptorChunks = NULL;

static void hidDataTxCb(void);
static void hidDataRxCb(void);
static void hidUSBReset(void);
static void usb_hid_clear(void);
static RESULT hidUSBDataSetup(uint8 request, uint8 interface, uint8 requestType, uint8 wValue0, uint8 wValue1, uint16 wIndex, uint16 wLength);
static RESULT hidUSBNoDataSetup(uint8 request, uint8 interface, uint8 requestType, uint8 wValue0, uint8 wValue1, uint16 wIndex);

static volatile GoGoHIDBuffer_t hidBuffers[MAX_HID_BUFFERS] = {{ 0 }};

#define HID_INTERFACE_OFFSET 	0x00
#define NUM_HID_ENDPOINTS          2
#define HID_INTERFACE_NUMBER (HID_INTERFACE_OFFSET+usbGoGoHIDPart.startInterface)

/*
 * Descriptors
 */
 

#define HID_ENDPOINT_TX      0
#define HID_ENDPOINT_RX      1
#define USB_HID_TX_ENDPOINT_INFO (&hidEndpoints[HID_ENDPOINT_TX])
#define USB_HID_RX_ENDPOINT_INFO (&hidEndpoints[HID_ENDPOINT_RX])
#define USB_HID_TX_ENDP (hidEndpoints[HID_ENDPOINT_TX].address)
#define USB_HID_RX_ENDP (hidEndpoints[HID_ENDPOINT_RX].address)
#define USB_HID_RX_PMA_PTR (hidEndpoints[HID_ENDPOINT_RX].pma)


typedef struct {
    //HID
    usb_descriptor_interface     	HID_Interface;
	GoGoHIDDescriptor			 	HID_Descriptor;
    usb_descriptor_endpoint      	HIDDataOutEndpoint;
    usb_descriptor_endpoint      	HIDDataInEndpoint;
} __packed hid_part_config;

static const hid_part_config hidPartConfigData = {
	.HID_Interface = {
		.bLength            = sizeof(usb_descriptor_interface),
        .bDescriptorType    = USB_DESCRIPTOR_TYPE_INTERFACE,
        .bInterfaceNumber   = HID_INTERFACE_OFFSET, // PATCH
        .bAlternateSetting  = 0x00,
        .bNumEndpoints      = NUM_HID_ENDPOINTS,    
        .bInterfaceClass    = USB_INTERFACE_CLASS_HID,
        .bInterfaceSubClass = USB_INTERFACE_SUBCLASS_HID,
        .bInterfaceProtocol = 0x00, /* Common AT Commands */
        .iInterface         = 0x00,
	},
	.HID_Descriptor = {
		.len				= 9,//sizeof(HIDDescDescriptor),
		.dtype				= HID_DESCRIPTOR_TYPE,
		.versionL			= 0x10,
		.versionH			= 0x01,
		.country			= 0x00,
		.numDesc			= 0x01,
		.desctype			= REPORT_DESCRIPTOR,//0x22,
		.descLenL			= 0x00, //PATCH
		.descLenH			= 0x00, //PATCH
	},
	.HIDDataOutEndpoint = {
		.bLength          = sizeof(usb_descriptor_endpoint),
        .bDescriptorType  = USB_DESCRIPTOR_TYPE_ENDPOINT,
        .bEndpointAddress = USB_DESCRIPTOR_ENDPOINT_OUT | 0, // PATCH: USB_HID_RX_ENDP
        .bmAttributes     = USB_EP_TYPE_INTERRUPT,
        .wMaxPacketSize   = 64, //PATCH
        .bInterval        = 0x0A,
	},
	.HIDDataInEndpoint = {
		.bLength          = sizeof(usb_descriptor_endpoint),
        .bDescriptorType  = USB_DESCRIPTOR_TYPE_ENDPOINT,
        .bEndpointAddress = USB_DESCRIPTOR_ENDPOINT_IN | 0, // PATCH: USB_HID_TX_ENDP
        .bmAttributes     = USB_EP_TYPE_INTERRUPT,
        .wMaxPacketSize   = 64, //PATCH
        .bInterval        = 0x0A,
	}
};

static ONE_DESCRIPTOR HID_Hid_Descriptor = {
    (uint8*)&hidPartConfigData.HID_Descriptor,
    sizeof(hidPartConfigData.HID_Descriptor)
};

static USBEndpointInfo hidEndpoints[NUM_HID_ENDPOINTS] = {
    {
        .callback = hidDataTxCb,
        .pmaSize = 64,
        .type = USB_GENERIC_ENDPOINT_TYPE_INTERRUPT,
        .tx = 1,
    },
    {
        .callback = hidDataRxCb,
        .pmaSize = 64,
        .type = USB_GENERIC_ENDPOINT_TYPE_INTERRUPT,
        .tx = 0,
    }
};

void gogo_usb_hid_setRXEPSize(uint32_t size) {
    if (size == 0 || size > 64)
        size = 64;
    hidEndpoints[HID_ENDPOINT_RX].pmaSize = size;
    rxEPSize = size;
}

void gogo_usb_hid_setTXEPSize(uint32_t size) {
    if (size == 0 || size > 64)
        size = 64;
    hidEndpoints[HID_ENDPOINT_TX].pmaSize = size;
    txEPSize = size;
}

#define OUT_BYTE(s,v) out[(uint8*)&(s.v)-(uint8*)&s]
#define OUT_16(s,v) *(uint16_t*)&OUT_BYTE(s,v) // OK on Cortex which can handle unaligned writes

static void getHIDPartDescriptor(uint8* out) {
    memcpy(out, &hidPartConfigData, sizeof(hid_part_config));
    // patch to reflect where the part goes in the descriptor
    OUT_BYTE(hidPartConfigData, HID_Interface.bInterfaceNumber) += usbGoGoHIDPart.startInterface;
    OUT_BYTE(hidPartConfigData, HIDDataOutEndpoint.bEndpointAddress) += USB_HID_RX_ENDP;
    OUT_BYTE(hidPartConfigData, HIDDataInEndpoint.bEndpointAddress) += USB_HID_TX_ENDP;
    uint16 size = usb_generic_chunks_length(reportDescriptorChunks);
    OUT_BYTE(hidPartConfigData, HID_Descriptor.descLenL) = (uint8)size;
    OUT_BYTE(hidPartConfigData, HID_Descriptor.descLenH) = (uint8)(size>>8);
    OUT_16(hidPartConfigData, HIDDataOutEndpoint.wMaxPacketSize) = rxEPSize;
    OUT_16(hidPartConfigData, HIDDataInEndpoint.wMaxPacketSize) = txEPSize;
}

USBCompositePart usbGoGoHIDPart = {
    .numInterfaces = 1,
    .numEndpoints = sizeof(hidEndpoints)/sizeof(*hidEndpoints),
    .descriptorSize = sizeof(hid_part_config),
    .getPartDescriptor = getHIDPartDescriptor,
    .usbInit = NULL,
    .usbReset = hidUSBReset,
    .usbDataSetup = hidUSBDataSetup,
    .usbNoDataSetup = hidUSBNoDataSetup,
    .usbClearFeature = NULL,
    .usbSetConfiguration = NULL,
    .clear = usb_hid_clear,
    .endpoints = hidEndpoints
};


#define HID_RX_BUFFER_SIZE	256 // must be power of 2
#define HID_RX_BUFFER_SIZE_MASK (HID_RX_BUFFER_SIZE-1)
// Rx data
static volatile uint8 hidBufferRx[HID_RX_BUFFER_SIZE];
// Write index to hidBufferRx
static volatile uint32 hid_rx_head = 0;
// Read index from hidBufferRx
static volatile uint32 hid_rx_tail = 0;

#define HID_TX_BUFFER_SIZE	256 // must be power of 2
#define HID_TX_BUFFER_SIZE_MASK (HID_TX_BUFFER_SIZE-1)
// Tx data
static volatile uint8 hidBufferTx[HID_TX_BUFFER_SIZE];
// Write index to hidBufferTx
static volatile uint32 hid_tx_head = 0;
// Read index from hidBufferTx
static volatile uint32 hid_tx_tail = 0;

void gogo_usb_hid_set_report_descriptor(struct usb_chunk* chunks) {
    reportDescriptorChunks = chunks;
}

    
static volatile GoGoHIDBuffer_t* usb_hid_find_buffer(uint8 type, uint8 reportID) {
    uint8 typeTest = type == HID_REPORT_TYPE_OUTPUT ? HID_BUFFER_MODE_OUTPUT : 0;
    for (int i=0; i<MAX_HID_BUFFERS; i++) {
        if ( hidBuffers[i].buffer != NULL &&
             ( hidBuffers[i].mode & HID_BUFFER_MODE_OUTPUT ) == typeTest && 
             hidBuffers[i].reportID == reportID) {
            return hidBuffers+i;
        }
    }
    return NULL;
}

void gogo_usb_hid_set_feature(uint8 reportID, uint8* data) {
    volatile GoGoHIDBuffer_t* buffer = usb_hid_find_buffer(HID_REPORT_TYPE_FEATURE, reportID);
    if (buffer != NULL) {
        usb_generic_pause_rx_ep0();
        unsigned delta = reportID != 0;
        memcpy((uint8*)buffer->buffer+delta, data, buffer->bufferSize-delta);
        if (reportID)
            buffer->buffer[0] = reportID;
        buffer->state = HID_BUFFER_READ;
        usb_generic_enable_rx_ep0();
        return;
    }
}

static uint8 have_unread_data_in_hid_buffer() {
    for (int i=0;i<MAX_HID_BUFFERS; i++) {
        if (hidBuffers[i].buffer != NULL && hidBuffers[i].state == HID_BUFFER_UNREAD)
            return 1;
    }
    return 0;
}

uint16_t gogo_usb_hid_get_data(uint8 type, uint8 reportID, uint8* out, uint8 poll) {
    volatile GoGoHIDBuffer_t* buffer;
    unsigned ret = 0;
    
    buffer = usb_hid_find_buffer(type, reportID);
    
    if (buffer == NULL)
        return 0;

    usb_generic_disable_interrupts_ep0();
    
    if (buffer->reportID == reportID && buffer->state != HID_BUFFER_EMPTY && !(poll && buffer->state == HID_BUFFER_READ)) {
        unsigned delta = reportID != 0;
        
        if (out != NULL)
            memcpy(out, (uint8*)buffer->buffer+delta, buffer->bufferSize-delta);
        
        if (poll) {
            buffer->state = HID_BUFFER_READ;
        }

        ret = buffer->bufferSize-delta;
    }
    
    if (! have_unread_data_in_hid_buffer() ) {
        usb_generic_enable_rx_ep0();
    }

    usb_generic_enable_interrupts_ep0();
            
    return ret;
}

void gogo_usb_hid_clear_buffers(uint8 type) {
    uint8 typeTest = type == HID_REPORT_TYPE_OUTPUT ? HID_BUFFER_MODE_OUTPUT : 0;
    for (int i=0; i<MAX_HID_BUFFERS; i++) {
        if (( hidBuffers[i].mode & HID_BUFFER_MODE_OUTPUT ) == typeTest) {
            hidBuffers[i].buffer = NULL;
        }
    }
}

static void usb_hid_clear(void) {
    ProtocolValue = 0;
    gogo_usb_hid_clear_buffers(HID_REPORT_TYPE_OUTPUT);
    gogo_usb_hid_clear_buffers(HID_REPORT_TYPE_FEATURE);
}

uint8 gogo_usb_hid_add_buffer(uint8 type, volatile GoGoHIDBuffer_t* buf) {
    if (type == HID_BUFFER_MODE_OUTPUT) 
        buf->mode |= HID_BUFFER_MODE_OUTPUT;
    else
        buf->mode &= ~HID_BUFFER_MODE_OUTPUT;
    memset((void*)buf->buffer, 0, buf->bufferSize);
    buf->buffer[0] = buf->reportID;

    volatile GoGoHIDBuffer_t* buffer = usb_hid_find_buffer(type, buf->reportID);

    if (buffer != NULL) {
        *buffer = *buf;
        return 1;
    }
    else {
        for (int i=0; i<MAX_HID_BUFFERS; i++) {
            if (hidBuffers[i].buffer == NULL) {
                hidBuffers[i] = *buf;
                return 1;
            }
        }
        return 0;
    }
}

void gogo_usb_hid_set_buffers(uint8 type, volatile GoGoHIDBuffer_t* bufs, int n) {
    uint8 typeMask = type == HID_REPORT_TYPE_OUTPUT ? HID_BUFFER_MODE_OUTPUT : 0;
    gogo_usb_hid_clear_buffers(type);
    for (int i=0; i<n; i++) {
        bufs[i].mode &= ~HID_REPORT_TYPE_OUTPUT;
        bufs[i].mode |= typeMask;
        gogo_usb_hid_add_buffer(type, bufs+i);
    }
}


/* Nonblocking byte receive.
 *
 * Copies up to len bytes from our private data buffer (*NOT* the PMA)
 * into buf and deq's the FIFO. */
uint32 gogo_usb_hid_rx(uint8* buf, uint32 len) {
    /* Copy bytes to buffer. */
    uint32 n_copied = gogo_usb_hid_peek(buf, len);

    /* Mark bytes as read. */
	uint16 tail = hid_rx_tail; // load volatile variable
	tail = (tail + n_copied) & HID_RX_BUFFER_SIZE_MASK;
	hid_rx_tail = tail; // store volatile variable

    uint32 rx_unread = (hid_rx_head - tail) & HID_RX_BUFFER_SIZE_MASK;
    /* If all bytes have been read, re-enable the RX endpoint, which
     * was set to NAK when the current batch of bytes was received. */
    if (rx_unread <= rxEPSize) {
        // usb_set_ep_rx_count(USB_HID_RX_ENDP, rxEPSize);
        usb_generic_enable_rx(USB_HID_RX_ENDPOINT_INFO);
    }
    return n_copied;
}

/* Nonblocking byte lookahead.
 *
 * Looks at unread bytes without marking them as read. */
uint32 gogo_usb_hid_peek(uint8* buf, uint32 len) {
    unsigned i;
    uint32 tail = hid_rx_tail;
	uint32 rx_unread = (hid_rx_head-tail) & HID_RX_BUFFER_SIZE_MASK;

    if (len > rx_unread) {
        len = rx_unread;
    }

    for (i = 0; i < len; i++) {
        buf[i] = hidBufferRx[tail];
        tail = (tail + 1) & HID_RX_BUFFER_SIZE_MASK;
    }

    return len;
}


uint32 gogo_usb_hid_data_available(void) {
    return (hid_rx_head - hid_rx_tail) & HID_RX_BUFFER_SIZE_MASK;
}

static void hidDataRxCb(void) {
    uint32 head = hid_rx_head;
    usb_generic_read_to_circular_buffer(USB_HID_RX_ENDPOINT_INFO,
                            hidBufferRx, HID_RX_BUFFER_SIZE, &head);
	hid_rx_head = head; // store volatile variable

	uint32 rx_unread = (head - hid_rx_tail) & HID_RX_BUFFER_SIZE_MASK;
	// only enable further Rx if there is enough room to receive one more packet
	if ( rx_unread < (HID_RX_BUFFER_SIZE-rxEPSize) ) {
        usb_generic_enable_rx(USB_HID_RX_ENDPOINT_INFO);
	}
}


/* This function is non-blocking.
 *
 * It copies data from a user buffer into the USB peripheral TX
 * buffer, and returns the number of bytes copied. */
uint32 gogo_usb_hid_tx(const uint8* buf, uint32 len)
{
	if (len==0) return 0; // no data to send

	uint32 head = hid_tx_head; // load volatile variable
	uint32 tx_unsent = (head - hid_tx_tail) & HID_TX_BUFFER_SIZE_MASK;

    // We can only put bytes in the buffer if there is place
    if (len > (HID_TX_BUFFER_SIZE-tx_unsent-1) ) {
        len = (HID_TX_BUFFER_SIZE-tx_unsent-1);
    }
	if (len==0) return 0; // buffer full

	uint16 i;
	// copy data from user buffer to USB Tx buffer
	for (i=0; i<len; i++) {
		hidBufferTx[head] = buf[i];
		head = (head+1) & HID_TX_BUFFER_SIZE_MASK;
	}
	hid_tx_head = head; // store volatile variable

	while(transmitting >= 0);
	
	if (transmitting<0) {
		hidDataTxCb(); // initiate data transmission
	}

    return len;
}



uint32 gogo_usb_hid_get_pending(void) {
    return (hid_tx_head - hid_tx_tail) & HID_TX_BUFFER_SIZE_MASK;
}


static void hidDataTxCb(void)
{
    usb_generic_send_from_circular_buffer(USB_HID_TX_ENDPOINT_INFO, 
        hidBufferTx, HID_TX_BUFFER_SIZE, hid_tx_head, &hid_tx_tail, &transmitting);
}


static void hidUSBReset(void) {
    /* Reset the RX/TX state */
    hid_rx_head = 0;
    hid_rx_tail = 0;
	hid_tx_head = 0;
	hid_tx_tail = 0;
    transmitting = -1;
}

#pragma GCC diagnostic ignored "-Wunused-parameter"
static RESULT hidUSBDataSetup(uint8 request, uint8 interface, uint8 requestType, uint8 wValue0, uint8 wValue1, uint16 wIndex, uint16 wLength) {
    (void)interface; // only one interface

    if ((requestType & (REQUEST_TYPE | RECIPIENT)) == (CLASS_REQUEST | INTERFACE_RECIPIENT)) {
    switch (request) {
        case SET_REPORT:
			if (wValue1 == HID_REPORT_TYPE_FEATURE || wValue1 == HID_REPORT_TYPE_OUTPUT) {
                volatile GoGoHIDBuffer_t* buffer = usb_hid_find_buffer(wValue1, wValue0);
				
				if (buffer == NULL) {
					return USB_UNSUPPORT;
				}
				
				if (0 == (buffer->mode & HID_BUFFER_MODE_NO_WAIT) && buffer->state == HID_BUFFER_UNREAD) {
					return USB_NOT_READY;
				} 
				else 
				{
//                    buffer->state = HID_BUFFER_EMPTY;
                    usb_generic_control_rx_setup(buffer->buffer, buffer->bufferSize, &(buffer->state));
                    buffer->state = HID_BUFFER_UNREAD;
				}
                return USB_SUCCESS;
			}
            break;
        case GET_REPORT:
            if (wValue1 == HID_REPORT_TYPE_FEATURE) {
				volatile GoGoHIDBuffer_t* buffer = usb_hid_find_buffer(HID_REPORT_TYPE_FEATURE, wValue0);
				
				if (buffer == NULL || buffer->state == HID_BUFFER_EMPTY) {
					return USB_UNSUPPORT; // TODO: maybe UNREADY on empty
				}

                usb_generic_control_tx_setup(buffer->buffer, buffer->bufferSize, NULL);
                return USB_SUCCESS;
			}
        default:
            break;
        }
    }

  if((requestType & (REQUEST_TYPE | RECIPIENT)) == (STANDARD_REQUEST | INTERFACE_RECIPIENT)){
    	switch (request){
    		case GET_DESCRIPTOR:
				if (wValue1 == REPORT_DESCRIPTOR) {
                    usb_generic_control_tx_chunk_setup(reportDescriptorChunks);
                    return USB_SUCCESS;
                } 		
				else if (wValue1 == HID_DESCRIPTOR_TYPE){
                    usb_generic_control_descriptor_tx(&HID_Hid_Descriptor);
                    return USB_SUCCESS;
				} 		
			
    			break;
    		case GET_PROTOCOL:
                usb_generic_control_tx_setup(&ProtocolValue, 1, NULL);
                return USB_SUCCESS;
		}
	}

    return USB_UNSUPPORT;
}


static RESULT hidUSBNoDataSetup(uint8 request, uint8 interface, uint8 requestType, uint8 wValue0, uint8 wValue1, uint16 wIndex) {
    (void)interface; // only one interface
    
	if ((requestType & (REQUEST_TYPE | RECIPIENT)) == (CLASS_REQUEST | INTERFACE_RECIPIENT)) {
        switch(request) {
            case SET_PROTOCOL:
                ProtocolValue = wValue0;
                return USB_SUCCESS;
        }
    }
    return USB_UNSUPPORT;
}

