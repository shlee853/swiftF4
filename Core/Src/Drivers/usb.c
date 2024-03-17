/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * usb.c
 */
#include <string.h>

/*ST includes */
#include "stm32f4xx.h"

/*FreeRtos includes*/
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "queuemonitor.h"

#include "config.h"
#include "usblink.h"
//#include "radiolink.h"
#include "usb.h"

#include "usbd_usr.h"
//#include "usb_conf.h"
#include "usbd_conf.h"
#include "usbd_desc.h"
//#include "usb_dcd.h"
//#include "usbd_req.h"
#include "usbd_ctlreq.h"
#include "usbd_cdc_if.h"

#include "crtp.h"
#include "static_mem.h"
//#include "vcp_esc_passthrough.h"
#include "bootloader.h"

//NO_DMA_CCM_SAFE_ZERO_INIT __ALIGN_BEGIN USB_OTG_CORE_HANDLE    USB_OTG_dev __ALIGN_END ;

static bool isInit = false;
static bool doingTransfer = false;
static bool doingVcpTransfer = false;
static bool rxStopped = true;
static uint16_t command = 0xFF;


// This should probably be reduced to a CRTP packet size
static xQueueHandle usbDataRx;
STATIC_MEM_QUEUE_ALLOC(usbDataRx, 5, sizeof(USBPacket)); /* Buffer USB packets (max 64 bytes) */
static xQueueHandle usbDataTx;
STATIC_MEM_QUEUE_ALLOC(usbDataTx, 1, sizeof(USBPacket)); /* Buffer USB packets (max 64 bytes) */

//#define USB_CDC_CONFIG_DESC_SIZ     98

#define CF_INTERFACE                0x0
#define VCP_COM_INTERFACE           0x1

/* Endpoints */
#define CF_IN_EP                    0x81  /* EP1 for data IN */
#define CF_OUT_EP                   0x01  /* EP1 for data OUT */

#define VCP_IN_EP                   0x82  /* EP2 for data IN */
#define VCP_OUT_EP                  0x02  /* EP2 for data OUT */
#define VCP_CMD_EP                  0x83  /* EP3 for command */

#define DEVICE_DESCRIPTOR           0x01
#define CONFIGURATION_DESCRIPTOR    0x02
#define STRING_DESCRIPTOR           0x03
#define INTERFACE_DESCRIPTOR        0x04
#define ENDPOINT_DESCRIPTOR         0x05

#define USB_CDC_IDLE                0
#define USB_CDC_BUSY                1
#define USB_CDC_ZLP                 2


extern USBD_HandleTypeDef hUsbDeviceFS;


/*********************************************
   CDC specific management functions
 *********************************************/
extern uint8_t USBD_DeviceDesc   [USB_LEN_DEV_DESC];

#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
  #if defined ( __ICCARM__ ) /*!< IAR Compiler */
    #pragma data_alignment=4
  #endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */
__ALIGN_BEGIN uint8_t usbd_cdc_CfgDesc  [USB_CDC_CONFIG_DESC_SIZ] __ALIGN_END ;

#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
  #if defined ( __ICCARM__ ) /*!< IAR Compiler */
    #pragma data_alignment=4
  #endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */
__ALIGN_BEGIN static __IO uint32_t  usbd_cdc_AltSet  __ALIGN_END = 0;

#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
  #if defined ( __ICCARM__ ) /*!< IAR Compiler */
    #pragma data_alignment=4
  #endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */
__ALIGN_BEGIN uint8_t USB_Rx_Buffer   [CDC_DATA_MAX_PACKET_SIZE] __ALIGN_END ;

#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
  #if defined ( __ICCARM__ ) /*!< IAR Compiler */
    #pragma data_alignment=4
  #endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */
__ALIGN_BEGIN uint8_t APP_Rx_Buffer   [APP_RX_DATA_SIZE] __ALIGN_END ;


#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
  #if defined ( __ICCARM__ ) /*!< IAR Compiler */
    #pragma data_alignment=4
  #endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */
__ALIGN_BEGIN uint8_t CmdBuff[CDC_CMD_PACKET_SIZE] __ALIGN_END ;

uint8_t  USB_Tx_State = USB_CDC_IDLE;

static uint32_t cdcCmd = 0xFF;
static uint32_t cdcLen = 0;

__ALIGN_BEGIN uint8_t  usbd_cf_CfgDesc[USB_CDC_CONFIG_DESC_SIZ] __ALIGN_END = {
  /***** Configuration descriptor ******/
  9,                         //bLength
  CONFIGURATION_DESCRIPTOR,  //bDescriptorType
  USB_CDC_CONFIG_DESC_SIZ,   //wTotalLength:no of returned bytes
  0x00,
  3,                         //bNumInterfaces:  3 interfaces (1 for CF, 2 for CDC)
  1,                         //bConfigurationValue
  0,                         //iConfiguration
  0x80,                      //bmAttribute (Bus powered, no remote wakeup)
  50,                        //bMaxPower (100mA, shall be enough)


  /***** Interface 0 descriptor: Crazyflie EPs ******/
  9,                         //bLength
  INTERFACE_DESCRIPTOR,      //bDescriptorType
  CF_INTERFACE,              //bInterfaceNumber
  0,                         //bAlternateSetting
  2,                         //bNumEndpoint (one in, one out)
  0xFF,                      //bInterfaceClass (VENDOR=0xFF)
  0xFF,                      //bInterfaceSubClass (VENDOR=0xFF)
  0,                         //bInterfaceProtocol (None)
  0,                         //iInterface
  /***** Endpoint 1 IN descriptor ******/
  7,                         //bLength
  ENDPOINT_DESCRIPTOR,       //bDescriptorType
  CF_IN_EP,                 //bEndpointAddess (EP1 IN)
  0x02,                      //bmAttributes (Bulk endpoint)
  0x40, 0x00,                //wMaxPacketSize (64 bytes)
  6,                         //bInterval (irrelevant for bulk endpoint)
  /***** Endpoint 1 OUT descriptor ******/
  7,                         //bLength
  ENDPOINT_DESCRIPTOR,       //bDescriptorType
  CF_OUT_EP,                //bEndpointAddess (EP1 OUT)
  0x02,                      //bmAttributes (Bulk endpoint)
  0x40, 0x00,                //wMaxPacketSize (64 bytes)
  6,                         //bInterval (irrelevant for bulk endpoint)


    /*---------------------------------------------------------------------------*/
    /////////////////////////////////////////////////////
    //Add 1 IAD class here // this one is for COM port
    0x08, // bLength: Interface Descriptor size
    0x0B, // bDescriptorType: IAD
    0x01, // bFirstInterface
    0x02, // bInterfaceCount
    0x02, // bFunctionClass: CDC
    0x02, // bFunctionSubClass
    0x01, // bFunctionProtocol
    0x02, // iFunction

    /*Interface Descriptor */
    0x09,   /* bLength: Interface Descriptor size */
    USB_DESC_TYPE_INTERFACE,  /* bDescriptorType: Interface */
    /* Interface descriptor type */
    VCP_COM_INTERFACE,   /* bInterfaceNumber: Number of Interface */
    0x00,   /* bAlternateSetting: Alternate setting */
    0x01,   /* bNumEndpoints: One endpoints used */
    0x02,   /* bInterfaceClass: Communication Interface Class */
    0x02,   /* bInterfaceSubClass: Abstract Control Model */
    0x01,   /* bInterfaceProtocol: Common AT commands */
    0x01,   /* iInterface: */

    /*Header Functional Descriptor*/
    0x05,   /* bLength: Endpoint Descriptor size */
    0x24,   /* bDescriptorType: CS_INTERFACE */
    0x00,   /* bDescriptorSubtype: Header Func Desc */
    0x10,   /* bcdCDC: spec release number */
    0x01,

    /*Call Management Functional Descriptor*/
    0x05,   /* bFunctionLength */
    0x24,   /* bDescriptorType: CS_INTERFACE */
    0x01,   /* bDescriptorSubtype: Call Management Func Desc */
    0x00,   /* bmCapabilities: D0+D1 */
    0x02,   /* bDataInterface: 2 */

    /*ACM Functional Descriptor*/
    0x04,   /* bFunctionLength */
    0x24,   /* bDescriptorType: CS_INTERFACE */
    0x02,   /* bDescriptorSubtype: Abstract Control Management desc */
    0x00,   /* bmCapabilities */

    /*Union Functional Descriptor*/
    0x05,   /* bFunctionLength */
    0x24,   /* bDescriptorType: CS_INTERFACE */
    0x06,   /* bDescriptorSubtype: Union func desc */
    0x01,   /* bMasterInterface: Communication class interface */
    0x02,   /* bSlaveInterface0: Data Class Interface */

    /*Endpoint 2 Descriptor*/
    0x07,                           /* bLength: Endpoint Descriptor size */
    USB_DESC_TYPE_ENDPOINT,   /* bDescriptorType: Endpoint */
    VCP_CMD_EP,                     /* bEndpointAddress */
    0x03,                           /* bmAttributes: Interrupt */
    LOBYTE(CDC_CMD_PACKET_SIZE),     /* wMaxPacketSize: */
    HIBYTE(CDC_CMD_PACKET_SIZE),
#ifdef USE_USB_OTG_HS
    0x10,                           /* bInterval: */
#else
    0xFF,                           /* bInterval: */
#endif /* USE_USB_OTG_HS */
    /*---------------------------------------------------------------------------*/

    /*Data class interface descriptor*/
    0x09,   /* bLength: Endpoint Descriptor size */
    USB_DESC_TYPE_INTERFACE,  /* bDescriptorType: */
    VCP_COM_INTERFACE + 0x01, /* bInterfaceNumber: Number of Interface */
    0x00,   /* bAlternateSetting: Alternate setting */
    0x02,   /* bNumEndpoints: Two endpoints used */
    0x0A,   /* bInterfaceClass: CDC */
    0x00,   /* bInterfaceSubClass: */
    0x00,   /* bInterfaceProtocol: */
    0x00,   /* iInterface: */

    /*Endpoint OUT Descriptor*/
    0x07,   /* bLength: Endpoint Descriptor size */
    USB_DESC_TYPE_ENDPOINT,      /* bDescriptorType: Endpoint */
    VCP_OUT_EP,                        /* bEndpointAddress */
    0x02,                              /* bmAttributes: Bulk */
    LOBYTE(CDC_DATA_MAX_PACKET_SIZE),  /* wMaxPacketSize: */
    HIBYTE(CDC_DATA_MAX_PACKET_SIZE),
    0x00,                              /* bInterval: ignore for Bulk transfer */

    /*Endpoint IN Descriptor*/
    0x07,   /* bLength: Endpoint Descriptor size */
    USB_DESC_TYPE_ENDPOINT,      /* bDescriptorType: Endpoint */
    VCP_IN_EP,                         /* bEndpointAddress */
    0x02,                              /* bmAttributes: Bulk */
    LOBYTE(CDC_DATA_MAX_PACKET_SIZE),  /* wMaxPacketSize: */
    HIBYTE(CDC_DATA_MAX_PACKET_SIZE),
    0x00                               /* bInterval: ignore for Bulk transfer */
};

/*
static uint8_t  usbd_cf_Init        (void  *pdev, uint8_t cfgidx);
static uint8_t  usbd_cf_DeInit      (void  *pdev, uint8_t cfgidx);
static uint8_t  usbd_cf_DataIn      (void *pdev, uint8_t epnum);
static uint8_t  usbd_cf_DataOut     (void *pdev, uint8_t epnum);
static uint8_t  *usbd_cf_GetCfgDesc (uint8_t speed, uint16_t *length);
static uint8_t  usbd_cf_SOF         (void *pdev);
static uint8_t  usbd_cf_Setup       (void *pdev , USB_SETUP_REQ  *req);
static uint8_t  usbd_cdc_EP0_RxReady(void *pdev);
*/
static USBPacket inPacket;
static USBPacket outPacket;
static USBPacket outVcpPacket;

/* CDC interface class callbacks structure */
/*
USBD_Class_cb_TypeDef cf_usb_cb =
{
  usbd_cf_Init,
  usbd_cf_DeInit,
  usbd_cf_Setup,
  NULL,
  usbd_cdc_EP0_RxReady,
  usbd_cf_DataIn,
  usbd_cf_DataOut,
  usbd_cf_SOF,
  NULL,
  NULL,
  usbd_cf_GetCfgDesc,
};

USBD_Usr_cb_TypeDef USR_cb =
{
  USBD_USR_Init,
  USBD_USR_DeviceReset,
  USBD_USR_DeviceConfigured,
  USBD_USR_DeviceSuspended,
  USBD_USR_DeviceResumed,
  USBD_USR_DeviceConnected,
  USBD_USR_DeviceDisconnected,
};
*/


bool usbGetDataBlocking(USBPacket *in)
{
  while (xQueueReceive(usbDataRx, in, portMAX_DELAY) != pdTRUE); // Don't return until we get some data on the USB

  // Disabling USB interrupt to make sure we can check and re-enable the endpoint
  // if it is not currently accepting data (ie. can happen if the RX queue was full)
  NVIC_DisableIRQ(OTG_FS_IRQn);
  if (rxStopped) {
	  USBD_LL_PrepareReceive(&hUsbDeviceFS, CF_OUT_EP, (uint8_t*)(inPacket.data), USB_RX_TX_PACKET_SIZE);
    rxStopped = false;
  }
  NVIC_EnableIRQ(OTG_FS_IRQn);

  return true;
}


static USBPacket outStage;

bool usbSendData(uint32_t size, uint8_t* data)
{
  outStage.size = size;
  memcpy(outStage.data, data, size);
  // Dont' block when sending
  return (xQueueSend(usbDataTx, &outStage, M2T(100)) == pdTRUE);
}


void usbInit(void)
{

//  MX_USB_DEVICE_Init();		// HAL 드라이버에서 생성됨
  usbDataRx = STATIC_MEM_QUEUE_CREATE(usbDataRx);
  DEBUG_QUEUE_MONITOR_REGISTER(usbDataRx);
  usbDataTx = STATIC_MEM_QUEUE_CREATE(usbDataTx);
  DEBUG_QUEUE_MONITOR_REGISTER(usbDataTx);

  isInit = true;
}

bool usbTest(void)
{
  return isInit;
}
