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
#include "radiolink.h"
#include "usb.h"

#include "usbd_usr.h"
#include "usbd_conf.h"
#include "usbd_desc.h"
#include "usbd_ctlreq.h"
#include "usbd_cdc_if.h"
#include "usbd_cdc.h"

#include "crtp.h"
#include "static_mem.h"
#include "vcp_esc_passthrough.h"
#include "bootloader.h"


static bool isInit = false;
static bool doingTransfer = false;
static bool doingVcpTransfer = false;
static bool rxStopped = true;
static uint16_t command = 0xFF;

static uint32_t cdcCmd = 0xFF;
static uint32_t cdcLen = 0;

// This should probably be reduced to a CRTP packet size
static xQueueHandle usbDataRx;
STATIC_MEM_QUEUE_ALLOC(usbDataRx, 5, sizeof(USBPacket)); /* Buffer USB packets (max 64 bytes) */
static xQueueHandle usbDataTx;
STATIC_MEM_QUEUE_ALLOC(usbDataTx, 1, sizeof(USBPacket)); /* Buffer USB packets (max 64 bytes) */

#define USB_CDC_CONFIG_DESC_SIZE     98
#define USBD_CFG_MAX_NUM                1
#define USBD_ITF_MAX_NUM                1

#define USB_OTG_EP_CONTROL                       0
#define USB_OTG_EP_ISOC                          1
#define USB_OTG_EP_BULK                          2
#define USB_OTG_EP_INT                           3
#define USB_OTG_EP_MASK                          3

#define CF_INTERFACE                0x0
#define VCP_COM_INTERFACE           0x1

/* Endpoints */
#define CF_IN_EP                    0x81  // 0x81  /* EP1 for data IN */
#define CF_OUT_EP                   0x01  // 0x01  /* EP1 for data OUT */

#define VCP_IN_EP                   0x82  // 0x82  /* EP2 for data IN */
#define VCP_OUT_EP                  0x02  // 0x02  /* EP2 for data OUT */
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
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;

//NO_DMA_CCM_SAFE_ZERO_INIT __ALIGN_BEGIN USBD_HandleTypeDef    hUsbDeviceFS __ALIGN_END ;


uint8_t  USB_Tx_State = USB_CDC_IDLE;


/*********************************************
   CDC specific management functions
 *********************************************/
//extern uint8_t USBD_DeviceDesc   [USB_LEN_DEV_DESC];

//extern uint8_t USBD_FS_DeviceDesc[]

#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
  #if defined ( __ICCARM__ ) /*!< IAR Compiler */
    #pragma data_alignment=4
  #endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */
__ALIGN_BEGIN uint8_t usbd_cdc_CfgDesc  [USB_CDC_CONFIG_DESC_SIZE] __ALIGN_END ;

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

__ALIGN_BEGIN uint8_t  usbd_cf_CfgDesc[USB_CDC_CONFIG_DESC_SIZE] __ALIGN_END = {
  /***** Configuration descriptor ******/
  9,                         //bLength
  CONFIGURATION_DESCRIPTOR,  //bDescriptorType
  USB_CDC_CONFIG_DESC_SIZE,   //wTotalLength:no of returned bytes
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

static uint8_t  usbd_cf_Init        (USBD_HandleTypeDef  *pdev, uint8_t cfgidx);
static uint8_t  usbd_cf_DeInit      (USBD_HandleTypeDef  *pdev, uint8_t cfgidx);
static uint8_t  usbd_cf_DataIn      (USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t  usbd_cf_DataOut     (USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t  *usbd_cf_GetCfgDesc (uint16_t *length);
static uint8_t  usbd_cf_SOF         (USBD_HandleTypeDef *pdev);
static uint8_t  usbd_cf_Setup       (USBD_HandleTypeDef *pdev , USBD_SetupReqTypedef  *req);
static uint8_t  usbd_cdc_EP0_RxReady(USBD_HandleTypeDef *pdev);

static USBPacket inPacket;
static USBPacket outPacket;
static USBPacket outVcpPacket;

/* CDC interface class callbacks structure */






USBD_ClassTypeDef cf_usb_cb =
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
  NULL,
  usbd_cf_GetCfgDesc,
  NULL,
  NULL,
};






typedef struct _USBD_USR_PROP
{
  void (*Init)(void);
  void (*DeviceReset)(uint8_t speed);
  void (*DeviceConfigured)(void);
  void (*DeviceSuspended)(void);
  void (*DeviceResumed)(void);

  void (*DeviceConnected)(void);
  void (*DeviceDisconnected)(void);

}
USBD_Usr_cb_TypeDef;


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




static void resetUSB(void) {
  portBASE_TYPE xTaskWokenByReceive = pdFALSE;

  crtpSetLink(radiolinkGetLink());

  if (isInit == true) {
    // Empty queue
    while (xQueueReceiveFromISR(usbDataTx, &outPacket, &xTaskWokenByReceive) == pdTRUE);
  }

  //  USB_OTG_FlushTxFifo(&USB_OTG_dev, CF_IN_EP);
  USBD_LL_FlushEP(&hUsbDeviceFS, CF_IN_EP);

  rxStopped = true;
  doingTransfer = false;
}






static uint8_t usbd_cf_Setup(USBD_HandleTypeDef *pdev , USBD_SetupReqTypedef  *req)
{

  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];
  uint16_t len;
  uint8_t ifalt = 0U;
  uint16_t status_info = 0U;
  USBD_StatusTypeDef ret = USBD_OK;


  if ((req->bmRequest & USB_REQ_TYPE_MASK) == USB_REQ_TYPE_VENDOR) // Crazyflie interface
  {


	  // 중요항목: 없으면 USB timeout error 발생
 //    ((USBD_CDC_ItfTypeDef *)pdev->pUserData[pdev->classId])->Control(req->bRequest, (uint8_t *)hcdc->data, req->wLength);

       len = MIN(CDC_REQ_MAX_DATA_SIZE, req->wLength);
       (void)USBD_CtlSendData(pdev, (uint8_t *)hcdc->data, len);

    command = req->wIndex;



    if (command == 0x01)
    {
      crtpSetLink(usblinkGetLink());

      //      if (rxStopped && !xQueueIsQueueFullFromISR(usbDataRx))
      if ( rxStopped && !xQueueIsQueueFullFromISR(usbDataRx))
      {

//        DCD_EP_PrepareRx(&hUsbDeviceFS, CF_OUT_EP, (uint8_t*)(inPacket.data), USB_RX_TX_PACKET_SIZE);

   	    inPacket.size =  USBD_LL_GetRxDataSize(pdev, CF_OUT_EP);
        USBD_LL_PrepareReceive(&hUsbDeviceFS, CF_OUT_EP, (uint8_t*)(inPacket.data), USB_RX_TX_PACKET_SIZE);

//   	    USBD_LL_Transmit(pdev, CF_IN_EP, (uint8_t*)inPacket.data, inPacket.size );


        rxStopped = false;
      }
    }
    else if(command == 0x02)
    {
      //restart system and transition to DFU bootloader mode
      //enter bootloader specific to STM32f4xx
      enter_bootloader(0, 0x00000000);
    }
    else
    {
      crtpSetLink(radiolinkGetLink());
//      usbd_cf_DataOut (pdev, CF_OUT_EP);


    }
  }
  else // VCP_COM_INTERFACE
  {
    switch (req->bmRequest & USB_REQ_TYPE_MASK)
    {
      /* CDC Class Requests -------------------------------*/
    case USB_REQ_TYPE_CLASS :
        /* Check if the request is a data setup packet */
        if (req->wLength)
        {
          /* Check if the request is Device-to-Host */
          if (req->bmRequest & 0x80)
          {
            /* Get the data to be sent to Host from interface layer */
            //APP_FOPS.pIf_Ctrl(req->bRequest, CmdBuff, req->wLength);
            ((USBD_CDC_ItfTypeDef *)pdev->pUserData[pdev->classId])->Control(req->bRequest, (uint8_t *)hcdc->data, req->wLength);

            /* Send the data to the host */
//            USBD_CtlSendData (pdev,  CmdBuff, req->wLength);
            len = MIN(CDC_REQ_MAX_DATA_SIZE, req->wLength);
            (void)USBD_CtlSendData(pdev, (uint8_t *)hcdc->data, len);


          }
          else /* Host-to-Device request */
          {
            /* Set the value of the current command to be processed */
            cdcCmd = req->bRequest;
            cdcLen = req->wLength;

            /* Prepare the reception of the buffer over EP0
            Next step: the received data will be managed in usbd_cdc_EP0_TxSent()
            function. */
    //        USBD_CtlPrepareRx (pdev, CmdBuff, req->wLength);

            hcdc->CmdOpCode = req->bRequest;
            hcdc->CmdLength = (uint8_t)MIN(req->wLength, USB_MAX_EP0_SIZE);

            (void)USBD_CtlPrepareRx(pdev, (uint8_t *)hcdc->data, hcdc->CmdLength);


          }
        }
        else /* No Data request */
        {
          ((USBD_CDC_ItfTypeDef *)pdev->pUserData[pdev->classId])->Control(req->bRequest, (uint8_t *)req, 0U);
          /* See the command as activating passthrough interface*/
          passthroughEnableFromISR();
        }

        return USBD_OK;

      default:
        USBD_CtlError (pdev, req);
        return USBD_FAIL;

      /* Standard Requests -------------------------------*/
    case USB_REQ_TYPE_STANDARD:
      switch (req->bRequest)
      {
      case USB_REQ_GET_DESCRIPTOR:
        USBD_CtlError (pdev, req);
        return USBD_FAIL;

      case USB_REQ_GET_INTERFACE :
        USBD_CtlSendData (pdev,
                          (uint8_t *)&usbd_cdc_AltSet,
                          1);
        break;

      case USB_REQ_SET_INTERFACE :
        if ((uint8_t)(req->wValue) < USBD_ITF_MAX_NUM)
        {
          usbd_cdc_AltSet = (uint8_t)(req->wValue);
        }
        else
        {
          /* Call the error management function (command will be nacked */
          USBD_CtlError (pdev, req);
        }
        break;
      }
    }
  }

  return USBD_OK;
}

static uint8_t  usbd_cf_Init (USBD_HandleTypeDef *pdev,  uint8_t cfgidx)
{
  UNUSED(cfgidx);
  USBD_CDC_HandleTypeDef *hcdc;

  hcdc = (USBD_CDC_HandleTypeDef *)USBD_malloc(sizeof(USBD_CDC_HandleTypeDef));

  if (hcdc == NULL)
  {
    pdev->pClassDataCmsit[pdev->classId] = NULL;
    return (uint8_t)USBD_EMEM;
  }

  (void)USBD_memset(hcdc, 0, sizeof(USBD_CDC_HandleTypeDef));

  pdev->pClassDataCmsit[pdev->classId] = (void *)hcdc;
  pdev->pClassData = pdev->pClassDataCmsit[pdev->classId];


  /* Open CF EP IN */
  USBD_LL_OpenEP(pdev, CF_IN_EP, USB_OTG_EP_BULK, USB_RX_TX_PACKET_SIZE);
  pdev->ep_in[CF_IN_EP & 0xFU].is_used = 1U;
  /* Open CF EP OUT */
  USBD_LL_OpenEP(pdev, CF_OUT_EP, USB_OTG_EP_BULK, USB_RX_TX_PACKET_SIZE);
  pdev->ep_out[CF_OUT_EP & 0xFU].is_used = 1U;


  /* Open EP IN for VCP*/
  USBD_LL_OpenEP(pdev, VCP_IN_EP, USB_OTG_EP_BULK, CDC_DATA_IN_PACKET_SIZE);
  pdev->ep_in[VCP_IN_EP & 0xFU].is_used = 1U;

  /* Open EP OUT for VCP*/
  USBD_LL_OpenEP(pdev, VCP_OUT_EP, USB_OTG_EP_BULK, CDC_DATA_OUT_PACKET_SIZE);
  pdev->ep_out[VCP_OUT_EP & 0xFU].is_used = 1U;

  /* Open Command IN EP for VCP*/
  pdev->ep_in[VCP_CMD_EP & 0xFU].bInterval = CDC_FS_BINTERVAL;
  USBD_LL_OpenEP(pdev, VCP_CMD_EP, USB_OTG_EP_INT, CDC_CMD_PACKET_SIZE);
  pdev->ep_in[VCP_CMD_EP & 0xFU].is_used = 1U;

  hcdc->RxBuffer = NULL;

  ((USBD_CDC_ItfTypeDef *)pdev->pUserData[pdev->classId])->Init();

  hcdc->TxState = 0U;
  hcdc->RxState = 0U;

  if (hcdc->RxBuffer == NULL)
  {
    return (uint8_t)USBD_EMEM;
  }

  /* Prepare Out endpoint to receive next packet */
  USBD_LL_PrepareReceive(pdev, CF_OUT_EP, (uint8_t*)(inPacket.data), USB_RX_TX_PACKET_SIZE);
  //  USBD_LL_PrepareReceive(pdev, CF_OUT_EP, hcdc->RxBuffer, USB_RX_TX_PACKET_SIZE);

  /* Prepare Out endpoint to receive next packet */
  USBD_LL_PrepareReceive(pdev, VCP_OUT_EP, (uint8_t*)USB_Rx_Buffer, CDC_DATA_OUT_PACKET_SIZE);

  rxStopped = false;

  return USBD_OK;
}

/**
  * @brief  usbd_cdc_Init
  *         DeInitialize the CDC layer
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t  usbd_cf_DeInit (USBD_HandleTypeDef  *pdev, uint8_t cfgidx)
{

    UNUSED(cfgidx);
  /* Close CF EP IN */
	USBD_LL_CloseEP(pdev, CF_IN_EP);
    pdev->ep_in[CF_IN_EP & 0xFU].is_used = 0U;

  /* Close CF EP OUT */
	USBD_LL_CloseEP(pdev, CF_OUT_EP);
    pdev->ep_out[CF_OUT_EP & 0xFU].is_used = 0U;

  /* Close VCP EP IN */
	USBD_LL_CloseEP(pdev, VCP_IN_EP);
	pdev->ep_in[VCP_IN_EP & 0xFU].is_used = 0U;

  /* Close VCP EP OUT */
    USBD_LL_CloseEP(pdev, VCP_OUT_EP);
    pdev->ep_out[VCP_OUT_EP & 0xFU].is_used = 0U;

  /* Close Command IN EP */
    USBD_LL_CloseEP(pdev, VCP_CMD_EP);
	pdev->ep_in[VCP_CMD_EP & 0xFU].is_used = 0U;
    pdev->ep_in[VCP_CMD_EP & 0xFU].bInterval = 0U;


    if (pdev->pClassDataCmsit[pdev->classId] != NULL)
    {
      ((USBD_CDC_ItfTypeDef *)pdev->pUserData[pdev->classId])->DeInit();
      (void)USBD_free(pdev->pClassDataCmsit[pdev->classId]);
      pdev->pClassDataCmsit[pdev->classId] = NULL;
      pdev->pClassData = NULL;
    }

  return USBD_OK;
}

/**
  * @brief  usbd_cdc_EP0_RxReady
  *         Data received on control endpoint
  * @param  pdev: device instance
  * @retval status
  */
uint8_t  usbd_cdc_EP0_RxReady (USBD_HandleTypeDef  *pdev)
{
	  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];

	  if (hcdc == NULL)
	  {
	    return (uint8_t)USBD_FAIL;
	  }

	  if ((pdev->pUserData[pdev->classId] != NULL) && (hcdc->CmdOpCode != 0xFFU))
	  {
	    ((USBD_CDC_ItfTypeDef *)pdev->pUserData[pdev->classId])->Control(hcdc->CmdOpCode,
	                                                                     (uint8_t *)hcdc->data,
	                                                                     (uint16_t)hcdc->CmdLength);
	    hcdc->CmdOpCode = 0xFFU;
	  }

	  return (uint8_t)USBD_OK;
}

/**
  * @brief  usbd_audio_DataIn
  *         Data sent on non-control IN endpoint
  * @param  pdev: device instance
  * @param  epnum: endpoint number
  * @retval status
  */
static uint8_t  usbd_cf_DataIn (USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  if (epnum == (CF_IN_EP &~ 0x80))
  {
    portBASE_TYPE xTaskWokenByReceive = pdFALSE;

    doingTransfer = false;

//    outPacket.size = inPacket.size;
  	//  USBD_LL_Transmit(pdev, CF_IN_EP, (uint8_t*)inPacket.data, inPacket.size );

    if (xQueueReceiveFromISR(usbDataTx, &outPacket, &xTaskWokenByReceive) == pdTRUE)
    {
      doingTransfer = true;
      USBD_LL_Transmit(pdev, CF_IN_EP, (uint8_t*)outPacket.data,  outPacket.size );

    }
    // Test
//    USBD_LL_Transmit(pdev, CF_IN_EP, (uint8_t*)inPacket.data,  inPacket.size );

  portYIELD_FROM_ISR(xTaskWokenByReceive);
  }
  else // VCP In endpoint
  {
    int i;
    doingVcpTransfer = false;

    for (i = 0; i < USB_RX_TX_PACKET_SIZE && (passthroughVcpTxReceiveFromISR(&outVcpPacket.data[i]) == pdTRUE); i++);

    if (i != 0)
    {
      doingVcpTransfer = true;

      USBD_LL_Transmit(pdev, VCP_IN_EP,(uint8_t*)outVcpPacket.data,  i );

    }
  }

  return USBD_OK;
}

static uint8_t  usbd_cf_SOF (USBD_HandleTypeDef *pdev)
{
  portBASE_TYPE xTaskWokenByReceive = pdFALSE;
  if (!doingTransfer) {
    if (xQueueReceiveFromISR(usbDataTx, &outPacket, &xTaskWokenByReceive) == pdTRUE)
    {
      doingTransfer = true;
  	  USBD_LL_FlushEP(pdev, CF_IN_EP);
      USBD_LL_Transmit(pdev, CF_IN_EP,(uint8_t*)outPacket.data,  outPacket.size);

    }
  }
  portYIELD_FROM_ISR(xTaskWokenByReceive);

  /* VCP */
  if (!doingVcpTransfer) {
   int i;
   for (i = 0; (i < USB_RX_TX_PACKET_SIZE) &&
         (passthroughVcpTxReceiveFromISR(&outVcpPacket.data[i]) == pdTRUE); i++)
   { };

    if (i != 0)
    {
      doingVcpTransfer = true;
	  USBD_LL_PrepareReceive(pdev, VCP_IN_EP, (uint8_t*)outVcpPacket.data, i);
    }
  }

  return USBD_OK;
}

/**
  * @brief  usbd_cf_DataOut
  *         Data received on non-control Out endpoint
  * @param  pdev: device instance
  * @param  epnum: endpoint number
  * @retval status
  */
static uint8_t  usbd_cf_DataOut (USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  uint8_t result;
  if (epnum == CF_OUT_EP)
  {
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

//    inPacket.size = ((USBD_HandleTypeDef*)pdev)->ep_out[epnum].total_length;
    inPacket.size =  USBD_LL_GetRxDataSize(pdev, epnum);

    if (xQueueSendFromISR(usbDataRx, &inPacket, &xHigherPriorityTaskWoken) == pdTRUE) {
      result = USBD_OK;
    } else {
      result = USBD_BUSY;
    }

    if (!xQueueIsQueueFullFromISR(usbDataRx)) {
      /* Prepare Out endpoint to receive next packet */
  	  USBD_LL_PrepareReceive(pdev, CF_OUT_EP, (uint8_t*)inPacket.data, USB_RX_TX_PACKET_SIZE);	// EP0 0x01로 들어온 데이터를 packet.data에 넣음

  	  // ??? 아래 BYPASS 코드가 없으면 READ시 인터럽트가 걸리지 않음
//  	  USBD_LL_FlushEP(pdev, CF_IN_EP);
//   	  USBD_LL_Transmit(pdev, CF_IN_EP, (uint8_t*)inPacket.data, 1);

      rxStopped = false;
    } else {
      rxStopped = true;
    }
  }
  else // VCP
  {
	  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];

	  if (pdev->pClassDataCmsit[pdev->classId] == NULL)
	  {
	    return (uint8_t)USBD_FAIL;
	  }

	  /* Get the received data length */
	  hcdc->RxLength = USBD_LL_GetRxDataSize(pdev, epnum);

    /* USB data will be immediately processed, this allow next USB traffic being
    NAKed till the end of the application Xfer */
    for (int i = 0; i < hcdc->RxLength; i++)
    {
      passthroughVcpRxSendFromISR(USB_Rx_Buffer[i]);
    }

    /* Prepare Out endpoint to receive next packet */
	USBD_LL_PrepareReceive(pdev, VCP_OUT_EP, (uint8_t*)USB_Rx_Buffer, CDC_DATA_OUT_PACKET_SIZE);

	USBD_LL_Transmit(pdev, VCP_IN_EP, (uint8_t*)USB_Rx_Buffer, hcdc->RxLength );
//	HAL_PCD_EP_Transmit(&hpcd_USB_OTG_FS, VCP_IN_EP, (uint8_t*)USB_Rx_Buffer, hcdc->RxLength );


    result = USBD_OK;
  }

  return result;
}

/**
  * @brief  USBD_cdc_GetCfgDesc
  *         Return configuration descriptor
  * @param  speed : current device speed
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t  *usbd_cf_GetCfgDesc (uint16_t *length)
{
//  *length = sizeof (usbd_cf_CfgDesc);
//  return usbd_cf_CfgDesc;


	  USBD_EpDescTypeDef *pcfEpOutDesc = USBD_GetEpDesc(usbd_cf_CfgDesc, CF_OUT_EP);
	  USBD_EpDescTypeDef *pcfEpInDesc = USBD_GetEpDesc(usbd_cf_CfgDesc, CF_IN_EP);
	  USBD_EpDescTypeDef *pvcpEpCmdDesc = USBD_GetEpDesc(usbd_cf_CfgDesc, VCP_CMD_EP);
	  USBD_EpDescTypeDef *pvcpEpOutDesc = USBD_GetEpDesc(usbd_cf_CfgDesc, VCP_OUT_EP);
	  USBD_EpDescTypeDef *pvcpEpInDesc = USBD_GetEpDesc(usbd_cf_CfgDesc, VCP_IN_EP);

	  if (pvcpEpCmdDesc != NULL)
	  {
		  pvcpEpCmdDesc->bInterval = CDC_FS_BINTERVAL;
	  }

	  if (pvcpEpOutDesc != NULL)
	  {
		  pvcpEpOutDesc->wMaxPacketSize = CDC_DATA_FS_MAX_PACKET_SIZE;
	  }

	  if (pvcpEpInDesc != NULL)
	  {
		  pvcpEpInDesc->wMaxPacketSize = CDC_DATA_FS_MAX_PACKET_SIZE;
	  }


	  if (pcfEpOutDesc != NULL)
	  {
		  pcfEpOutDesc->wMaxPacketSize = CDC_DATA_FS_MAX_PACKET_SIZE;
	  }

	  if (pcfEpInDesc != NULL)
	  {
		  pcfEpInDesc->wMaxPacketSize = CDC_DATA_FS_MAX_PACKET_SIZE;
	  }


	  *length = (uint16_t)sizeof(usbd_cf_CfgDesc);
	  return usbd_cf_CfgDesc;



}

/**
* @brief  USBD_USR_Init
*         Displays the message on LCD for host lib initialization
* @param  None
* @retval None
*/
void USBD_USR_Init(void)
{

}

/**
* @brief  USBD_USR_DeviceReset
* @param  speed : device speed
* @retval None
*/
void USBD_USR_DeviceReset(uint8_t speed)
{
  resetUSB();
}


/**
* @brief  USBD_USR_DeviceConfigured
* @param  None
* @retval Staus
*/
void USBD_USR_DeviceConfigured(void)
{
}

/**
* @brief  USBD_USR_DeviceSuspended
* @param  None
* @retval None
*/
void USBD_USR_DeviceSuspended(void)
{
  /* USB communication suspended (probably USB unplugged). Switch back to radiolink */
  resetUSB();
}


/**
* @brief  USBD_USR_DeviceResumed
* @param  None
* @retval None
*/
void USBD_USR_DeviceResumed(void)
{
}


/**
* @brief  USBD_USR_DeviceConnected
* @param  None
* @retval Staus
*/
void USBD_USR_DeviceConnected(void)
{
}


/**
* @brief  USBD_USR_DeviceDisonnected
* @param  None
* @retval Staus
*/
void USBD_USR_DeviceDisconnected(void)
{
  resetUSB();
}















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
  //  return (xQueueSend(usbDataTx, &outStage, M2T(100)) == pdTRUE);
    if(xQueueSend(usbDataTx, &outStage, M2T(100))) {
        usbd_cf_DataIn(&hUsbDeviceFS, CF_OUT_EP);
    	return pdTRUE;
    }
    else {
    	return pdFALSE;
    }


}


void usbInit(void)
{

//  MX_USB_DEVICE_Init();		// HAL 드라이버에서 생성됨

//  USBD_Init(&USB_OTG_dev,USB_OTG_FS_CORE_ID, &USR_desc, &cf_usb_cb, &USR_cb);

  if (USBD_Init(&hUsbDeviceFS, &FS_Desc, DEVICE_FS) != USBD_OK)
  {
    Error_Handler();
  }

  if (USBD_RegisterClass(&hUsbDeviceFS, &cf_usb_cb) != USBD_OK)
//  if (USBD_RegisterClass(&hUsbDeviceFS, &USBD_CDC) != USBD_OK)
  {
    Error_Handler();
  }
  if (USBD_CDC_RegisterInterface(&hUsbDeviceFS, &USBD_Interface_fops_FS) != USBD_OK)
  {
    Error_Handler();
  }
  if (USBD_Start(&hUsbDeviceFS) != USBD_OK)
  {
    Error_Handler();
  }



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
