#define LAB3 1

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "usb_device_config.h"
#include "usb.h"
#include "usb_device_stack_interface.h"
#include "usb_class_hid.h"
#include "mouse.h"
#include "usb_descriptor.h"

#include "fsl_i2c_master_driver.h"
#include "fxos8700.h"

#if (OS_ADAPTER_ACTIVE_OS == OS_ADAPTER_BM)
#include "user_config.h"
#endif

#if (OS_ADAPTER_ACTIVE_OS == OS_ADAPTER_SDK)
#include "fsl_device_registers.h"
#include "fsl_clock_manager.h"
#include "board.h"
#include "fsl_debug_console.h"

#include <stdio.h>
#include <stdlib.h>

#endif


/*****************************************************************************
 * Constant and Macro's - None
 *****************************************************************************/

/*****************************************************************************
 * Global Functions Prototypes
 *****************************************************************************/


/****************************************************************************
 * Global Variables
 ****************************************************************************/
/* Add all the variables needed for mouse.c to this structure */
extern usb_desc_request_notify_struct_t g_desc_callback;
static mouse_global_variable_struct_t g_mouse;

/*****************************************************************************
 * Local Types - None
 *****************************************************************************/
enum { RIGHT, DOWN, LEFT, UP, NONE };
/*****************************************************************************
 * Local Functions Prototypes
 *****************************************************************************/
void USB_App_Device_Callback(uint8_t event_type, void* val, void* arg);
uint8_t USB_App_Class_Callback(uint8_t request, uint16_t value, uint8_t ** data,
    uint32_t* size, void* arg);
/*****************************************************************************
 * Local Variables 
 *****************************************************************************/
int move_direction;

/*****************************************************************************
 * Local Functions
 *****************************************************************************/

/*****************************************************************************
 * 
 *      @name     move_mouse
 *
 *      @brief    This function gets makes the cursor on screen move left,right
 *                up and down
 *
 *      @param    None      
 *
 *      @return   None
 *     
 * 
 ******************************************************************************/
static void move_mouse(void)
{
    static int8_t x = 0, y = 0;

    static uint8_t dir = (uint8_t) RIGHT;

#if !LAB3
    /* Lab 1 and 2: Normal USB move_mouse algorithm */
    switch(dir)
    {
    case RIGHT:
    	//Add code to turn on LED with GPIO_DRV call
    	GPIO_DRV_WritePinOutput(kGpioLED1, 0);
        g_mouse.rpt_buf[1] = 2;
        g_mouse.rpt_buf[2] = 0;
        x++;
        if (x > 100)
        {
            dir++;
        }
        break;
    case DOWN:
    	//Add code to turn on LED with GPIO_DRV call
    	GPIO_DRV_WritePinOutput(kGpioLED1, 0);
        g_mouse.rpt_buf[1] = 0;
        g_mouse.rpt_buf[2] = 2;
        y++;
        if (y > 100)
        {
            dir++;
        }
        break;
    case LEFT:
    	//Add code to turn off LED with GPIO_DRV call
    	GPIO_DRV_WritePinOutput(kGpioLED1, 1);
        g_mouse.rpt_buf[1] = (uint8_t)(-2);
        g_mouse.rpt_buf[2] = 0;
        x--;
        if (x < 0)
        {
            dir++;
        }
        break;
    case UP:
    	//Add code to turn off LED with GPIO_DRV call
    	GPIO_DRV_WritePinOutput(kGpioLED1, 1);
        g_mouse.rpt_buf[1] = 0;
        g_mouse.rpt_buf[2] = (uint8_t)(-2);
        y--;
        if (y < 0)
        {
            dir = RIGHT;
        }
        break;
    }
#endif

#if LAB3
    /* Lab 3: Accelerometer based move_mouse */

   if(move_direction==RIGHT)
   {
	   //Insert code to on LED using GPIO_DRV
	   GPIO_DRV_WritePinOutput(kGpioLED1, 0);
	   g_mouse.rpt_buf[1] = 2;
	   g_mouse.rpt_buf[2] = 0;
   }
   else if(move_direction==LEFT)
   {
	   //Insert code to turn on LED using GPIO_DRV
	   GPIO_DRV_WritePinOutput(kGpioLED1, 0);
	   g_mouse.rpt_buf[1] = -2;
       g_mouse.rpt_buf[2] = 0;
   }
   else
   {
	   //Insert code to turn off LED using GPIO_DRV
	   GPIO_DRV_WritePinOutput(kGpioLED1, 1);
       g_mouse.rpt_buf[1] = 0;
       g_mouse.rpt_buf[2] = 0;
   }
#endif

    //Send move data
    (void) USB_Class_HID_Send_Data(g_mouse.app_handle, HID_ENDPOINT,
        g_mouse.rpt_buf, MOUSE_BUFF_SIZE);
}

/******************************************************************************
 * 
 *    @name        USB_App_Device_Callback
 *    
 *    @brief       This function handles the callback  
 *                  
 *    @param       handle : handle to Identify the controller
 *    @param       event_type : value of the event
 *    @param       val : gives the configuration value 
 * 
 *    @return      None
 *
 *****************************************************************************/
void USB_App_Device_Callback(uint8_t event_type, void* val, void* arg)
{
    UNUSED_ARGUMENT (arg)
    UNUSED_ARGUMENT (val)

    switch(event_type)
    {
    case USB_DEV_EVENT_BUS_RESET:
        g_mouse.mouse_init = FALSE;
        if (USB_OK == USB_Class_HID_Get_Speed(g_mouse.app_handle, &g_mouse.app_speed))
        {
            USB_Desc_Set_Speed(g_mouse.app_handle, g_mouse.app_speed);
        }
        break;
    case USB_DEV_EVENT_ENUM_COMPLETE:
        g_mouse.mouse_init = TRUE;
        printf("App device enum complete\n");
        move_mouse();/* run the cursor movement code */
        break;
    case USB_DEV_EVENT_ERROR:
        /* user may add code here for error handling 
         NOTE : val has the value of error from h/w*/
        break;
    default:
        break;
    }
    return;
}

/******************************************************************************
 * 
 *    @name        USB_App_Class_Callback
 *    
 *    @brief       This function handles the callback for Get/Set report req  
 *                  
 *    @param       request  :  request type
 *    @param       value    :  give report type and id
 *    @param       data     :  pointer to the data 
 *    @param       size     :  size of the transfer
 *
 *    @return      status
 *                  USB_OK  :  if successful
 *                  else return error
 *
 *****************************************************************************/
uint8_t USB_App_Class_Callback
(
    uint8_t request,
    uint16_t value,
    uint8_t ** data,
    uint32_t* size,
    void* arg
) 
{
    uint8_t error = USB_OK;
    uint8_t index = (uint8_t)((request - 2) & USB_HID_REQUEST_TYPE_MASK);

    if ((request == USB_DEV_EVENT_SEND_COMPLETE) && (value == USB_REQ_VAL_INVALID))
    {
        if ((g_mouse.mouse_init) && (arg != NULL))
        {
            printf("App class callback send complete\n");
            move_mouse();/* run the cursor movement code */
        }
        return error;
    }
    /* index == 0 for get/set idle, index == 1 for get/set protocol */
    *size = 0;
    /* handle the class request */
    switch(request)
    {
    case USB_HID_GET_REPORT_REQUEST:
        *data = &g_mouse.rpt_buf[0]; /* point to the report to send */
        *size = MOUSE_BUFF_SIZE; /* report size */
        break;

    case USB_HID_SET_REPORT_REQUEST:
        for (index = 0; index < MOUSE_BUFF_SIZE; index++)
        { /* copy the report sent by the host */
            g_mouse.rpt_buf[index] = *(*data + index);
        }
        break;

    case USB_HID_GET_IDLE_REQUEST:
        /* point to the current idle rate */
        *data = &g_mouse.app_request_params[index];
        *size = REQ_DATA_SIZE;
        break;

    case USB_HID_SET_IDLE_REQUEST:
        /* set the idle rate sent by the host */
        g_mouse.app_request_params[index] = (uint8_t)((value & MSB_MASK) >>
        HIGH_BYTE_SHIFT);
        break;

    case USB_HID_GET_PROTOCOL_REQUEST:
        /* point to the current protocol code 
         0 = Boot Protocol
         1 = Report Protocol*/
        *data = &g_mouse.app_request_params[index];
        *size = REQ_DATA_SIZE;
        break;

    case USB_HID_SET_PROTOCOL_REQUEST:
        /* set the protocol sent by the host 
         0 = Boot Protocol
         1 = Report Protocol*/
        g_mouse.app_request_params[index] = (uint8_t)(value);
        break;
    }
    return error;
}

/******************************************************************************
 *  
 *   @name        TestApp_Init
 * 
 *   @brief       This function is the entry for mouse (or other usage)
 * 
 *   @param       None
 * 
 *   @return      None
 **                
 *****************************************************************************/
void APP_init(void)
{
    hid_config_struct_t config_struct;

    /* initialize the Global Variable Structure */
    OS_Mem_zero(&g_mouse, sizeof(mouse_global_variable_struct_t));
    OS_Mem_zero(&config_struct, sizeof(hid_config_struct_t));

#if (OS_ADAPTER_ACTIVE_OS == OS_ADAPTER_MQX)
    g_mouse.rpt_buf = (uint8_t*) OS_Mem_alloc_uncached_align(MOUSE_BUFF_SIZE, 32);
    if (NULL == g_mouse.rpt_buf)
    {
        USB_PRINTF("\nMalloc error in APP_init\n");
        return;
    }
    OS_Mem_zero(g_mouse.rpt_buf, MOUSE_BUFF_SIZE);
#endif
    /* Initialize the USB interface */
    USB_PRINTF("USB Mouse Demo\r\n");
    config_struct.hid_application_callback.callback = USB_App_Device_Callback;
    config_struct.hid_application_callback.arg = &g_mouse.app_handle;
    config_struct.class_specific_callback.callback = USB_App_Class_Callback;
    config_struct.class_specific_callback.arg = &g_mouse.app_handle;
    config_struct.desc_callback_ptr = &g_desc_callback;

    g_mouse.app_speed = USB_SPEED_FULL;
    USB_Class_HID_Init(CONTROLLER_ID, &config_struct, &g_mouse.app_handle);
}

void APP_task()
{
    USB_HID_Periodic_Task();
}

#if (OS_ADAPTER_ACTIVE_OS == OS_ADAPTER_MQX)

/*FUNCTION*----------------------------------------------------------------
 * 
 * Function Name  : Main_Task
 * Returned Value : None
 * Comments       :
 *     First function called.  Calls Test_App
 *     callback functions.
 * 
 *END*--------------------------------------------------------------------*/
void Main_Task(uint32_t param)
{
    UNUSED_ARGUMENT (param)
    APP_init();
}
#endif

#if (OS_ADAPTER_ACTIVE_OS == OS_ADAPTER_SDK)

#if defined(FSL_RTOS_MQX)
void Main_Task(uint32_t param);
TASK_TEMPLATE_STRUCT MQX_template_list[] =
{
    {   1L, Main_Task, 2500L, MQX_MAIN_TASK_PRIORITY, "Main", MQX_AUTO_START_TASK},
    {   0L, 0L, 0L, 0L, 0L, 0L}
};
#endif

#if (USE_RTOS)
static void Task_Start(void *arg)
{
    APP_init();
    for (;; )
    {
        ;
    }
}
#endif



int main(void)
{

	i2c_master_state_t fxos8700_master;

	uint8_t reg;

	uint8_t sendBuff[1] = {0xFF};       //save data sent to i2c slave
	uint8_t receiveBuff[6] = {0xFF};    //save data received from i2c slave

	i2c_master_state_t master;

	i2c_device_t slave =
	{
			.address = 0x1C,       //accel
	        .baudRate_kbps = 100
	};

	hardware_init();
	OSA_Init();
	dbg_uart_init();

	//Initialize GPIO
	GPIO_DRV_OutputPinInit(&ledPins[0]);
	GPIO_DRV_WritePinOutput(kGpioLED1, 0);

#ifdef LAB3
	    // Init I2C module
	    I2C_DRV_MasterInit(BOARD_I2C_COMM_INSTANCE, &master);

	    OSA_TimeDelay(500);

	    //Place FXOS8700 into standby mode
	    reg=CTRL_REG1;
	    sendBuff[0]=0x0;
	    I2C_DRV_MasterSendDataBlocking(BOARD_I2C_COMM_INSTANCE, &slave, &reg, 1, sendBuff,  1,  200);

	    //Setup FXOS8700 at 200Hz sample rate and enable sampling
	    reg=CTRL_REG1;
	    sendBuff[0]=0x0D;
	    I2C_DRV_MasterSendDataBlocking(BOARD_I2C_COMM_INSTANCE, &slave, &reg, 1, sendBuff,  1, 200);
#endif


    APP_init(); //Initialize USB HID Class

    //Let USB run while read in accelerometer data
    while(1)
    {
	    reg=OUT_X_MSB_REG;  //0x01

	    //Insert code to read accelerometer data
	    I2C_DRV_MasterReceiveDataBlocking(BOARD_I2C_COMM_INSTANCE, &slave, &reg, 1,receiveBuff, 6, 200);

    	if(receiveBuff[0]>150 && receiveBuff[0]<200)
    	{
    		move_direction=RIGHT;
    	}
    	else if(receiveBuff[0]>40 && receiveBuff[0]<100)
    	{
    		move_direction=LEFT;
    	}
    	else
    	{
    		move_direction=NONE;
    	}
    }

}
#endif
/* EOF */
