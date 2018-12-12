//
// Created by Christoph Schramm on 06.12.18.
//



#include <sys/types.h>
#include "spi.h"
#include "cc3100.h"
#include <logging/log.h>
LOG_MODULE_REGISTER(cc3100, 3);
#include <gpio.h>

#include <hal/nrf_gpiote.h>
#include <simplelink.h>
#include "lib/sl_common.h"


#define SPI_TIMEOUT_MAX 0x1000

static struct device *gpio_hib;

/* Static function declarations */
static int SPI_Init();
static int SPI_CS_Init();
static void Error_Handler();

unsigned int key;

K_THREAD_STACK_DEFINE(simplelink_workq_stack,
					  2048);
static struct k_work_q simplelink_workq;

static _volatile bool IntIsMasked;
static bool has_pending_irq;
// from: TI sample
P_EVENT_HANDLER     pIrqEventHandler = 0;

// TODO:
static struct spi_simplelink_data *default_driver_data;

void ASSERT_CS(void)
{
	struct device *gpio = device_get_binding(CONFIG_SPI_FLASH_W25QXXDV_GPIO_SPI_CS_DRV_NAME);
	gpio_pin_write(gpio, CONFIG_SPI_FLASH_W25QXXDV_GPIO_SPI_CS_PIN, 0);
}

void DEASSERT_CS(void)
{
	struct device *gpio = device_get_binding(CONFIG_SPI_FLASH_W25QXXDV_GPIO_SPI_CS_DRV_NAME);
	gpio_pin_write(gpio, CONFIG_SPI_FLASH_W25QXXDV_GPIO_SPI_CS_PIN, 1);
}

// from: cc_pal.c

void NwpMaskInterrupt_work()
{
	key = irq_lock();
	nrf_gpiote_int_disable(NRF_GPIOTE_EVENTS_PORT);
	IntIsMasked = true;

	LOG_DBG("MAsked");
}
void NwpMaskInterrupt()
{
	static struct k_work irq_work;
	k_work_init(&irq_work, NwpMaskInterrupt_work);
	k_work_submit_to_queue(&simplelink_workq, &irq_work);
}

void NwpUnMaskInterrupt_work()
{

	irq_unlock(key);
	nrf_gpiote_int_enable(NRF_GPIOTE_EVENTS_PORT);
	IntIsMasked = false;
	LOG_DBG("UNmasked");
}
static void simplelink_irq_handle(struct k_work *work);
void NwpUnMaskInterrupt()
{

	static struct k_work irq_work;
	k_work_init(&irq_work, NwpUnMaskInterrupt_work);
	k_work_submit_to_queue(&simplelink_workq, &irq_work);
	if(has_pending_irq)
	{
		has_pending_irq = false;
		simplelink_irq_handle(NULL);
	}
}

int Semaphore_create_handle(SemaphoreP_Handle* pSemHandle)
{
	SemaphoreP_Params params;

	SemaphoreP_Params_init(&params);

	params.mode = SemaphoreP_Mode_BINARY;

#ifndef SL_PLATFORM_MULTI_THREADED
	params.callback = tiDriverSpawnCallback;
#endif
	(*(pSemHandle)) = SemaphoreP_create(1, &params);

	if(!(*(pSemHandle)))
	{
		return SemaphoreP_FAILURE ;
	}

	return SemaphoreP_OK;
}

int SemaphoreP_delete_handle(SemaphoreP_Handle* pSemHandle)
{
	SemaphoreP_delete(*(pSemHandle));
	return SemaphoreP_OK;
}

int SemaphoreP_post_handle(SemaphoreP_Handle* pSemHandle)
{
	SemaphoreP_post(*(pSemHandle));
	return SemaphoreP_OK;
}


int Mutex_create_handle(MutexP_Handle* pMutexHandle)
{
	MutexP_Params params;

	MutexP_Params_init(&params);
#ifndef SL_PLATFORM_MULTI_THREADED
	params.callback = tiDriverSpawnCallback;
#endif

	(*(pMutexHandle)) = MutexP_create(&params);

	if(!(*(pMutexHandle)))
	{
		return MutexP_FAILURE ;
	}

	return MutexP_OK;
}

int  MutexP_delete_handle(MutexP_Handle* pMutexHandle)
{
	MutexP_delete(*(pMutexHandle));
	return(MutexP_OK);
}

int Mutex_unlock(MutexP_Handle pMutexHandle)
{
	//LOG_DBG("Mutex_unlock");
	MutexP_unlock(pMutexHandle, 0);
	return(MutexP_OK);
}


int Mutex_lock(MutexP_Handle pMutexHandle)
{
	//LOG_DBG("Mutex_lock");
	MutexP_lock(pMutexHandle);
	return(MutexP_OK);
}


unsigned long TimerGetCurrentTimestamp()
{
	return (ClockP_getSystemTicks());
}


void NwpWaitForShutDownInd()
{
	LOG_DBG("NwpWaitForShutDownInd");
//	volatile unsigned long nwp_wakup_ind = HWREG(NWP_LPDS_WAKEUPCFG);
//	_SlTimeoutParams_t SlTimeoutInfo = {0};
//
//	_SlDrvStartMeasureTimeout(&SlTimeoutInfo, NWP_LPDS_WAKEUPCFG_TIMEOUT_MSEC);
//
//    while(nwp_wakup_ind != NWP_LPDS_WAKEUPCFG_APPS2NWP)
//    {
//    	if(_SlDrvIsTimeoutExpired(&SlTimeoutInfo))
//    	{
//    		return;
//    	}
//    	nwp_wakup_ind = HWREG(NWP_LPDS_WAKEUPCFG);
//    }

	return ;
}





















int NwpRegisterInterruptHandler(P_EVENT_HANDLER InterruptHdl , void* pValue) {
	LOG_DBG("NwpRegisterInterruptHandler");
	if(InterruptHdl)
	{
		pIrqEventHandler = InterruptHdl;

	}

	// TODO: move gpio
	struct device *gpio = device_get_binding(CONFIG_SPI_FLASH_W25QXXDV_GPIO_IRQ_DRV_NAME);

	if(!InterruptHdl)
	{
		LOG_DBG("Interrupt handler removed");
		gpio_pin_disable_callback(gpio, CONFIG_SPI_FLASH_W25QXXDV_GPIO_IRQ_PIN);
		has_pending_irq = false;
	} else {
		LOG_DBG("Interrupt handler added");
		gpio_pin_enable_callback(gpio, CONFIG_SPI_FLASH_W25QXXDV_GPIO_IRQ_PIN);
	}
	NwpUnMaskInterrupt();

	return OS_OK ;
}

// from: cc_pal.c
void NwpPowerOn(void)
{
	LOG_DBG("PWR on");
	struct device *gpio = device_get_binding(CONFIG_SPI_FLASH_W25QXXDV_GPIO_NHIB_DRV_NAME);
	gpio_pin_write(gpio, CONFIG_SPI_FLASH_W25QXXDV_GPIO_NHIB_PIN, 1);
}


// from: cc_pal.c
void NwpPowerOff(void)
{
	LOG_DBG("PWR off");
	// TODO: move gpio
	struct device *gpio = device_get_binding(CONFIG_SPI_FLASH_W25QXXDV_GPIO_NHIB_DRV_NAME);
	gpio_pin_write(gpio, CONFIG_SPI_FLASH_W25QXXDV_GPIO_NHIB_PIN, 0);
	k_sleep(100);
}


/*
 * ASYNCHRONOUS EVENT HANDLERS -- Start
 */
/*!
    \brief This function handles WLAN events

    \param[in]      pWlanEvents is the event passed to the handler

    \return         none

    \note

    \warning
*/
_u8 g_Status = 0;
_u32 g_DeviceIp = 0;
_i8 g_p2p_dev[MAXIMAL_SSID_LENGTH + 1];
void SimpleLinkWlanEventHandler(SlWlanEvent_t *pWlanEvent)
{
	LOG_ERR("SimpleLionkEventHandler");
	if(pWlanEvent == NULL)
	{
		LOG_ERR(" [WLAN EVENT] NULL Pointer Error");
		return;
	}

	switch(pWlanEvent->Event)
	{
		case SL_WLAN_CONNECT_EVENT:
			/*
			 * Information about the connected AP (like name, MAC etc) will be
			 * available in 'slWlanConnectAsyncResponse_t' - Applications
			 * can use it if required
			 *
			 * slWlanConnectAsyncResponse_t *pEventData = NULL;
			 * pEventData = &pWlanEvent->EventData.STAandP2PModeWlanConnected;
			 *
			 */
			SET_STATUS_BIT(g_Status, STATUS_BIT_CONNECTION);
			break;
		case SL_WLAN_STA_CONNECTED_EVENT:
			/*
			 * Information about the connected STA (like name, MAC etc) will be
			 * available in 'slPeerInfoAsyncResponse_t' - Applications
			 * can use it if required
			 *
			 * slPeerInfoAsyncResponse_t *pEventData = NULL;
			 * pEventData = &pWlanEvent->EventData.APModeStaConnected;
			 *
			 */
			SET_STATUS_BIT(g_Status, STATUS_BIT_STA_CONNECTED);
			break;

		case SL_WLAN_DISCONNECT_EVENT:
			/*
			 * Information about the disconnected STA and reason code will be
			 * available in 'slWlanConnectAsyncResponse_t' - Applications
			 * can use it if required
			 *
			 * slWlanConnectAsyncResponse_t *pEventData = NULL;
			 * pEventData = &pWlanEvent->EventData.STAandP2PModeDisconnected;
			 *
			 */
			CLR_STATUS_BIT(g_Status, STATUS_BIT_CONNECTION);
			CLR_STATUS_BIT(g_Status, STATUS_BIT_IP_ACQUIRED);
			break;
		case SL_WLAN_STA_DISCONNECTED_EVENT:
			/*
			 * Information about the connected STA (device name, MAC) will be
			 * available in 'slPeerInfoAsyncResponse_t' - Applications
			 * can use it if required
			 *
			 * slPeerInfoAsyncResponse_t *pEventData = NULL;
			 * pEventData = &pWlanEvent->EventData.APModeStaConnected;
			 *
			 */
			CLR_STATUS_BIT(g_Status, STATUS_BIT_STA_CONNECTED);
			CLR_STATUS_BIT(g_Status, STATUS_BIT_IP_LEASED);
			break;

		case SL_WLAN_CONNECTION_FAILED_EVENT:
			/*
			 * Status code for connection faliure will be available
			 * in 'slWlanConnFailureAsyncResponse_t' - Application
			 * can use it if required.
			 *
			 * slWlanConnFailureAsyncResponse_t *pEventData = NULL;
			 * pEventData = &pWlanEvent->EventData.P2PModewlanConnectionFailure
			 *
			 */
			;
			SET_STATUS_BIT(g_Status, STATUS_BIT_CONNECTION_FAILED);
			break;

		case SL_WLAN_P2P_NEG_REQ_RECEIVED_EVENT:
			SET_STATUS_BIT(g_Status, STATUS_BIT_P2P_NEG_REQ_RECEIVED);

			pal_Memset(g_p2p_dev, '\0', MAXIMAL_SSID_LENGTH + 1);
			pal_Memcpy(g_p2p_dev,pWlanEvent->EventData.P2PModeNegReqReceived.go_peer_device_name,
					   pWlanEvent->EventData.P2PModeNegReqReceived.go_peer_device_name_len);
			break;

		case SL_WLAN_P2P_DEV_FOUND_EVENT:
			/*
			 * Information about the remote P2P device (device name and MAC)
			 * will be available in 'slPeerInfoAsyncResponse_t' - Applications
			 * can use it if required
			 *
			 * slPeerInfoAsyncResponse_t *pEventData = NULL;
			 * pEventData = &pWlanEvent->EventData.P2PModeDevFound;
			 *
			 */
			break;

		default:
			break;

	}
}

/*!
    \brief This function handles events for IP address acquisition via DHCP
           indication

    \param[in]      pNetAppEvent is the event passed to the handler

    \return         None

    \note

    \warning
*/
void SimpleLinkNetAppEventHandler(SlNetAppEvent_t *pNetAppEvent)
{
	if(pNetAppEvent == NULL)
	{
		LOG_ERR(" [NETAPP EVENT] NULL Pointer Error");
		return;
	}

	switch( pNetAppEvent->Event )
	{
		case SL_NETAPP_IPV4_IPACQUIRED_EVENT:
		{
			SlIpV4AcquiredAsync_t *pEventData = NULL;
			pEventData = &pNetAppEvent->EventData.ipAcquiredV4;

			g_DeviceIp = pEventData->ip;
		}
			SET_STATUS_BIT(g_Status, STATUS_BIT_IP_ACQUIRED);
			break;
		case SL_NETAPP_IP_LEASED_EVENT:
			/*
			 * Information about the connection (like leased IP, lease time etc)
			 * will be available in 'SlIpLeasedAsync_t'
			 * Applications can use it if required
			 *
			 * SlIpLeasedAsync_t *pEventData = NULL;
			 * pEventData = &pNetAppEvent->EventData.ipLeased;
			 *
			 */
			SET_STATUS_BIT(g_Status, STATUS_BIT_IP_LEASED);
			break;

		default:
			LOG_WRN(" [NETAPP EVENT] Unexpected event");
			break;
	}
}

/*!
    \brief This function handles general error events indication

    \param[in]      pDevEvent is the event passed to the handler

    \return         None
*/
void SimpleLinkGeneralEventHandler(SlDeviceEvent_t *pDevEvent)
{
	/*
	 * Most of the general errors are not FATAL are are to be handled
	 * appropriately by the application
	 */
	LOG_DBG(" [GENERAL EVENT] ");
	if(pDevEvent->Event)
	{
		LOG_ERR("Error: %d", pDevEvent->Event);
	}
}

/*!
    \brief open spi communication port to be used for communicating with a SimpleLink device

    Given an interface name and option flags, this function opens the spi communication port
    and creates a file descriptor. This file descriptor can be used afterwards to read and
    write data from and to this specific spi channel.
    The SPI speed, clock polarity, clock phase, chip select and all other attributes are all
    set to hardcoded values in this function.

    \param          ifName      -   points to the interface name/path. The interface name is an
                                    optional attributes that the simple link driver receives
                                    on opening the device. in systems that the spi channel is
                                    not implemented as part of the os device drivers, this
                                    parameter could be NULL.
    \param          flags       -   option flags

    \return         upon successful completion, the function shall open the spi channel and return
                    a non-negative integer representing the file descriptor.
                    Otherwise, -1 shall be returned

    \sa             spi_Close , spi_Read , spi_Write
    \note
    \warning
*/
Fd_t spi_Open(char *ifName, unsigned long flags)
{

	SPI_Init();

	// TODO: move gpio
	struct device *gpio = device_get_binding(CONFIG_SPI_FLASH_W25QXXDV_GPIO_NHIB_DRV_NAME);
	gpio_pin_configure(gpio, CONFIG_SPI_FLASH_W25QXXDV_GPIO_NHIB_PIN, GPIO_DIR_OUT);
	gpio_pin_write(gpio, CONFIG_SPI_FLASH_W25QXXDV_GPIO_NHIB_PIN, 1);

	k_sleep(50);
	return 0;
}

/*!
    \brief closes an opened spi communication port

    \param          fd          -   file descriptor of an opened SPI channel

    \return         upon successful completion, the function shall return 0.
                    Otherwise, -1 shall be returned

    \sa             spi_Open
    \note
    \warning
*/
int spi_Close(Fd_t fd)
{
	LOG_DBG("Implement me");
	/* Disable Interrupt in GPIOA module... */
//	CC3100_InterruptDisable();
//
//	/* Deinitialize SPI */
//	HAL_SPI_DeInit(&SpiHandle);
	return 0;
}

/*!
    \brief attempts to write up to len bytes to the SPI channel

    \param          fd          -   file descriptor of an opened SPI channel

    \param          pBuff       -   points to first location to start getting the data from

    \param          len         -   number of bytes to write to the SPI channel

    \return         upon successful completion, the function shall return 0.
                    Otherwise, -1 shall be returned

    \sa             spi_Open , spi_Read
    \note           This function could be implemented as zero copy and return only upon successful completion
                    of writing the whole buffer, but in cases that memory allocation is not too tight, the
                    function could copy the data to internal buffer, return back and complete the write in
                    parallel to other activities as long as the other SPI activities would be blocked untill
                    the entire buffer write would be completed
    \warning
*/
u8_t tx_data_in_ram[CONFIG_SPI_SIMPLELINK_SPI_BUFFER_SIZE];
struct spi_buf tx_buf = {
		.len = 0,
		.buf = tx_data_in_ram
};

// copied from spi_flash_w25
int spi_Write (Fd_t fd, unsigned char *pBuff, int len)
{
	// TODO: don't use default driver data
	memset(tx_data_in_ram, 0, CONFIG_SPI_SIMPLELINK_SPI_BUFFER_SIZE);
	memcpy(tx_data_in_ram, pBuff, (size_t)len);





	tx_buf.len = (size_t)len;

	struct spi_buf buf[1] = {
			{
					.buf = tx_data_in_ram,//pBuff,
					.len = (size_t)len
			}
	};
	const struct spi_buf_set tx = {
			.buffers = buf,
			.count = 1
	};

	LOG_DBG("Writing %u bytes", len);

	//ASSERT_CS();
	//spi_transceive(default_driver_data->spi, &default_driver_data->spi_cfg, &tx, &rx);
	//DEASSERT_CS();

	spi_write(default_driver_data->spi, &default_driver_data->spi_cfg, &tx);
	LOG_HEXDUMP_DBG(tx_data_in_ram, len, "After TX bytes");

	/*
	struct spi_simplelink_data *const driver_data = default_driver_data;

	u8_t data_in_ram[CONFIG_SPI_SIMPLELINK_SPI_BUFFER_SIZE];
	memset(data_in_ram, 0, CONFIG_SPI_SIMPLELINK_SPI_BUFFER_SIZE);
	memcpy(data_in_ram, pBuff, len);

	struct spi_buf buf[1] = {
			{
					.buf = data_in_ram,//pBuff,
					.len = (size_t)len
			}
	};

	const struct spi_buf_set tx = {
			.buffers = buf,
			.count = 1
	};


	int write_success = spi_write(driver_data->spi, &driver_data->spi_cfg, &tx);
	LOG_DBG("Write success: %d", write_success);
	return write_success;*/

	return len;
}

static int spi_flash_wb_access(struct spi_simplelink_data *ctx,
							   const u8_t* cmd,
							   void *data, size_t length, bool write)
{
	u8_t access[4];
	struct spi_buf buf[2] = {
			{
					.buf = access
			},
			{
					.buf = data,
					.len = ((length+3)>>2)
			}
	};
	struct spi_buf_set tx = {
			.buffers = buf,
	};



	//tx.count = length ? 2 : 1;
	tx.count = 1;


	if (!write) {
		const struct spi_buf_set rx = {
				.buffers = buf,
				.count = 1
		};

		return spi_read(ctx->spi, &ctx->spi_cfg, &rx);
		//return spi_transceive(ctx->spi, &ctx->spi_cfg, &tx, &rx);
	}

	return spi_write(ctx->spi, &ctx->spi_cfg, &tx);
}

/*!
    \brief attempts to read up to len bytes from SPI channel into a buffer starting at pBuff.

    \param          fd          -   file descriptor of an opened SPI channel

    \param          pBuff       -   points to first location to start writing the data

    \param          len         -   number of bytes to read from the SPI channel

    \return         upon successful completion, the function shall return 0.
                    Otherwise, -1 shall be returned

    \sa             spi_Open , spi_Write
    \note
    \warning
*/
int spi_Read(Fd_t fd, unsigned char *pBuff, int len)
{
	/*
	static int spi_flash_wb_access(struct spi_simplelink_data *ctx,
							   u8_t cmd, bool addressed, off_t offset,
							   void *data, size_t length, bool write)
	 */
	//u8_t data_in_ram[CONFIG_SPI_SIMPLELINK_SPI_BUFFER_SIZE];
	//memset(data_in_ram, 0, CONFIG_SPI_SIMPLELINK_SPI_BUFFER_SIZE);

/*
	u8_t data_in_ram[CONFIG_SPI_SIMPLELINK_SPI_BUFFER_SIZE];
	memset(data_in_ram, 0, CONFIG_SPI_SIMPLELINK_SPI_BUFFER_SIZE);
	memcpy(data_in_ram, pBuff, len);


	ASSERT_CS();
	int wb_succ = spi_flash_wb_access(default_driver_data, data_in_ram, data_in_ram, 4, false);
	DEASSERT_CS();
	//LOG_HEXDUMP_DBG(tx_buf.buf, tx_buf.len, "TX bytes");
	LOG_HEXDUMP_DBG(data_in_ram, len, "RX bytes");

	return wb_succ;*/
	LOG_DBG("Reading %d bytes", len);
	// TODO: don't use default driver data


	struct spi_simplelink_data *const driver_data = default_driver_data;


	size_t data_len = (size_t) len;//0;

	struct spi_buf buf[1] = {
			{
					.buf = pBuff,
					.len = len//(size_t) len
			}
	};

	struct spi_buf bufs_out[1] = { tx_buf };


	const struct spi_buf_set rx = {
			.buffers = buf,
			.count = 1
	};


	int read_success = -1;

	if(1 == 2 && tx_buf.len > 0)
	{
		const struct spi_buf_set tx = {
				.buffers = bufs_out,
				.count = 1
		};

		LOG_DBG("Writing before read");
		LOG_HEXDUMP_DBG(tx_buf.buf, tx_buf.len, "TX %u bytes");
		read_success = spi_transceive(driver_data->spi, &driver_data->spi_cfg, &tx, &rx);
		data_len += tx_buf.len;
		LOG_HEXDUMP_DBG(buf->buf, buf->len, "RX %u bytes");

	} else {

		read_success = spi_read(driver_data->spi, &driver_data->spi_cfg, &rx);
		data_len = 0;
		LOG_HEXDUMP_DBG(pBuff, buf->len, "RX bytes");


	}


	return len;

}

/**
  * @brief SPI MSP Initialization
  *        This function configures the hardware resources used in this example:
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration
  *           - NVIC configuration for SPI interrupt request enable
  * @param hspi: SPI handle pointer
  * @retval None
  */
static int SPI_Init()
{
	/* Set the SPI parameters */
	LOG_DBG("Implement me");
//	SpiHandle.Instance               = SPI2;
//	SpiHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
//	SpiHandle.Init.Direction         = SPI_DIRECTION_2LINES;
//	SpiHandle.Init.CLKPhase          = SPI_PHASE_1EDGE;
//	SpiHandle.Init.CLKPolarity       = SPI_POLARITY_LOW;
//	SpiHandle.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLED;
//	SpiHandle.Init.CRCPolynomial     = 7;
//	SpiHandle.Init.DataSize          = SPI_DATASIZE_8BIT;
//	SpiHandle.Init.FirstBit          = SPI_FIRSTBIT_MSB;
//	SpiHandle.Init.NSS               = SPI_NSS_SOFT;
//	SpiHandle.Init.TIMode            = SPI_TIMODE_DISABLED;
//	SpiHandle.Init.Mode              = SPI_MODE_MASTER;
//	if(HAL_SPI_Init(&SpiHandle) != HAL_OK)
//	{
//		/* Initialization Error */
//		Error_Handler();
//	}
//
//	/* Configure the SPI CS to be on PB12 */
SPI_CS_Init();
//
//	/* Enable interrupt on the GPIOA pin of CC3100 IRQ */
//	CC3100_InterruptEnable();

	return 0;
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
	LOG_DBG("Implement me");
	while(1)
		;
}

/**/
static int SPI_CS_Init()
{
	struct device *gpio = device_get_binding(CONFIG_SPI_FLASH_W25QXXDV_GPIO_SPI_CS_DRV_NAME);
	gpio_pin_configure(gpio, CONFIG_SPI_FLASH_W25QXXDV_GPIO_SPI_CS_PIN, GPIO_DIR_OUT);
	gpio_pin_write(gpio, CONFIG_SPI_FLASH_W25QXXDV_GPIO_SPI_CS_PIN, 1);
//	GPIO_InitTypeDef   GPIO_InitStructure;
//
//	/* Enable GPIOx clock */
//	__GPIOB_CLK_ENABLE();
//
//	/* Configure pin as input floating */
//	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
//	GPIO_InitStructure.Pull = GPIO_PULLUP;
//	GPIO_InitStructure.Pin = SPI_CS_PIN;
//	GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
//
//	HAL_GPIO_Init(SPI_CS_PORT, &GPIO_InitStructure);
//
//	HAL_GPIO_WritePin(SPI_CS_PORT, SPI_CS_PIN, GPIO_PIN_SET);
	return 0;
}







static int spi_simplelink_access(struct spi_simplelink_data *ctx,
							   u8_t cmd, bool addressed, off_t offset,
							   void *data, size_t length, bool write)
{
	u8_t access[4];
	struct spi_buf buf[2] = {
			{
					.buf = access
			},
			{
					.buf = data,
					.len = length
			}
	};
	struct spi_buf_set tx = {
			.buffers = buf,
	};

	access[0] = cmd;

	if (addressed) {
		access[1] = (u8_t) (offset >> 16);
		access[2] = (u8_t) (offset >> 8);
		access[3] = (u8_t) offset;

		buf[0].len = 4;
	} else {
		buf[0].len = 1;
	}

	tx.count = length ? 2 : 1;

	if (!write) {
		const struct spi_buf_set rx = {
				.buffers = buf,
				.count = 2
		};

		return spi_transceive(ctx->spi, &ctx->spi_cfg, &tx, &rx);
	}

	return spi_write(ctx->spi, &ctx->spi_cfg, &tx);
}

static inline int spi_simplelink_id(struct device *dev)
{
	struct spi_flash_data *const driver_data = dev->driver_data;
	u32_t temp_data;
	u8_t buf[3];

	/* TODO
	if (spi_simplelink_access(driver_data, W25QXXDV_CMD_RDID,
							false, 0, buf, 3, false) != 0) {
		return -EIO;
	}

	temp_data = ((u32_t) buf[0]) << 16;
	temp_data |= ((u32_t) buf[1]) << 8;
	temp_data |= (u32_t) buf[2];

	if (temp_data != CONFIG_SPI_FLASH_W25QXXDV_DEVICE_ID) {
		return -ENODEV;
	}*/

	return 0;
}


static struct gpio_callback gpio_cb;

/*
 k_delayed_work_init(&init_work, simplelink_init_work);

	k_work_q_start(&simplelink_workq,
				   simplelink_workq_stack,
				   K_THREAD_STACK_SIZEOF(simplelink_workq_stack),
				   K_PRIO_COOP(12));

	ret = k_delayed_work_submit_to_queue(&simplelink_workq,
										 &init_work, K_MSEC(10));
 */

static void simplelink_irq_handle(struct k_work *work)
{
	LOG_DBG("Handle IRQ");
	struct device *gpio = device_get_binding(CONFIG_SPI_FLASH_W25QXXDV_GPIO_IRQ_DRV_NAME);

	//DiagIrqHandler();

	u32_t val = 0;
	if(!IntIsMasked && pIrqEventHandler )
	{
		LOG_DBG("IRQ triggered");
		pIrqEventHandler();


		// TODO: only testing
		//LOG_DBG("Remove callback");
		//gpio_pin_disable_callback(gpio, CONFIG_SPI_FLASH_W25QXXDV_GPIO_IRQ_PIN);
	} else {
		has_pending_irq = true;
		LOG_DBG("Interrupt triggered, but not handled");
		if(IntIsMasked)
		{
			LOG_DBG("Because interrupt is masked: %u", IntIsMasked);
		} else {
			LOG_DBG("Because there's no interrupt handler");
		}
	}
#ifdef CONFIG_HAS_HW_NRF_GPIOTE
	nrf_gpiote_event_clear(NRF_GPIOTE_EVENTS_PORT);
#endif
}

void button_pressed(struct device *gpiob, struct gpio_callback *cb,
					u32_t pins)
{
	static struct k_work irq_work;
	k_work_init(&irq_work, simplelink_irq_handle);
	k_work_submit_to_queue(&simplelink_workq, &irq_work);

}

static int spi_simplelink_configure(struct device *dev)
{
	struct spi_simplelink_data *data = dev->driver_data;
	default_driver_data = data;

	LOG_INF("Binding CC3100 to %s", log_strdup(CONFIG_SPI_SIMPLELINK_SPI_NAME));
	data->spi = device_get_binding(CONFIG_SPI_SIMPLELINK_SPI_NAME);
	if (!data->spi) {
		LOG_WRN("Can't bind SPI");
		return -EINVAL;
	}

	data->spi_cfg.frequency = CONFIG_SPI_SIMPLELINK_SPI_FREQ_0;
	data->spi_cfg.operation = SPI_OP_MODE_MASTER | SPI_WORD_SET(8) |
			 SPI_TRANSFER_MSB;
	data->spi_cfg.slave = CONFIG_SPI_SIMPLELINK_SPI_SLAVE;

	LOG_INF("Configuring CC3100");

#if defined(CONFIG_SPI_SIMPLELINK_GPIO_SPI_CS)
	data->cs_ctrl.gpio_dev = device_get_binding(
		CONFIG_SPI_FLASH_W25QXXDV_GPIO_SPI_CS_DRV_NAME);
	if (!data->cs_ctrl.gpio_dev) {
		return -ENODEV;
	}

	data->cs_ctrl.gpio_pin = CONFIG_SPI_FLASH_W25QXXDV_GPIO_SPI_CS_PIN;
	data->cs_ctrl.delay = CONFIG_SPI_FLASH_W25QXXDV_GPIO_CS_WAIT_DELAY;

	data->spi_cfg.cs = &data->cs_ctrl;
#endif



	// TODO: move gpio
	struct device *gpio = device_get_binding(CONFIG_SPI_FLASH_W25QXXDV_GPIO_IRQ_DRV_NAME);
	// triggers once:
	//gpio_pin_configure(gpio, CONFIG_SPI_FLASH_W25QXXDV_GPIO_IRQ_PIN, GPIO_DIR_IN | GPIO_INT | GPIO_PUD_NORMAL | GPIO_INT_EDGE | GPIO_INT_DOUBLE_EDGE);
	// never triggers:
	//gpio_pin_configure(gpio, CONFIG_SPI_FLASH_W25QXXDV_GPIO_IRQ_PIN, GPIO_DIR_IN | GPIO_INT | GPIO_PUD_NORMAL | GPIO_INT_EDGE | GPIO_INT_ACTIVE_HIGH);

	gpio_pin_configure(gpio, CONFIG_SPI_FLASH_W25QXXDV_GPIO_IRQ_PIN, GPIO_DIR_IN | GPIO_INT | GPIO_PUD_NORMAL | GPIO_INT_EDGE | GPIO_INT_DOUBLE_EDGE);

	gpio_init_callback(&gpio_cb, button_pressed, BIT(CONFIG_SPI_FLASH_W25QXXDV_GPIO_IRQ_PIN));
	gpio_add_callback(gpio, &gpio_cb);



	tx_buf.len = 0;


	return spi_simplelink_id(dev);
}











#define BUF_SIZE 1400
union
{
	_u8 BsdBuf[BUF_SIZE];
	_u32 demobuf[BUF_SIZE/4];
} uBuf;
/* Application specific status/error codes */
typedef enum{
	DEVICE_NOT_IN_STATION_MODE = -0x7D0,        /* Choosing this number to avoid overlap w/ host-driver's error codes */
	P2P_CONNECTION_FAILED      = DEVICE_NOT_IN_STATION_MODE -1,
	TCP_RECV_ERROR             = P2P_CONNECTION_FAILED - 1,

	STATUS_CODE_MAX = -0xBB8
}e_AppStatusCodes;
#define SL_STOP_TIMEOUT        0xFF





static _i32 initializeAppVariables()
{
	g_Status = 0;
	g_DeviceIp = 0;
	pal_Memset(uBuf.BsdBuf, 0, sizeof(uBuf));
	pal_Memset(g_p2p_dev, 0, MAXIMAL_SSID_LENGTH + 1);

	return SUCCESS;
}

static _i32 configureSimpleLinkToDefaultState()
{
	SlVersionFull   ver = {0};
	_WlanRxFilterOperationCommandBuff_t  RxFilterIdMask = {0};

	_u8           val = 1;
	_u8           configOpt = 0;
	_u8           configLen = 0;
	_u8           power = 0;

	_i32          retVal = -1;
	_i32          mode = -1;

	static uint16_t step_delay = 1;
	mode = sl_Start(0, 0, 0);

	LOG_INF("SimpleLink started (mode: %ld)", mode);

	ASSERT_ON_ERROR(mode);


	/* If the device is not in station-mode, try configuring it in station-mode */
	if (ROLE_STA != mode)
	{
		LOG_DBG("SimpleLink is not in station-mode");
		if (ROLE_AP == mode)
		{
			LOG_DBG("Device is in AP mode");

			/* If the device is in AP mode, we need to wait for this event before doing anything */
			while(!IS_IP_ACQUIRED(g_Status)) {

			// TODO:
				//_SlNonOsMainLoopTask();
				LOG_DBG("Wait for IP");
				k_sleep(1000);



			}
		}
		/* Switch to STA role and restart */
		retVal = sl_WlanSetMode(ROLE_STA);
		ASSERT_ON_ERROR(retVal);

		retVal = sl_Stop(SL_STOP_TIMEOUT);
		ASSERT_ON_ERROR(retVal);

		retVal = sl_Start(0, 0, 0);
		ASSERT_ON_ERROR(retVal);

		/* Check if the device is in station again */
		if (ROLE_STA != retVal)
		{
			/* We don't want to proceed if the device is not coming up in station-mode */
			ASSERT_ON_ERROR(DEVICE_NOT_IN_STATION_MODE);
		}
	}

	get_device:
	/* Get the device's version-information */
	configOpt = SL_DEVICE_GENERAL_VERSION;
	configLen = sizeof(ver);
	retVal = sl_DevGet(SL_DEVICE_GENERAL_CONFIGURATION, &configOpt, &configLen, (_u8 *)(&ver));

	if(retVal != 0)
	{
		LOG_WRN("Something bad happened (code %ld), will reset", retVal);

		sl_Stop(SL_STOP_TIMEOUT);
		sl_Start(0,0,0);
		goto get_device;
	}
	ASSERT_ON_ERROR(retVal);

	/* Set connection policy to Auto + SmartConfig (Device's default connection policy) */
	retVal = sl_WlanPolicySet(SL_POLICY_CONNECTION, SL_CONNECTION_POLICY(1, 0, 0, 0, 1), NULL, 0);
	ASSERT_ON_ERROR(retVal);

	LOG_INF("Wlan policy is set");
	k_sleep(step_delay);
	/* Remove all profiles */
	retVal = sl_WlanProfileDel(0xFF);
	ASSERT_ON_ERROR(retVal);

	LOG_INF("Wlan profiles deleted");
	k_sleep(step_delay);
	/*
	 * Device in station-mode. Disconnect previous connection if any
	 * The function returns 0 if 'Disconnected done', negative number if already disconnected
	 * Wait for 'disconnection' event if 0 is returned, Ignore other return-codes
	 */


	retVal = sl_WlanDisconnect();
	if(retVal != SL_ERROR_WIFI_ALREADY_DISCONNECTED)
	{
		LOG_INF("Wlan disconnected %ld", retVal);
		k_sleep(step_delay);
		if(0 == retVal)
		{

			while(IS_CONNECTED(g_Status)) {
				// TODO: _SlNonOsMainLoopTask();
				LOG_DBG("Wait for connection");
				k_sleep(1000);
			}
		}
	}
	LOG_INF("Wlan disconnected");

	/* Enable DHCP client*/
	retVal = sl_NetCfgSet(SL_IPV4_STA_P2P_CL_DHCP_ENABLE,1,1,&val);

	ASSERT_ON_ERROR(retVal);

	LOG_INF("Net Configuration set");
	k_sleep(step_delay);

	/* Disable scan */
	configOpt = SL_SCAN_POLICY(0);
	retVal = sl_WlanPolicySet(SL_POLICY_SCAN , configOpt, NULL, 0);
	ASSERT_ON_ERROR(retVal);

	LOG_INF("Scan disabled");
	k_sleep(step_delay);

	/* Set Tx power level for station mode
	   Number between 0-15, as dB offset from max power - 0 will set maximum power */
	power = 0;
	retVal = sl_WlanSet(SL_WLAN_CFG_GENERAL_PARAM_ID, WLAN_GENERAL_PARAM_OPT_STA_TX_POWER, 1, (_u8 *)&power);
	ASSERT_ON_ERROR(retVal);

	LOG_INF("Wlan set");
	k_sleep(step_delay);
	/* Set PM policy to normal */
	retVal = sl_WlanPolicySet(SL_POLICY_PM , SL_NORMAL_POLICY, NULL, 0);
	ASSERT_ON_ERROR(retVal);

	LOG_INF("Wlan policy set");
	k_sleep(step_delay);

	/* Unregister mDNS services */
	retVal = sl_NetAppMDNSUnRegisterService(0, 0);
	ASSERT_ON_ERROR(retVal);

	LOG_INF("Net app unregistered mdns");
	k_sleep(step_delay);

	/* Remove  all 64 filters (8*8) */
	pal_Memset(RxFilterIdMask.FilterIdMask, 0xFF, 8);
	retVal = sl_WlanRxFilterSet(SL_REMOVE_RX_FILTER, (_u8 *)&RxFilterIdMask,
								sizeof(_WlanRxFilterOperationCommandBuff_t));
	ASSERT_ON_ERROR(retVal);

	LOG_INF("Wlan filter set");
	k_sleep(step_delay);

	retVal = sl_Stop(SL_STOP_TIMEOUT);
	ASSERT_ON_ERROR(retVal);

	LOG_INF("Device stopped");
	k_sleep(step_delay);

	retVal = initializeAppVariables();
	ASSERT_ON_ERROR(retVal);

	LOG_INF("App variables initialized");
	k_sleep(step_delay);

	return retVal; /* Success */
}

K_SEM_DEFINE(sl_init_sem, 1, 1);

static void simplelink_init(void)
{
	k_sem_take(&sl_init_sem, K_FOREVER);
	long ret_val = configureSimpleLinkToDefaultState();
	LOG_INF("SimpleLink initialized: %lu", ret_val);
}



static int spi_simplelink_init(struct device *dev)
{
	int ret = 0;
	static uint8_t instance_count = 0;


	instance_count++;
	LOG_INF("Initializing CC3100 via SPI %u", instance_count);

	ret = spi_simplelink_configure(dev);

	k_work_q_start(&simplelink_workq,
				   simplelink_workq_stack,
				   K_THREAD_STACK_SIZEOF(simplelink_workq_stack),
				   K_PRIO_COOP(12));

	simplelink_init();

	if (!ret) {
		// TODO: dev->driver_api = &spi_flash_api;
	}

	return ret;
}

static struct spi_simplelink_data spi_simplelink_driver_data;

DEVICE_INIT(spi_simplelink, CONFIG_SPI_SIMPLELINK_DRV_NAME,
			spi_simplelink_init,
			&spi_simplelink_driver_data,
			NULL, POST_KERNEL,
			CONFIG_SPI_SIMPLELINK_PRIORITY);