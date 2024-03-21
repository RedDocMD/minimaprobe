/**
 * Copyright (c) 2023 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "tusb_edpt_handler.h"
#include "DAP.h"

static uint8_t itf_num;
static uint8_t _rhport;

volatile uint32_t _resp_len;

static uint8_t _out_ep_addr;
static uint8_t _in_ep_addr;

static buffer_t USBRequestBuffer;
static buffer_t USBResponseBuffer;

static uint8_t DAPRequestBuffer[DAP_PACKET_SIZE];
static uint8_t DAPResponseBuffer[DAP_PACKET_SIZE];

#define WR_IDX(x) (x.wptr % DAP_PACKET_COUNT)
#define RD_IDX(x) (x.rptr % DAP_PACKET_COUNT)

#define WR_SLOT_PTR(x) &(x.data[WR_IDX(x)][0])
#define RD_SLOT_PTR(x) &(x.data[RD_IDX(x)][0])

bool buffer_full(buffer_t *buffer)
{
	return ((buffer->wptr + 1) % DAP_PACKET_COUNT == buffer->rptr);
}

bool buffer_empty(buffer_t *buffer)
{
	return (buffer->wptr == buffer->rptr);
}

void dap_edpt_init(void) {

}

void dap_edpt_reset(uint8_t __unused rhport)
{
	itf_num = 0;
}

char * dap_cmd_string[] = {
	[ID_DAP_Info               ] = "DAP_Info",
	[ID_DAP_HostStatus         ] = "DAP_HostStatus",
	[ID_DAP_Connect            ] = "DAP_Connect",
	[ID_DAP_Disconnect         ] = "DAP_Disconnect",
	[ID_DAP_TransferConfigure  ] = "DAP_TransferConfigure",
	[ID_DAP_Transfer           ] = "DAP_Transfer",
	[ID_DAP_TransferBlock      ] = "DAP_TransferBlock",
	[ID_DAP_TransferAbort      ] = "DAP_TransferAbort",
	[ID_DAP_WriteABORT         ] = "DAP_WriteABORT",
	[ID_DAP_Delay              ] = "DAP_Delay",
	[ID_DAP_ResetTarget        ] = "DAP_ResetTarget",
	[ID_DAP_SWJ_Pins           ] = "DAP_SWJ_Pins",
	[ID_DAP_SWJ_Clock          ] = "DAP_SWJ_Clock",
	[ID_DAP_SWJ_Sequence       ] = "DAP_SWJ_Sequence",
	[ID_DAP_SWD_Configure      ] = "DAP_SWD_Configure",
	[ID_DAP_SWD_Sequence       ] = "DAP_SWD_Sequence",
	[ID_DAP_JTAG_Sequence      ] = "DAP_JTAG_Sequence",
	[ID_DAP_JTAG_Configure     ] = "DAP_JTAG_Configure",
	[ID_DAP_JTAG_IDCODE        ] = "DAP_JTAG_IDCODE",
	[ID_DAP_SWO_Transport      ] = "DAP_SWO_Transport",
	[ID_DAP_SWO_Mode           ] = "DAP_SWO_Mode",
	[ID_DAP_SWO_Baudrate       ] = "DAP_SWO_Baudrate",
	[ID_DAP_SWO_Control        ] = "DAP_SWO_Control",
	[ID_DAP_SWO_Status         ] = "DAP_SWO_Status",
	[ID_DAP_SWO_ExtendedStatus ] = "DAP_SWO_ExtendedStatus",
	[ID_DAP_SWO_Data           ] = "DAP_SWO_Data",
	[ID_DAP_QueueCommands      ] = "DAP_QueueCommands",
	[ID_DAP_ExecuteCommands    ] = "DAP_ExecuteCommands",
};


uint16_t dap_edpt_open(uint8_t __unused rhport, tusb_desc_interface_t const *itf_desc, uint16_t max_len)
{

	TU_VERIFY(TUSB_CLASS_VENDOR_SPECIFIC == itf_desc->bInterfaceClass &&
			DAP_INTERFACE_SUBCLASS == itf_desc->bInterfaceSubClass &&
			DAP_INTERFACE_PROTOCOL == itf_desc->bInterfaceProtocol, 0);

	//  Initialise circular buffer indices
	USBResponseBuffer.wptr = 0;
	USBResponseBuffer.rptr = 0;
	USBRequestBuffer.wptr = 0;
	USBRequestBuffer.rptr = 0;

	// Initialse full/empty flags
	USBResponseBuffer.wasFull = false;
	USBResponseBuffer.wasEmpty = true;
	USBRequestBuffer.wasFull = false;
	USBRequestBuffer.wasEmpty = true;

	uint16_t const drv_len = sizeof(tusb_desc_interface_t) + (itf_desc->bNumEndpoints * sizeof(tusb_desc_endpoint_t));
	TU_VERIFY(max_len >= drv_len, 0);
	itf_num = itf_desc->bInterfaceNumber;

	// Initialising the OUT endpoint

	tusb_desc_endpoint_t *edpt_desc = (tusb_desc_endpoint_t *) (itf_desc + 1);
	uint8_t ep_addr = edpt_desc->bEndpointAddress;

	_out_ep_addr = ep_addr;

	// The OUT endpoint requires a call to usbd_edpt_xfer to initialise the endpoint, giving tinyUSB a buffer to consume when a transfer occurs at the endpoint
	usbd_edpt_open(rhport, edpt_desc);
	usbd_edpt_xfer(rhport, ep_addr, WR_SLOT_PTR(USBRequestBuffer), DAP_PACKET_SIZE);

	// Initiliasing the IN endpoint

	edpt_desc++;
	ep_addr = edpt_desc->bEndpointAddress;

	_in_ep_addr = ep_addr;

	// The IN endpoint doesn't need a transfer to initialise it, as this will be done by the main loop of dap_thread
	usbd_edpt_open(rhport, edpt_desc);

	return drv_len;

}

bool dap_edpt_control_xfer_cb(uint8_t __unused rhport, uint8_t stage, tusb_control_request_t const *request)
{
	return false;
}

// Manage USBResponseBuffer (request) write and USBRequestBuffer (response) read indices
bool dap_edpt_xfer_cb(uint8_t __unused rhport, uint8_t ep_addr, xfer_result_t result, uint32_t xferred_bytes)
{
	const uint8_t ep_dir = tu_edpt_dir(ep_addr);

	if(ep_dir == TUSB_DIR_IN)
	{
		if(xferred_bytes >= 0u && xferred_bytes <= DAP_PACKET_SIZE)
		{
			USBResponseBuffer.rptr++;

			// This checks that the buffer was not empty in DAP thread, which means the next buffer was not queued up for the in endpoint callback
			// So, queue up the buffer at the new read index, since we expect read to catch up to write at this point.
			// It is possible for the read index to be multiple spaces behind the write index (if the USB callbacks are lagging behind dap thread),
			// so we account for this by only setting wasEmpty to true if the next callback will empty the buffer
			if(!USBResponseBuffer.wasEmpty)
			{
				usbd_edpt_xfer(rhport, ep_addr, RD_SLOT_PTR(USBResponseBuffer), (uint16_t) _resp_len);
				USBResponseBuffer.wasEmpty = (USBResponseBuffer.rptr + 1) == USBResponseBuffer.wptr;
			}

			//  Wake up DAP thread after processing the callback
			vTaskResume(dap_taskhandle);
			return true;
		}

		return false;

	} else if(ep_dir == TUSB_DIR_OUT)    {

		if(xferred_bytes >= 0u && xferred_bytes <= DAP_PACKET_SIZE)
		{
			// Only queue the next buffer in the out callback if the buffer is not full
			// If full, we set the wasFull flag, which will be checked by dap thread
			if(!buffer_full(&USBRequestBuffer))
			{
				USBRequestBuffer.wptr++;
				usbd_edpt_xfer(rhport, ep_addr, WR_SLOT_PTR(USBRequestBuffer), DAP_PACKET_SIZE);
				USBRequestBuffer.wasFull = false;
			}
			else {
				USBRequestBuffer.wasFull = true;
			}

			//  Wake up DAP thread after processing the callback
			vTaskResume(dap_taskhandle);
			return true;
		}

		return false;
	}
	else return false;
}

static void render_dap_transfer_req(const uint8_t *buf)
{
	int req = buf[3], cnt = buf[2];
	const uint32_t *wordptr;
	probe_info("DAP_Transfer => Idx = %d, Cnt = %d, ", buf[1], cnt);
	if (req & 0x1)
		probe_info("Access port, ");
	else
		probe_info("Debug port, ");
	if (req & 0x2)
		probe_info("Write, ");
	else
	 	probe_info("Read, ");
	probe_info("A[3:2] = %d", (req & 0xC) >> 2);
	if (req & 0x80)
		probe_info(", Timestamp");
	else
	 	probe_info(", No timestamp");
	if (cnt > 0) {
		wordptr = (const uint32_t *)(buf + 4);
		probe_info(", Data = ");
		for (int i = 0; i < cnt; i++)
			probe_info(" 0x%08lx", wordptr[i]);
	}
}

static void render_dap_transfer_resp(const uint8_t *buf)
{
	int resp = buf[2], cnt = buf[1];
	const uint32_t *wordptr;
	probe_info("DAP_Transfer => Cnt = %d, ", cnt);
	int ack = resp & 0x7;
	if (ack == 1)
		probe_info("OK (or FAULT for JTAG)");
	else if (ack == 2)
		probe_info("WAIT");
	else if (ack == 4)
		probe_info("FAULT");
	else if (ack == 7)
		probe_info("NO_ACK");
	else
	 	probe_info("Weird ACK = %d", ack);
	if (resp & 0x8)
		probe_info(", Protocol Error (SWD)");
	if (resp & 0x10)
		probe_info(", Value mismatch");
	probe_info(", Timestamp = %ld", *(const uint32_t *)&buf[3]);
	if (cnt > 0) {
		probe_info(", Data = ");
		wordptr = (const uint32_t *)(buf + 7);
		for (int i = 0; i < cnt; i++)
			probe_info(" 0x%08lx", wordptr[i]);
	}
}	

static void render_dap_req(const uint8_t *buf)
{
	uint8_t cmd;

	probe_info("DAP Request: ");
	cmd = buf[0];
	switch (cmd) {
	case ID_DAP_Connect:
		if (buf[1] == 0)
			probe_info("DAP_Connect => Default");
		else if (buf[1] == 1)
			probe_info("DAP_Connect => SWD");
		else if (buf[1] == 2)
			probe_info("DAP_Connect => JTAG");
		break;
	case ID_DAP_SWJ_Clock:
		probe_info("DAP_SWJ_Clock => %ld Hz", *(uint32_t *)&buf[1]);
		break;
	case ID_DAP_SWJ_Sequence:
		probe_info("DAP_SWJ_Sequence => %d bits", buf[1]);
		break;
	case ID_DAP_Transfer:
		render_dap_transfer_req(buf);
		break;
	}
	probe_info("\n");
}

static void render_dap_resp(const uint8_t *buf)
{
	uint8_t cmd;

	cmd = buf[0];
	probe_info("DAP Response: ");
	switch (cmd) {
	case ID_DAP_Connect:
		if (buf[1] == 0)
			probe_info("DAP_Connect => Init failed");
		else if (buf[1] == 1)
			probe_info("DAP_Connect => Init SWD");
		else if (buf[1] == 2)
			probe_info("DAP_Connect => Init JTAG");
		break;
	case ID_DAP_SWJ_Clock:
		if (buf[1] == 0)
			probe_info("DAP_SWJ_Clock => OK");
		else if (buf[1] == 0xFF)
			probe_info("DAP_SWJ_Clock => FAIL");
		break;
	case ID_DAP_SWJ_Sequence:
		if (buf[1] == 0)
			probe_info("DAP_SWJ_Sequence => OK");
		else if (buf[1] == 0xFF)
			probe_info("DAP_SWJ_Sequence => FAIL");
		break;
	case ID_DAP_Transfer:
		render_dap_transfer_resp(buf);
		break;
	}
	probe_info("\n");
}

void dap_thread(void *ptr)
{
	uint32_t n;
	do
	{
		while(USBRequestBuffer.rptr != USBRequestBuffer.wptr)
		{
			/*
			 * Atomic command support - buffer QueueCommands, but don't process them
			 * until a non-QueueCommands packet is seen.
			 */
			n = USBRequestBuffer.rptr;
			while (USBRequestBuffer.data[n % DAP_PACKET_COUNT][0] == ID_DAP_QueueCommands) {
				probe_info("%lu %lu DAP queued cmd %s len %02x\n",
					       USBRequestBuffer.wptr, USBRequestBuffer.rptr,
					       dap_cmd_string[USBRequestBuffer.data[n % DAP_PACKET_COUNT][0]], USBRequestBuffer.data[n % DAP_PACKET_COUNT][1]);
				USBRequestBuffer.data[n % DAP_PACKET_COUNT][0] = ID_DAP_ExecuteCommands;
				n++;
				while (n == USBRequestBuffer.wptr) {
					/* Need yield in a loop here, as IN callbacks will also wake the thread */
					probe_info("DAP wait\n");
					vTaskSuspend(dap_taskhandle);
				}
			}
			// Read a single packet from the USB buffer into the DAP Request buffer
			memcpy(DAPRequestBuffer, RD_SLOT_PTR(USBRequestBuffer), DAP_PACKET_SIZE);
			probe_info("%lu %lu DAP cmd %s len %02x\n",
				       USBRequestBuffer.wptr, USBRequestBuffer.rptr,
				       dap_cmd_string[DAPRequestBuffer[0]], DAPRequestBuffer[1]);
			USBRequestBuffer.rptr++;

			// If the buffer was full in the out callback, we need to queue up another buffer for the endpoint to consume, now that we know there is space in the buffer.
			if(USBRequestBuffer.wasFull)
			{
				vTaskSuspendAll(); // Suspend the scheduler to safely update the write index
				USBRequestBuffer.wptr++;
				usbd_edpt_xfer(_rhport, _out_ep_addr, WR_SLOT_PTR(USBRequestBuffer), DAP_PACKET_SIZE);
				USBRequestBuffer.wasFull = false;
				xTaskResumeAll();
			}

			render_dap_req(DAPRequestBuffer);
			_resp_len = DAP_ExecuteCommand(DAPRequestBuffer, DAPResponseBuffer);
			render_dap_resp(DAPResponseBuffer);
			probe_info("%lu %lu DAP resp %s\n",
					USBResponseBuffer.wptr, USBResponseBuffer.rptr,
					dap_cmd_string[DAPResponseBuffer[0]]);


			//  Suspend the scheduler to avoid stale values/race conditions between threads
			vTaskSuspendAll();

			if(buffer_empty(&USBResponseBuffer))
			{
				memcpy(WR_SLOT_PTR(USBResponseBuffer), DAPResponseBuffer, (uint16_t) _resp_len);
				USBResponseBuffer.wptr++;

				usbd_edpt_xfer(_rhport, _in_ep_addr, RD_SLOT_PTR(USBResponseBuffer), (uint16_t) _resp_len);
			} else {

				memcpy(WR_SLOT_PTR(USBResponseBuffer), DAPResponseBuffer, (uint16_t) _resp_len);
				USBResponseBuffer.wptr++;

				// The In callback needs to check this flag to know when to queue up the next buffer.
				USBResponseBuffer.wasEmpty = false;
			}
			xTaskResumeAll();
		}

		// Suspend DAP thread until it is awoken by a USB thread callback
		vTaskSuspend(dap_taskhandle);

	} while (1);

}

usbd_class_driver_t const _dap_edpt_driver =
{
		.init = dap_edpt_init,
		.reset = dap_edpt_reset,
		.open = dap_edpt_open,
		.control_xfer_cb = dap_edpt_control_xfer_cb,
		.xfer_cb = dap_edpt_xfer_cb,
		.sof = NULL,
#if CFG_TUSB_DEBUG >= 2
		.name = "DAP ENDPOINT"
#endif
};

// Add the custom driver to the tinyUSB stack
usbd_class_driver_t const *usbd_app_driver_get_cb(uint8_t *driver_count)
{
	*driver_count = 1;
	return &_dap_edpt_driver;
}

