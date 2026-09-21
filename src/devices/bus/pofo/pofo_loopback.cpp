// license:BSD-3-Clause
// copyright-holders:POFOSCAB project
#include "emu.h"
#include "pofo_loopback.h"

namespace {
#define M82C55A_TAG "u1"
}

DEFINE_DEVICE_TYPE(POFO_LOOPBACK, pofo_loopback_device, "pofo_loopback", "POFOSCAB Raw I/O Loopback Test")

pofo_loopback_device::pofo_loopback_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, POFO_LOOPBACK, tag, owner, clock)
	, device_portfolio_expansion_slot_interface(mconfig, *this)
	, m_ppi(*this, M82C55A_TAG)
	, m_port_c_in(0xff)
{
}

pofo_loopback_device::~pofo_loopback_device()
{
}

void pofo_loopback_device::device_add_mconfig(machine_config &config)
{
	I8255A(config, m_ppi);
	m_ppi->in_pa_callback().set(FUNC(pofo_loopback_device::data_in_r));
	m_ppi->out_pa_callback().set(FUNC(pofo_loopback_device::data_out_w));
	m_ppi->in_pc_callback().set(FUNC(pofo_loopback_device::status_in_r));
	m_ppi->out_pc_callback().set(FUNC(pofo_loopback_device::status_out_w));
}

void pofo_loopback_device::device_start()
{
	save_item(NAME(m_port_c_in));
}

void pofo_loopback_device::device_reset()
{
	m_ppi->reset();
}

// Same address decode as pofo_bridge's pre-refactor variant / hpc101.cpp.
uint8_t pofo_loopback_device::nrdi_r(offs_t offset, uint8_t data, bool iom, bool bcom, bool ncc1)
{
	if (!bcom)
	{
		if ((offset & 0x0f) == 0x0f)
			data = 0x02;

		if ((offset & 0x0c) == 0x08)
			data = m_ppi->read(offset & 0x03);
	}

	return data;
}

void pofo_loopback_device::nwri_w(offs_t offset, uint8_t data, bool iom, bool bcom, bool ncc1)
{
	if (!bcom)
	{
		if ((offset & 0x0c) == 0x08)
			m_ppi->write(offset & 0x03, data);
	}
}

uint8_t pofo_loopback_device::data_in_r()
{
	// Port A is not wired as an input by the real Smart Cable client
	// side in this test - the loop only cares about A(out) -> C(in).
	return 0xff;
}

// The physical loop: bit0 (outData) and bit1 (outClock) of whatever the
// Portfolio just wrote to port A land straight on bit0 (inData) and
// bit1 (inClock) of port C, exactly like a real shorted DB25 2/3->12/13
// pair - no handshake, no delay, no socket.
void pofo_loopback_device::data_out_w(uint8_t data)
{
	m_port_c_in = (m_port_c_in & 0xfc) | (data & 0x03);
}

uint8_t pofo_loopback_device::status_in_r()
{
	return m_port_c_in;
}

void pofo_loopback_device::status_out_w(uint8_t data)
{
}
