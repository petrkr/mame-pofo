// license:BSD-3-Clause
// copyright-holders:POFOSCAB project
/***************************************************************************

    Atari Portfolio raw I/O loopback test card

    Wires port A bit 0 (outData) and bit 1 (outClock) straight back onto
    port C bit 0 (inData) and bit 1 (inClock) - exactly the same two
    pins a real Smart Cable loops on a physical wire (DB25 pin 2/3
    shorted to pin 12/13), but inside the emulated i8255 with no
    handshake protocol, no TCP socket, and no external process at all.

    Exists purely to measure raw emulated I/O throughput (OUT/IN cycle
    rate) independent of the Smart Cable wire protocol's handshake
    timing, for comparison against the same measurement on a real
    Portfolio with the equivalent physical pins shorted.

***************************************************************************/

#ifndef MAME_BUS_POFO_POFO_LOOPBACK_H
#define MAME_BUS_POFO_POFO_LOOPBACK_H

#pragma once

#include "exp.h"
#include "machine/i8255.h"

class pofo_loopback_device : public device_t, public device_portfolio_expansion_slot_interface
{
public:
	pofo_loopback_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
	virtual ~pofo_loopback_device();

protected:
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;

	bool pdet() override { return 1; }

	virtual uint8_t nrdi_r(offs_t offset, uint8_t data, bool iom, bool bcom, bool ncc1) override;
	virtual void nwri_w(offs_t offset, uint8_t data, bool iom, bool bcom, bool ncc1) override;

private:
	uint8_t data_in_r();
	void data_out_w(uint8_t data);
	uint8_t status_in_r();
	void status_out_w(uint8_t data);

	required_device<i8255_device> m_ppi;

	uint8_t m_port_c_in;
};

DECLARE_DEVICE_TYPE(POFO_LOOPBACK, pofo_loopback_device)

#endif // MAME_BUS_POFO_POFO_LOOPBACK_H
