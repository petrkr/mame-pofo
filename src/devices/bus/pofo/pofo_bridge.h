// license:BSD-3-Clause
// copyright-holders:POFOSCAB project
/***************************************************************************

    Atari Portfolio test-harness bridge expansion card

    Plugs into the Portfolio expansion port in place of a real HPC-101
    (or any other `exp` card) and forwards its i8255 PPI's port A (data)
    and port C (status) lines over a plain TCP socket, so an external
    test harness can drive them directly - byte at a time, not bit at a
    time - without going through the Centronics bus abstraction at all.

    Unlike a real HPC-101 (which fixes port A as output/port C as input,
    matching a one-directional printer), this card wires BOTH directions
    on both ports (in_pa/out_pa, in_pc/out_pc) - real Mode 0 8255
    hardware allows software to choose either direction on the same
    physical pins via the control word, and the actual protocol this
    bridges (see POFOSCAB's PROTOCOL.md / PortfolioLink.cpp) needs that:
    the Portfolio ROM drives DATA0/DATA1 one way at times and reads
    PE/SELECT the other way, depending on which half of the handshake is
    active.

    Deliberately not a Centronics peripheral: earlier attempts wiring
    this behind a `centronics_device` slot ran into upstream MAME never
    wiring HPC-101's in_pa_callback/out_pc_callback (see git history) -
    going straight to the expansion slot interface avoids all of that.

***************************************************************************/

#ifndef MAME_BUS_POFO_POFO_BRIDGE_H
#define MAME_BUS_POFO_POFO_BRIDGE_H

#pragma once

#include "exp.h"
#include "machine/i8255.h"

class pofo_bridge_device : public device_t, public device_portfolio_expansion_slot_interface
{
public:
	pofo_bridge_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
	virtual ~pofo_bridge_device();

protected:
	virtual void device_start() override ATTR_COLD;
	virtual void device_stop() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;

	// device_portfolio_expansion_slot_interface overrides - pdet()
	// defaults to 0 (no card present); without this override the
	// Portfolio ROM never even sees a card plugged into the expansion
	// port, regardless of what nrdi_r/nwri_w do (observed as a
	// "Communication error" trying to enter the File Transfer Server -
	// same override real HPC-101 makes, see hpc101.cpp).
	bool pdet() override { return 1; }

private:
	// Wire protocol: {register_id, value} byte pairs, either direction.
	// register_id selects which i8255 port the byte applies to - PORT_A
	// (data0-7) and PORT_C (status, only bits 0-1 = PE/SELECT are wired
	// on the real cable, see PROTOCOL.md, but the full byte is sent for
	// simplicity/future extension).
	enum register_id : uint8_t
	{
		REG_PORT_A = 0,
		REG_PORT_C = 1,
	};

	uint8_t data_in_r();
	void data_out_w(uint8_t data);
	uint8_t status_in_r();
	void status_out_w(uint8_t data);

	TIMER_CALLBACK_MEMBER(poll_socket);

	void open_listen_socket();
	void accept_pending();
	void drain_socket();
	void apply_incoming(uint8_t reg, uint8_t value);
	void send_register(uint8_t reg, uint8_t value);
	void close_client();

	required_device<i8255_device> m_ppi;
	memory_passthrough_handler m_ppi_tap;

	emu_timer *m_poll_timer;

	// last value the bridge told us to present on port A/C when the
	// Portfolio's i8255 is configured to read that port (in_pa_r/in_pc_r)
	uint8_t m_port_a_in;
	uint8_t m_port_c_in;

	int m_listen_fd;
	int m_client_fd;

	// TCP is a byte stream - a single recv() is not guaranteed to return
	// a whole {register_id, value} pair at once, so partial reads are
	// reassembled here rather than discarded.
	uint8_t m_recv_partial;
	bool m_recv_have_partial;

	uint16_t m_tcp_port;
};

DECLARE_DEVICE_TYPE(POFO_BRIDGE, pofo_bridge_device)

#endif // MAME_BUS_POFO_POFO_BRIDGE_H
