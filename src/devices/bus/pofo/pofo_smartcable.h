// license:BSD-3-Clause
// copyright-holders:POFOSCAB project
/***************************************************************************

    Atari Portfolio Smart Cable byte-level TCP bridge

    Implements the Smart Cable bit-bang handshake (clock/data toggling,
    no checksum) in C++ as a non-blocking state machine, and exposes it
    over TCP at whole-byte granularity:

        client -> device: {0x01, byte}        send this byte
        device -> client: {0x81, ok, 0}       send result (ok: 1=sent, 0=timeout)
        client -> device: {0x02, 0x00}        receive a byte
        device -> client: {0x82, ok, byte}    received byte (ok: 1=valid,
                                               0=timeout, byte undefined)
        client -> device: {0x03, enabled}     enable/disable idle Port C events
        device -> client: {0x83, 1, value}    event subscription result
        device -> client: {0x84, 0, value}    unsolicited changed Port C value
        client -> device: {0x04, 0x00}        cancel an in-flight byte operation
        device -> client: {0x85, 1, 0}        cancellation complete

    Port C events are opt-in and are emitted only while no SEND/RECEIVE
    byte operation is in flight.  Existing clients never receive them.

    Checksum, block framing, and the file-transfer application protocol
    are left to the client - only the electrical bit-toggling happens
    here.

***************************************************************************/

#ifndef MAME_BUS_POFO_POFO_SMARTCABLE_H
#define MAME_BUS_POFO_POFO_SMARTCABLE_H

#pragma once

#include "exp.h"
#include "machine/i8255.h"

class pofo_smartcable_device : public device_t, public device_portfolio_expansion_slot_interface
{
public:
	pofo_smartcable_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
	virtual ~pofo_smartcable_device();

protected:
	virtual void device_start() override ATTR_COLD;
	virtual void device_stop() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;

	bool pdet() override { return 1; }

private:
	// Each byte is 4 bit pairs, each pair a clock-low half and a
	// clock-high half.
	enum class op_kind : uint8_t
	{
		NONE,
		SEND,
		RECEIVE,
	};

	enum class phase : uint8_t
	{
		IDLE,
		SEND_DELAY,       // fixed startup delay before the first bit pair
		SEND_SETUP_LOW,   // drive data+clock=1, then data alone (clock=0)
		SEND_WAIT_LOW,    // wait for the ROM's clock to go low
		SEND_SETUP_HIGH,  // drive data alone, then data+clock=1
		SEND_WAIT_HIGH,   // wait for the ROM's clock to go high
		RECV_DELAY,       // fixed startup delay before the first RECV_WAIT_LOW
		RECV_WAIT_LOW,    // wait for ROM clock low, latch a data bit
		RECV_WAIT_HIGH,   // wait for ROM clock high, latch a data bit
		DONE,
	};

	uint8_t data_in_r();
	void data_out_w(uint8_t data);
	uint8_t status_in_r();
	void status_out_w(uint8_t data);

	TIMER_CALLBACK_MEMBER(poll_socket);
	TIMER_CALLBACK_MEMBER(step);

	void open_listen_socket();
	void accept_pending();
	void drain_socket();
	void close_client();
	void send_reply(uint8_t reg, uint8_t ok, uint8_t value);

	void start_send(uint8_t byte_to_send);
	void start_receive();
	void finish_op(bool ok, uint8_t received_byte);

	required_device<i8255_device> m_ppi;

	emu_timer *m_poll_timer;
	emu_timer *m_step_timer;

	// ROM reads port A (outData bit0 / outClock bit1) and writes port C
	// (inData bit0 / inClock bit1, from our point of view). m_port_c_in
	// is kept up to date by status_out_w() whenever the ROM writes it;
	// step() only ever reads this cached value, never m_ppi directly.
	uint8_t m_port_a_out;
	uint8_t m_port_c_in;
	bool m_port_c_events;

	// Deadline for the SEND_DELAY/RECV_DELAY startup pause. Waiting on an
	// absolute clock LEVEL (not an edge relative to some captured
	// baseline) is what lets send/receive operations chain smoothly
	// back-to-back inside one block transfer - each op naturally starts
	// with the ROM sitting at the level the previous op left it in. But
	// a standalone receive_byte() call issued after a client-side pause
	// (no preceding op in this device) can race a clock line already
	// sitting at the level we're about to wait for, without it ever
	// having freshly transitioned for THIS call - a short fixed delay
	// before the first wait gives the ROM time to actually move off
	// that level first.
	attotime m_delay_deadline;

	op_kind m_op;
	phase m_phase;
	uint8_t m_send_data;    // remaining bits to send, MSB-first, shifted left each step
	uint8_t m_recv_data;    // bits received so far, shifted in LSB->MSB per step per protocol
	int m_bit_index;        // 0-3, which of the 4 bit-pairs we're on
	attotime m_op_deadline;

	int m_listen_fd;
	int m_client_fd;

	uint8_t m_recv_partial;
	bool m_recv_have_partial;

	uint16_t m_tcp_port;
};

DECLARE_DEVICE_TYPE(POFO_SMARTCABLE, pofo_smartcable_device)

#endif // MAME_BUS_POFO_POFO_SMARTCABLE_H
