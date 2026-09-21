// license:BSD-3-Clause
// copyright-holders:POFOSCAB project
#include "emu.h"
#include "pofo_smartcable.h"

#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>

namespace {
#define M82C55A_TAG "u1"

// Fine enough to not add its own latency to the bit-bang handshake.
constexpr attotime POLL_INTERVAL = attotime::from_usec(20);
constexpr uint16_t DEFAULT_TCP_PORT = 9997;
constexpr attotime CLOCK_TIMEOUT = attotime::from_seconds(2);

// Wire protocol register ids (see header comment).
enum wire_reg : uint8_t
{
	REQ_SEND    = 0x01,
	REQ_RECEIVE = 0x02,
	REPLY_SEND  = 0x81,
	REPLY_RECV  = 0x82,
};
}

DEFINE_DEVICE_TYPE(POFO_SMARTCABLE, pofo_smartcable_device, "smartcable", "Atari Portfolio Smart Cable Bridge")

pofo_smartcable_device::pofo_smartcable_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, POFO_SMARTCABLE, tag, owner, clock)
	, device_portfolio_expansion_slot_interface(mconfig, *this)
	, m_ppi(*this, M82C55A_TAG)
	, m_poll_timer(nullptr)
	, m_step_timer(nullptr)
	, m_port_a_out(0)
	, m_port_c_in(0xff)
	, m_delay_deadline(attotime::zero)
	, m_op(op_kind::NONE)
	, m_phase(phase::IDLE)
	, m_send_data(0)
	, m_recv_data(0)
	, m_bit_index(0)
	, m_listen_fd(-1)
	, m_client_fd(-1)
	, m_recv_partial(0)
	, m_recv_have_partial(false)
	, m_tcp_port(DEFAULT_TCP_PORT)
{
}

pofo_smartcable_device::~pofo_smartcable_device()
{
}

void pofo_smartcable_device::device_add_mconfig(machine_config &config)
{
	I8255A(config, m_ppi);
	m_ppi->in_pa_callback().set(FUNC(pofo_smartcable_device::data_in_r));
	m_ppi->out_pa_callback().set(FUNC(pofo_smartcable_device::data_out_w));
	m_ppi->in_pc_callback().set(FUNC(pofo_smartcable_device::status_in_r));
	m_ppi->out_pc_callback().set(FUNC(pofo_smartcable_device::status_out_w));
}

void pofo_smartcable_device::device_start()
{
	save_item(NAME(m_port_a_out));
	save_item(NAME(m_port_c_in));

	m_slot->iospace().install_read_tap(0x807f, 0x807f, "pofo_smartcable_id",
		[] (offs_t offset, u8 &data, u8) { data = 0x02; });

	memory_passthrough_handler ppi_tap;
	ppi_tap = m_slot->iospace().install_readwrite_tap(0x8078, 0x807b, "pofo_smartcable_ppi",
		[this] (offs_t offset, u8 &data, u8) { data = m_ppi->read(offset & 0x03); },
		[this] (offs_t offset, u8 &data, u8) { m_ppi->write(offset & 0x03, data); }, &ppi_tap);

	open_listen_socket();

	m_poll_timer = timer_alloc(FUNC(pofo_smartcable_device::poll_socket), this);
	m_poll_timer->adjust(POLL_INTERVAL, 0, POLL_INTERVAL);

	m_step_timer = timer_alloc(FUNC(pofo_smartcable_device::step), this);
}

void pofo_smartcable_device::device_stop()
{
	close_client();
	if (m_listen_fd >= 0)
	{
		close(m_listen_fd);
		m_listen_fd = -1;
	}
}

void pofo_smartcable_device::device_reset()
{
	m_ppi->reset();
}

// Port A: the ROM reads this to sample our outData/outClock bits.
// in_pa_callback fires when the ROM performs an IN on port A - we hand
// back whatever step() last wrote into m_port_a_out directly (not
// through m_ppi - a device on the peripheral side of an i8255 sets its
// output via the callback return value, not by calling write() on its
// own PPI, which is reserved for the CPU/ROM side driving the bus).
uint8_t pofo_smartcable_device::data_in_r()
{
	return m_port_a_out;
}

// Not used in this device's role: the ROM never writes port A.
void pofo_smartcable_device::data_out_w(uint8_t data)
{
}

// Not used in this device's role: we never read port C as an input on
// our own i8255 latch.
uint8_t pofo_smartcable_device::status_in_r()
{
	return 0xff;
}

// Fires when the ROM performs an OUT to port C - this is the ROM
// telling us its inData/inClock state. step() reads m_port_c_in
// directly (never m_ppi) to sample the ROM's side of the handshake.
void pofo_smartcable_device::status_out_w(uint8_t data)
{
	m_port_c_in = data;
}

void pofo_smartcable_device::open_listen_socket()
{
	m_listen_fd = socket(AF_INET, SOCK_STREAM, 0);
	if (m_listen_fd < 0)
	{
		logerror("pofo_smartcable: socket() failed\n");
		return;
	}

	int reuse = 1;
	setsockopt(m_listen_fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

	sockaddr_in addr{};
	addr.sin_family = AF_INET;
	addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
	addr.sin_port = htons(m_tcp_port);

	if (bind(m_listen_fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0)
	{
		logerror("pofo_smartcable: bind() to port %u failed\n", m_tcp_port);
		close(m_listen_fd);
		m_listen_fd = -1;
		return;
	}

	if (listen(m_listen_fd, 1) < 0)
	{
		logerror("pofo_smartcable: listen() failed\n");
		close(m_listen_fd);
		m_listen_fd = -1;
		return;
	}

	fcntl(m_listen_fd, F_SETFL, O_NONBLOCK);
	logerror("pofo_smartcable: listening on 127.0.0.1:%u\n", m_tcp_port);
}

void pofo_smartcable_device::accept_pending()
{
	if (m_listen_fd < 0 || m_client_fd >= 0)
		return;

	int fd = accept(m_listen_fd, nullptr, nullptr);
	if (fd >= 0)
	{
		fcntl(fd, F_SETFL, O_NONBLOCK);
		int nodelay = 1;
		setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));
		m_client_fd = fd;
		logerror("pofo_smartcable: client connected\n");
	}
}

void pofo_smartcable_device::close_client()
{
	if (m_client_fd >= 0)
	{
		close(m_client_fd);
		m_client_fd = -1;
	}

	// A client can disconnect mid-request - clear any partially-received
	// request byte so the next client's first byte isn't misread as the
	// second half of a stale {reg, value} pair.
	m_recv_have_partial = false;

	// Also abandon any in-flight op - a new client (e.g. the bridge
	// reconnecting after this one dropped) must start from a clean
	// IDLE state, not have step()'s timer still ticking down toward a
	// finish_op() that would send_reply() into a *different* client's
	// reply stream (or into a closed fd - send_reply() itself no-ops
	// when m_client_fd < 0, but by then the new client may already have
	// connected and set it).
	m_op = op_kind::NONE;
	m_phase = phase::IDLE;
	if (m_step_timer)
		m_step_timer->adjust(attotime::never);
}

TIMER_CALLBACK_MEMBER(pofo_smartcable_device::poll_socket)
{
	accept_pending();
	drain_socket();
}

void pofo_smartcable_device::drain_socket()
{
	if (m_client_fd < 0)
		return;

	uint8_t byte;
	for (;;)
	{
		ssize_t n = recv(m_client_fd, &byte, 1, 0);
		if (n == 1)
		{
			if (!m_recv_have_partial)
			{
				m_recv_partial = byte;
				m_recv_have_partial = true;
			}
			else
			{
				uint8_t reg = m_recv_partial;
				uint8_t value = byte;
				m_recv_have_partial = false;

				// A new request while an op is still in flight means the
				// client already gave up on it (its own timeout fired)
				// and moved on - normal after a missed/garbled handshake
				// retry. Silently abandon the stale op instead of letting
				// it run to its own deadline: if left alone, it would
				// eventually send a reply the client no longer expects,
				// permanently shifting the reply stream out of sync with
				// requests for the rest of the TCP connection (the client
				// has no way to tell that reply apart from the one for
				// the request it just sent). Abandoning it here never
				// sends a reply for it, so nothing is left to misalign
				// later replies - only ever start_send()/start_receive()
				// scheduled below.
				if (m_op != op_kind::NONE)
				{
					m_op = op_kind::NONE;
					m_phase = phase::IDLE;
					m_step_timer->adjust(attotime::never);
				}

				if (reg == REQ_SEND)
					start_send(value);
				else if (reg == REQ_RECEIVE)
					start_receive();
			}
			continue;
		}
		if (n == 0)
		{
			logerror("pofo_smartcable: client disconnected\n");
			close_client();
		}
		break;
	}
}

void pofo_smartcable_device::send_reply(uint8_t reg, uint8_t ok, uint8_t value)
{
	if (m_client_fd < 0)
		return;

	uint8_t buf[3] = { reg, ok, value };
	send(m_client_fd, buf, sizeof(buf), MSG_NOSIGNAL);
}

void pofo_smartcable_device::start_send(uint8_t byte_to_send)
{
	m_op = op_kind::SEND;
	m_send_data = byte_to_send;
	m_bit_index = 0;
	m_op_deadline = machine().time() + CLOCK_TIMEOUT;
	m_delay_deadline = machine().time() + attotime::from_usec(250);
	m_phase = phase::SEND_DELAY;
	m_step_timer->adjust(POLL_INTERVAL, 0, POLL_INTERVAL);
}

void pofo_smartcable_device::start_receive()
{
	m_op = op_kind::RECEIVE;
	m_recv_data = 0;
	m_bit_index = 0;
	m_op_deadline = machine().time() + CLOCK_TIMEOUT;
	m_delay_deadline = machine().time() + attotime::from_usec(250);
	m_phase = phase::RECV_DELAY;
	m_step_timer->adjust(POLL_INTERVAL, 0, POLL_INTERVAL);
}

void pofo_smartcable_device::finish_op(bool ok, uint8_t received_byte)
{
	if (m_op == op_kind::SEND)
		send_reply(REPLY_SEND, ok ? 1 : 0, 0);
	else if (m_op == op_kind::RECEIVE)
		send_reply(REPLY_RECV, ok ? 1 : 0, received_byte);

	m_op = op_kind::NONE;
	m_phase = phase::IDLE;
	m_step_timer->adjust(attotime::never);
}

// Each call handles one half-edge of one bit-pair, then returns - the
// POLL_INTERVAL emu_timer drives it forward. m_port_c_in (bit0=inData,
// bit1=inClock) is kept current by status_out_w() whenever the ROM
// writes port C - step() only ever reads that cached value.
TIMER_CALLBACK_MEMBER(pofo_smartcable_device::step)
{
	if (machine().time() >= m_op_deadline)
	{
		finish_op(false, 0);
		return;
	}

	uint8_t rom_clock_bit = (m_port_c_in & 0x01) ? 1 : 0;

	switch (m_phase)
	{
	case phase::SEND_DELAY:
		if (machine().time() >= m_delay_deadline)
			m_phase = phase::SEND_SETUP_LOW;
		break;
	case phase::RECV_DELAY:
		if (machine().time() >= m_delay_deadline)
			m_phase = phase::RECV_WAIT_LOW;
		break;
	case phase::SEND_SETUP_LOW:
	{
		uint8_t b = ((m_send_data & 0x80) >> 7) | 2;
		m_port_a_out = b;
		b = (m_send_data & 0x80) >> 7;
		m_port_a_out = b;
		m_send_data <<= 1;
		m_phase = phase::SEND_WAIT_LOW;
		break;
	}
	case phase::SEND_WAIT_LOW:
		if (rom_clock_bit == 0)
			m_phase = phase::SEND_SETUP_HIGH;
		break;
	case phase::SEND_SETUP_HIGH:
	{
		uint8_t b = (m_send_data & 0x80) >> 7;
		m_port_a_out = b;
		b = ((m_send_data & 0x80) >> 7) | 2;
		m_port_a_out = b;
		m_send_data <<= 1;
		m_phase = phase::SEND_WAIT_HIGH;
		break;
	}
	case phase::SEND_WAIT_HIGH:
		if (rom_clock_bit != 0)
		{
			m_bit_index++;
			if (m_bit_index >= 4)
				finish_op(true, 0);
			else
				m_phase = phase::SEND_SETUP_LOW;
		}
		break;

	case phase::RECV_WAIT_LOW:
		if (rom_clock_bit == 0)
		{
			uint8_t bit = (m_port_c_in & 0x02) ? 1 : 0;
			m_recv_data = static_cast<uint8_t>((m_recv_data << 1) | bit);
			m_port_a_out = 0;
			m_phase = phase::RECV_WAIT_HIGH;
		}
		break;
	case phase::RECV_WAIT_HIGH:
		if (rom_clock_bit != 0)
		{
			uint8_t bit = (m_port_c_in & 0x02) ? 1 : 0;
			m_recv_data = static_cast<uint8_t>((m_recv_data << 1) | bit);
			m_port_a_out = 2;
			m_bit_index++;
			if (m_bit_index >= 4)
				finish_op(true, m_recv_data);
			else
				m_phase = phase::RECV_WAIT_LOW;
		}
		break;

	default:
		break;
	}
}
