// license:BSD-3-Clause
// copyright-holders:POFOSCAB project
#include "emu.h"
#include "pofo_bridge.h"

#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>

namespace {
#define M82C55A_TAG "u1"

// Polled at a fine enough grain to not visibly delay a bit-bang
// handshake driven from the far end of the socket - the wire protocol
// this bridges (see POFOSCAB's PROTOCOL.md / PortfolioLink.cpp) waits
// on signal edges with microsecond-scale timeouts, so anything much
// coarser than this would show up as added latency on every bit.
constexpr attotime POLL_INTERVAL = attotime::from_usec(20);
constexpr uint16_t DEFAULT_TCP_PORT = 9999;
}

DEFINE_DEVICE_TYPE(POFO_BRIDGE, pofo_bridge_device, "pofo_bridge", "POFOSCAB Test Harness Bridge")

pofo_bridge_device::pofo_bridge_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, POFO_BRIDGE, tag, owner, clock)
	, device_portfolio_expansion_slot_interface(mconfig, *this)
	, m_ppi(*this, M82C55A_TAG)
	, m_poll_timer(nullptr)
	, m_port_a_in(0xff)
	, m_port_c_in(0xff)
	, m_listen_fd(-1)
	, m_client_fd(-1)
	, m_recv_partial(0)
	, m_recv_have_partial(false)
	, m_tcp_port(DEFAULT_TCP_PORT)
{
}

pofo_bridge_device::~pofo_bridge_device()
{
}

void pofo_bridge_device::device_add_mconfig(machine_config &config)
{
	I8255A(config, m_ppi);
	m_ppi->in_pa_callback().set(FUNC(pofo_bridge_device::data_in_r));
	m_ppi->out_pa_callback().set(FUNC(pofo_bridge_device::data_out_w));
	m_ppi->in_pc_callback().set(FUNC(pofo_bridge_device::status_in_r));
	m_ppi->out_pc_callback().set(FUNC(pofo_bridge_device::status_out_w));
	// Port B (control latch on a real HPC-101/Centronics setup) is not
	// used by the Smart Cable wire protocol at all - left unconnected.
}

void pofo_bridge_device::device_start()
{
	save_item(NAME(m_port_a_in));
	save_item(NAME(m_port_c_in));

	open_listen_socket();

	m_poll_timer = timer_alloc(FUNC(pofo_bridge_device::poll_socket), this);
	m_poll_timer->adjust(POLL_INTERVAL, 0, POLL_INTERVAL);
}

void pofo_bridge_device::device_stop()
{
	close_client();
	if (m_listen_fd >= 0)
	{
		close(m_listen_fd);
		m_listen_fd = -1;
	}
}

void pofo_bridge_device::device_reset()
{
	m_ppi->reset();
}

uint8_t pofo_bridge_device::nrdi_r(offs_t offset, uint8_t data, bool iom, bool bcom, bool ncc1)
{
	// Same address decode as a real HPC-101 (see hpc101.cpp) - the PPI
	// occupies offset bits [3:2]==0b10 within the card's I/O window, with
	// an identification byte 0x02 at offset bits [3:0]==0xf.
	if (!bcom)
	{
		if ((offset & 0x0f) == 0x0f)
			data = 0x02;

		if ((offset & 0x0c) == 0x08)
			data = m_ppi->read(offset & 0x03);
	}

	return data;
}

void pofo_bridge_device::nwri_w(offs_t offset, uint8_t data, bool iom, bool bcom, bool ncc1)
{
	if (!bcom)
	{
		if ((offset & 0x0c) == 0x08)
			m_ppi->write(offset & 0x03, data);
	}
}

uint8_t pofo_bridge_device::data_in_r()
{
	return m_port_a_in;
}

void pofo_bridge_device::data_out_w(uint8_t data)
{
	send_register(REG_PORT_A, data);
}

uint8_t pofo_bridge_device::status_in_r()
{
	return m_port_c_in;
}

void pofo_bridge_device::status_out_w(uint8_t data)
{
	send_register(REG_PORT_C, data);
}

void pofo_bridge_device::open_listen_socket()
{
	m_listen_fd = socket(AF_INET, SOCK_STREAM, 0);
	if (m_listen_fd < 0)
	{
		logerror("pofo_bridge: socket() failed\n");
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
		logerror("pofo_bridge: bind() to port %u failed\n", m_tcp_port);
		close(m_listen_fd);
		m_listen_fd = -1;
		return;
	}

	if (listen(m_listen_fd, 1) < 0)
	{
		logerror("pofo_bridge: listen() failed\n");
		close(m_listen_fd);
		m_listen_fd = -1;
		return;
	}

	fcntl(m_listen_fd, F_SETFL, O_NONBLOCK);
	logerror("pofo_bridge: listening on 127.0.0.1:%u\n", m_tcp_port);
}

void pofo_bridge_device::accept_pending()
{
	if (m_listen_fd < 0 || m_client_fd >= 0)
		return;

	int fd = accept(m_listen_fd, nullptr, nullptr);
	if (fd >= 0)
	{
		fcntl(fd, F_SETFL, O_NONBLOCK);
		// Without this, Nagle's algorithm buffers our 2-byte
		// {register_id, value} writes waiting for either more data or
		// the peer's ACK, and the peer's delayed-ACK timer (typically
		// ~40ms on Linux) stacks with it - this single flag was the
		// root cause of a ~40-70ms per-byte handshake stall (see
		// POFOSCAB's tests/tests.md investigation).
		int nodelay = 1;
		setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));
		m_client_fd = fd;
		logerror("pofo_bridge: bridge client connected\n");
	}
}

void pofo_bridge_device::close_client()
{
	if (m_client_fd >= 0)
	{
		close(m_client_fd);
		m_client_fd = -1;
	}
}

TIMER_CALLBACK_MEMBER(pofo_bridge_device::poll_socket)
{
	accept_pending();
	drain_socket();
}

void pofo_bridge_device::drain_socket()
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
				apply_incoming(m_recv_partial, byte);
				m_recv_have_partial = false;
			}
			continue;
		}
		if (n == 0)
		{
			logerror("pofo_bridge: bridge client disconnected\n");
			close_client();
		}
		// n < 0: EAGAIN/EWOULDBLOCK (no more data) or a real error either
		// way, nothing more to read from a non-blocking socket right now.
		break;
	}
}

void pofo_bridge_device::apply_incoming(uint8_t reg, uint8_t value)
{
	switch (reg)
	{
	case REG_PORT_A: m_port_a_in = value; break;
	case REG_PORT_C: m_port_c_in = value; break;
	default:
		logerror("pofo_bridge: unknown register id %u from bridge\n", reg);
		break;
	}
}

void pofo_bridge_device::send_register(uint8_t reg, uint8_t value)
{
	if (m_client_fd < 0)
		return;

	uint8_t buf[2] = { reg, value };
	// Best-effort - if the bridge isn't keeping up or has disconnected,
	// drop the update rather than block the emulated machine on a send().
	send(m_client_fd, buf, sizeof(buf), MSG_NOSIGNAL);
}
