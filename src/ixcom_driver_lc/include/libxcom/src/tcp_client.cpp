/*.*******************************************************************
 FILENAME: tcp_client.cpp
 **********************************************************************
 *  PROJECT: iXCOM_SDK
 *
 *
 *---------------------------------------------------------------------
 * 	Copyright 2022, iMAR Navigation
 *---------------------------------------------------------------------
 * 	MODULE DESCRIPTION:
 *
 ---------------------------------------------------------------------*/
#include "ixcom/tcp_client.h"
#ifdef _WIN32
#include <winsock2.h>
#else
#include <arpa/inet.h>
#include <sys/socket.h>
#endif
#include <cerrno>
#include <ctime>
#include <unistd.h>
#include <utility>
#ifndef EOK
#define EOK 0
#endif
#define USEC_PER_SEC 1000000
namespace xcom {
static constexpr int ReadTimeout = 1000 * 1000 * 5;
TcpClient::TcpClient(std::string addr, int port)
    : _addr(std::move(addr)),
      _port(port) {}
bool TcpClient::open_connection() {
    _error = false;
    if((_sd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        _error = true;
        return false;
    }
    _serv_addr.sin_family = AF_INET;
    _serv_addr.sin_port   = htons(_port);
    if(inet_pton(AF_INET, _addr.c_str(), &_serv_addr.sin_addr) <= 0) {
        _error = true;
        return false;
    }
    if(connect(_sd, (struct sockaddr*)&_serv_addr, sizeof(_serv_addr)) < 0) {
        _error = true;
        return false;
    }
    return true;
}
int TcpClient::read_data(uint8_t* rx_buffer, std::size_t length) const {
    if(_error) {
        return -1;
    }
    return static_cast<int>(read(_sd, rx_buffer, length));
}
int TcpClient::write_data(const uint8_t* tx_buffer, std::size_t length) const {
    if(_error) {
        return -1;
    }
    return static_cast<int>(send(_sd, reinterpret_cast<const char*>(tx_buffer), length, 0));
}
bool TcpClient::close_connection() {
    _error = false;
    if(close(_sd) != EOK) {
        if(errno != EBADF) {
            return false;
        }
    }
    return true;
}
bool TcpClient::set_read_timeout(int usec) const {
    if(_error) {
        return false;
    }
    struct ::timeval tv {};
    tv.tv_sec  = 0;
    tv.tv_usec = usec;
    normalize_timespec(&tv);
    return (setsockopt(_sd, SOL_SOCKET, SO_RCVTIMEO, reinterpret_cast<const char*>(&tv), sizeof tv) == EOK);
}
bool TcpClient::is_ok() const { return !_error; }
void TcpClient::normalize_timespec(struct ::timeval* ts) {
    while(ts->tv_usec >= USEC_PER_SEC) {
        ++ts->tv_sec;
        ts->tv_usec -= USEC_PER_SEC;
    }
    while(ts->tv_usec < 0) {
        --ts->tv_sec;
        ts->tv_usec += USEC_PER_SEC;
    }
}
ssize_t TcpClient::read_line(void* buffer, size_t n) const {
    ssize_t num_read;
    ssize_t to_read;
    uint8_t* buf;
    char ch;
    if(n <= 0 || buffer == nullptr) {
        errno = EINVAL;
        return -1;
    }
    buf     = static_cast<uint8_t*>(buffer);
    to_read = 0;
    for(;;) {
        num_read = read(_sd, &ch, 1);
        if(num_read == -1) {
            if(errno == EINTR) { /* Interrupted --> restart read() */
                continue;
            } else {
                return -1; /* Some other error */
            }
        } else if(num_read == 0) { /* EOF */
            if(to_read == 0) {     /* No bytes read; return 0 */
                return 0;
            } else { /* Some bytes read; add '\0' */
                break;
            }
        } else {                                        /* 'numRead' must be 1 if we get here */
            if(to_read < static_cast<ssize_t>(n) - 1) { /* Discard > (n - 1) bytes */
                to_read++;
                *buf++ = ch;
            }
            if(ch == '\n') {
                break;
            }
        }
    }
    *buf = '\0';
    return to_read;
}
/*
 * *****************************************************************************************************************************************
 */
XcomClientReader::XcomClientReader(ClientInterface& client)
    : _client(client) {}
XcomClientReader::~XcomClientReader() { _client.close_connection(); }
bool XcomClientReader::initialize() noexcept {
    _client.close_connection();
    if(!_client.open_connection()) {
        return false;
    } else {
        return _client.set_read_timeout(ReadTimeout);
    }
}
int32_t XcomClientReader::read(uint8_t* buffer, std::size_t buffer_length) noexcept { return _client.read_data(buffer, buffer_length); }
/*
 * *****************************************************************************************************************************************
 */
XcomClientWriter::XcomClientWriter(ClientInterface& client)
    : _client(client) {}
XcomClientWriter::~XcomClientWriter() { _client.close_connection(); }
bool XcomClientWriter::initialize() noexcept { return _client.is_ok(); }
int32_t XcomClientWriter::write(const uint8_t* buffer, std::size_t buffer_length) noexcept {
    return _client.write_data(buffer, buffer_length);
}
}  // namespace xcom
