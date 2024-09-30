/*.*******************************************************************
 FILENAME: tcp_client.h
 **********************************************************************
 *  PROJECT: ROS2_iNAT
 *
 *
 *---------------------------------------------------------------------
 * 	Copyright 2022, iMAR Navigation
 *---------------------------------------------------------------------
 * 	MODULE DESCRIPTION:
 *
 ---------------------------------------------------------------------*/
#ifndef INAT_THIRD_PARTY_LIBXCOM_SRC_TCP_CLIENT_H
#define INAT_THIRD_PARTY_LIBXCOM_SRC_TCP_CLIENT_H
#include <ixcom/ixcom.h>
#include <netinet/in.h>
#include <string>
#include <sys/time.h>
namespace xcom {
class ClientInterface {
public:
    ClientInterface()          = default;
    virtual ~ClientInterface() = default;
    [[nodiscard]] virtual bool open_connection()                                             = 0;
    virtual bool close_connection()                                                          = 0;
    [[nodiscard]] virtual int read_data(uint8_t* rx_buffer, std::size_t length) const        = 0;
    [[nodiscard]] virtual int write_data(const uint8_t* tx_buffer, std::size_t length) const = 0;
    [[nodiscard]] virtual bool set_read_timeout(int usec) const                              = 0;
    [[nodiscard]] virtual bool is_ok() const                                                 = 0;
};
class TcpClient : public ClientInterface {
public:
    explicit TcpClient(std::string addr, int port);
    ~TcpClient() override = default;
    [[nodiscard]] bool open_connection() override;
    bool close_connection() override;
    [[nodiscard]] int read_data(uint8_t* rx_buffer, std::size_t length) const override;
    [[nodiscard]] int write_data(const uint8_t* tx_buffer, std::size_t length) const override;
    [[nodiscard]] bool set_read_timeout(int usec) const override;
    [[nodiscard]] bool is_ok() const override;
    ssize_t read_line(void* buffer, size_t n) const;
private:
    std::string _addr;
    int _port;
    struct sockaddr_in _serv_addr {};
    int _sd     = -1;
    bool _error = false;
    static void normalize_timespec(struct ::timeval* ts);
};
class XcomClientReader : public xcom::IReader {
public:
    explicit XcomClientReader(ClientInterface& client);
    ~XcomClientReader() override;
    bool initialize() noexcept override;
    int32_t read(uint8_t* buffer, std::size_t buffer_length) noexcept override;
private:
    ClientInterface& _client;
};
class XcomClientWriter : public xcom::IWriter {
public:
    explicit XcomClientWriter(ClientInterface& client);
    ~XcomClientWriter() override;
    bool initialize() noexcept override;
    int32_t write(const uint8_t* buffer, std::size_t buffer_length) noexcept override;
private:
    ClientInterface& _client;
};
}  // namespace xcom
#endif  //INAT_THIRD_PARTY_LIBXCOM_SRC_TCP_CLIENT_H
