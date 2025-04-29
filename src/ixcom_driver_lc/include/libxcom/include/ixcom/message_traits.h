#ifndef LIBXCOM_MESSAGE_TRAITS_H
#define LIBXCOM_MESSAGE_TRAITS_H
#include <cmath>
#include <ixcom/XCOMdat.h>
#include <type_traits>
template<typename>
struct MessageTraits;
template<>
struct MessageTraits<XCOMmsg_GNSSSOL> {
    static constexpr uint8_t Id = XCOM_MSGID_GNSSSOL;
};
template<>
struct MessageTraits<XCOMmsg_GNSSTIME> {
    static constexpr uint8_t Id = XCOM_MSGID_GNSSTIME;
};
template<>
struct MessageTraits<XCOMmsg_SYSSTAT> {
    static constexpr uint8_t Id = XCOM_MSGID_SYSSTAT;
};
template<>
struct MessageTraits<XCOMmsg_INSSOL> {
    static constexpr uint8_t Id = XCOM_MSGID_INSSOL;
};
template<>
struct MessageTraits<XCOMmsg_IMUCORR> {
    static constexpr uint8_t Id = XCOM_MSGID_IMUCORR;
};
template<>
struct MessageTraits<XCOMmsg_EKFSTDDEV> {
    static constexpr uint8_t Id = XCOM_MSGID_EKFSTDDEV;
};
template<>
struct MessageTraits<XCOMmsg_INSSOLECEF> {
    static constexpr uint8_t Id = XCOM_MSGID_INSSOLECEF;
};
template<>
struct MessageTraits<XCOMmsg_EKFSTDDEVECEF> {
    static constexpr uint8_t Id = XCOM_MSGID_EKFSTDDEVECEF;
};
template<>
struct MessageTraits<XCOMmsg_INSDCM> {
    static constexpr uint8_t Id = XCOM_MSGID_INSDCM;
};
template<>
struct MessageTraits<XCOMmsg_MAGDATA> {
    static constexpr uint8_t Id = XCOM_MSGID_MAGDATA;
};
template<>
struct MessageTraits<XCOMmsg_GNSSLEVERARM> {
    static constexpr uint8_t Id = XCOM_MSGID_GNSSLEVERARM;
};
#endif