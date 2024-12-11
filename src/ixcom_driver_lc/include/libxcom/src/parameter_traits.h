#ifndef LIBXCOM_PARAMETER_TRAITS_H
#define LIBXCOM_PARAMETER_TRAITS_H
#include <cmath>
#include <ixcom/XCOMdat.h>
#include <type_traits>
template<typename>
struct ParameterTraits;
template<>
struct ParameterTraits<XCOMParSYS_MAINTIMING> {
    static constexpr uint16_t Id = XCOMPAR_PARSYS_MAINTIMING;
};
template<>
struct ParameterTraits<XCOMParSYS_PRESCALER> {
    static constexpr uint16_t Id = XCOMPAR_PARSYS_PRESCALER;
};
template<>
struct ParameterTraits<XCOMParSYS_FWVERSION> {
    static constexpr uint16_t Id = XCOMPAR_PARSYS_FWVERSION;
};
template<>
struct ParameterTraits<XCOMParDAT_SYSSTAT> {
    static constexpr uint16_t Id = XCOMPAR_PARDAT_SYSSTAT;
};
template<>
struct ParameterTraits<XCOMParXCOM_SERIALPORT> {
    static constexpr uint16_t Id = XCOMPAR_PARXCOM_SERIALPORT;
};
template<>
struct ParameterTraits<XCOMParXCOM_INTERFACE> {
    static constexpr uint16_t Id = XCOMPAR_PARXCOM_INTERFACE;
};
template<>
struct ParameterTraits<XCOMParDAT_VEL> {
    static constexpr uint16_t Id = XCOMPAR_PARDAT_VEL;
};
template<>
struct ParameterTraits<XCOMParGNSS_LOCKOUTSYSTEM> {
    static constexpr uint16_t Id = XCOMPAR_PARGNSS_LOCKOUTSYSTEM;
};
template<>
struct ParameterTraits<XCOMParDAT_POS> {
    static constexpr uint16_t Id = XCOMPAR_PARDAT_POS;
};
template<>
struct ParameterTraits<XCOMParEKF_MAGATTAID> {
    static constexpr uint16_t Id = XCOMPAR_PAREKF_MAGATTAID;
};
template<>
struct ParameterTraits<XCOMParEKF_IMUCONFIG2> {
    static constexpr uint16_t Id = XCOMPAR_PAREKF_IMUCONFIG2;
};
template<>
struct ParameterTraits<XCOMParXCOM_POSTPROC> {
    static constexpr uint16_t Id = XCOMPAR_PARXCOM_POSTPROC;
};
template<>
struct ParameterTraits<XCOMParEKF_STARTUPV2> {
    static constexpr uint16_t Id = XCOMPAR_PAREKF_STARTUPV2;
};
#endif