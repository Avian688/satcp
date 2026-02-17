// Satcp.h
//
// A minimal "Satcp" TCP flavour that overrides TcpCubic, with empty overrides
// (it just forwards to TcpCubic).
//

#ifndef SATCP_TRANSPORTLAYER_TCP_FLAVOURS_SATCP_H_
#define SATCP_TRANSPORTLAYER_TCP_FLAVOURS_SATCP_H_

#include "../../../../../cubic/src/transportlayer/tcp/flavours/TcpCubic.h"

namespace inet {
namespace tcp {

/**
 * Satcp: a minimal TCP flavour that derives from TcpCubic.
 */
class INET_API Satcp : public TcpCubic
{
public:
    Satcp();

    void initialize() override;
    void established(bool active) override;

    virtual void processRexmitTimer(TcpEventCode &event) override;

protected:
    bool handover_status;
    virtual void updateCubicCwnd(uint32_t acked) override;
};

} // namespace tcp
} // namespace inet

#endif /* SATCP_TRANSPORTLAYER_TCP_FLAVOURS_SATCP_H_ */
