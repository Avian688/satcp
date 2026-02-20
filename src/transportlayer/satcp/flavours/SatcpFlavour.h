// Satcp.h
//
// A minimal "Satcp" TCP flavour that overrides TcpCubic, with empty overrides
// (it just forwards to TcpCubic).
//

#ifndef SATCP_TRANSPORTLAYER_TCP_FLAVOURS_SATCPFLAVOUR_H_
#define SATCP_TRANSPORTLAYER_TCP_FLAVOURS_SATCPFLAVAVOUR_H_

#include "../../../../../cubic/src/transportlayer/tcp/flavours/TcpCubic.h"
#include <vector>

namespace inet {
namespace tcp {

/**
 * Satcp: a minimal TCP flavour that derives from TcpCubic.
 */
class INET_API SatcpFlavour : public TcpCubic
{
public:
    struct HandoverEvent {
            enum Type { Crash, Connect } type;
            simtime_t t;
        };

    SatcpFlavour();

    void initialize() override;
    void established(bool active) override;

    void processRexmitTimer(TcpEventCode &event) override;

protected:
    simtime_t handover_end_time = SIMTIME_ZERO;
    bool handover_status = false;

    std::vector<HandoverEvent> handoverEvents;
    size_t nextHandoverEventIndex = 0;

    void parseHandoverEventsFromScenario(cXMLElement *scenario);
    void updateHandoverStatusFromScenario();

    void updateCubicCwnd(uint32_t acked) override;
};

} // namespace tcp
} // namespace inet

#endif /* SATCP_TRANSPORTLAYER_TCP_FLAVOURS_SATCPFLAVOUR_H_ */
