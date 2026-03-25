#ifndef _MPFQ_H
#define _MPFQ_H

// ============ HEADER INCLUSIONS ============
#include "Transaction.h"
#include "SimulatorObject.h"
#include "SystemConfiguration.h"
#include "Callback.h"

#include <memory>
#include <stdint.h>
#include <ostream>
#include <fstream>
#include <cstring>
#include <map>
#include <string>
#include <vector>
#include <set>
#include <list>
#include <sstream>
#include <queue>

using namespace std;

namespace DRAMSim {

// ============ FORWARD DECLARATIONS ============
typedef CallbackBase<bool,unsigned,uint64_t,double,double,double> Callback_t;
typedef Transaction& t_ptr;

class PFQ;
class MPTC;
class MemorySystemTop;

// ============ MPFQ CLASS DEFINITION ============

class MPFQ : public SimulatorObject {
private:
    // ============ PRIVATE MEMBER VARIABLES ============                              
    MemorySystemTop* memorySystemTop_;
    // Logging
    vector<ofstream> &DDRSim_log;
    vector<ofstream> &trace_log;    
    ofstream perf_log;              
    string log_path;
    uint64_t channel_ohot;

    std::vector<std::unique_ptr<PFQ>> pfqs_;
    MPTC* mptc_;

public:
    // ============ CONSTRUCTOR AND DESTRUCTOR ============
    MPFQ(MemorySystemTop* top, vector<ofstream> &DDRSim_log_, vector<ofstream> &trace_log_);
    virtual ~MPFQ();
    
    void addFastRead(unsigned pfq_idx); 
    bool returnReadData(data_packet& packet);
    void update();

    void setMPTC(MPTC* mptc);
    void associateWithPTCs();
    PFQ* getPFQ(unsigned index) const;
    MemorySystemTop* getMemorySystemTop() const {return memorySystemTop_;}

    // ============ CALLBACK REGISTRATION ============
    void RegisterCallbacks(Callback_t* readData,
                           Callback_t* writeCB,
                           Callback_t* readCB,
                           Callback_t* cmdCB);
    
    // ============ STATUS QUERY FUNCTIONS ============
    uint8_t get_occ(uint32_t chl) const;           
    uint8_t get_bandwidth(uint32_t chl) const;

    // Callback function pointers
    Callback_t* ReturnReadData;     
    Callback_t* WriteResp;          
    Callback_t* ReadResp;           
    Callback_t* CmdResp;                               
              
};

} // namespace DRAMSim

#endif // _MPFQ_H