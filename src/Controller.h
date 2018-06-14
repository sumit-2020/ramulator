#ifndef __CONTROLLER_H
#define __CONTROLLER_H

#include <cassert>
#include <cstdio>
#include <deque>
#include <fstream>
#include <list>
#include <string>
#include <vector>
#include <ctime>

#include "Config.h"
#include "DRAM.h"
#include "Refresh.h"
#include "Request.h"
#include "Scheduler.h"
#include "Statistics.h"


#include "ALDRAM.h"
#include "SALP.h"
#include "TLDRAM.h"

using namespace std;

namespace ramulator
{

    extern bool warmup_complete;

template <typename T>
class Controller
{
protected:
    // For counting bandwidth
    ScalarStat read_transaction_bytes;
    ScalarStat write_transaction_bytes;

    ScalarStat row_hits;
    ScalarStat row_misses;
    ScalarStat row_conflicts;
    VectorStat read_row_hits;
    VectorStat read_row_misses;
    VectorStat read_row_conflicts;
    VectorStat write_row_hits;
    VectorStat write_row_misses;
    VectorStat write_row_conflicts;
    ScalarStat useless_activates;

    ScalarStat read_latency_avg;
    ScalarStat read_latency_sum;

    ScalarStat req_queue_length_avg;
    ScalarStat req_queue_length_sum;
    ScalarStat read_req_queue_length_avg;
    ScalarStat read_req_queue_length_sum;
    ScalarStat write_req_queue_length_avg;
    ScalarStat write_req_queue_length_sum;

#ifndef INTEGRATED_WITH_GEM5
    VectorStat record_read_hits;
    VectorStat record_read_misses;
    VectorStat record_read_conflicts;
    VectorStat record_write_hits;
    VectorStat record_write_misses;
    VectorStat record_write_conflicts;
#endif

public:
    /* Member Variables */
    long clk = 0;
    DRAM<T>* channel;

    Scheduler<T>* scheduler;  // determines the highest priority request whose commands will be issued
    RowPolicy<T>* rowpolicy;  // determines the row-policy (e.g., closed-row vs. open-row)
    RowTable<T>* rowtable;  // tracks metadata about rows (e.g., which are open and for how long)
    Refresh<T>* refresh;

    const int RND_TIME=1000; // only used in AHB type scheduler
    const int HISTORY_TIME=400; // only used in AHB type scheduler
    const bool SMART_PDN=false; // whether to insert PDN smartly for ranks which aren't in use or not
    
    // const int choice1_wt=10;// weights for the three choices in AHB; should total 100
    //const int choice2_wt=45;
    //const int choice3_wt=45;
    
    int choice1count=0;
    int choice2count=0;
    int choice3count=0;    
    int arbiter1count=0;
    int arbiter2count=0;
    int arbiter3count=0; 
    
    array<typename T::Command, 2> history; // SUMIT Array to store the last 2 issued requests
    array<int, 2> bankhistory; // Array to store the most recently accessed two banks
    int readCountLast10000=1;  // SUMIT Counters to count # of reads & writes in last 10000 issued commands
    int writeCountLast10000=1;
    int arbiter=2; // 1/2/3 Based on rdwrratio >1.2/>0.8/<0.8 respectively
    float rdwrratio=1; //updated continuously; checked every HISTORY_TIME ticks to update arbiter
    int rndnum=1; //a randum number which is generated every RND_TIME
    int choice=1; //updated every RND_TIME ticks; only used in AHB type scheduler; 1/2/3 for latency(performance)/AHB(to match rdwrratio)/power_aware

    
    struct Queue {
        list<Request> q;
        unsigned int max = 32;
        unsigned int size() {return q.size();}
    };

    Queue readq;  // queue for read requests
    Queue writeq;  // queue for write requests
    Queue actq; // read and write requests for which activate was issued are moved to 
                   // actq, which has higher priority than readq and writeq.
                   // This is an optimization
                   // for avoiding useless activations (i.e., PRECHARGE
                   // after ACTIVATE w/o READ of WRITE command)
    Queue otherq;  // queue for all "other" requests (e.g., refresh)
    

    deque<Request> pending;  // read requests that are about to receive data from DRAM
    bool write_mode = false;  // whether write requests should be prioritized over reads
    float wr_high_watermark = 0.8f; // threshold for switching to write mode
    float wr_low_watermark = 0.2f; // threshold for switching back to read mode
    //long refreshed = 0;  // last time refresh requests were generated

    /* Command trace for DRAMPower 3.1 */
    string cmd_trace_prefix = "cmd-trace-";
    vector<ofstream> cmd_trace_files;
    bool record_cmd_trace = false;
    /* Commands to stdout */
    bool print_cmd_trace = false;

    /* Constructor */
    Controller(const Config& configs, DRAM<T>* channel) :
        channel(channel),
        scheduler(new Scheduler<T>(this)),
        rowpolicy(new RowPolicy<T>(this)),
        rowtable(new RowTable<T>(this)),
        refresh(new Refresh<T>(this)),
        cmd_trace_files(channel->children.size())
    {
        record_cmd_trace = configs.record_cmd_trace();
        print_cmd_trace = configs.print_cmd_trace();
        if (record_cmd_trace){
            if (configs["cmd_trace_prefix"] != "") {
              cmd_trace_prefix = configs["cmd_trace_prefix"];
            }
            string prefix = cmd_trace_prefix + "chan-" + to_string(channel->id) + "-rank-";
            string suffix = ".cmdtrace";
            for (unsigned int i = 0; i < channel->children.size(); i++)
                cmd_trace_files[i].open(prefix + to_string(i) + suffix);
        }

        // regStats

        row_hits
            .name("row_hits_channel_"+to_string(channel->id) + "_core")
            .desc("Number of row hits per channel per core")
            .precision(0)
            ;
        row_misses
            .name("row_misses_channel_"+to_string(channel->id) + "_core")
            .desc("Number of row misses per channel per core")
            .precision(0)
            ;
        row_conflicts
            .name("row_conflicts_channel_"+to_string(channel->id) + "_core")
            .desc("Number of row conflicts per channel per core")
            .precision(0)
            ;

        read_row_hits
            .init(configs.get_core_num())
            .name("read_row_hits_channel_"+to_string(channel->id) + "_core")
            .desc("Number of row hits for read requests per channel per core")
            .precision(0)
            ;
        read_row_misses
            .init(configs.get_core_num())
            .name("read_row_misses_channel_"+to_string(channel->id) + "_core")
            .desc("Number of row misses for read requests per channel per core")
            .precision(0)
            ;
        read_row_conflicts
            .init(configs.get_core_num())
            .name("read_row_conflicts_channel_"+to_string(channel->id) + "_core")
            .desc("Number of row conflicts for read requests per channel per core")
            .precision(0)
            ;

        write_row_hits
            .init(configs.get_core_num())
            .name("write_row_hits_channel_"+to_string(channel->id) + "_core")
            .desc("Number of row hits for write requests per channel per core")
            .precision(0)
            ;
        write_row_misses
            .init(configs.get_core_num())
            .name("write_row_misses_channel_"+to_string(channel->id) + "_core")
            .desc("Number of row misses for write requests per channel per core")
            .precision(0)
            ;
        write_row_conflicts
            .init(configs.get_core_num())
            .name("write_row_conflicts_channel_"+to_string(channel->id) + "_core")
            .desc("Number of row conflicts for write requests per channel per core")
            .precision(0)
            ;

        useless_activates
            .name("useless_activates_"+to_string(channel->id)+ "_core")
            .desc("Number of useless activations. E.g, ACT -> PRE w/o RD or WR")
            .precision(0)
            ;

        read_transaction_bytes
            .name("read_transaction_bytes_"+to_string(channel->id))
            .desc("The total byte of read transaction per channel")
            .precision(0)
            ;
        write_transaction_bytes
            .name("write_transaction_bytes_"+to_string(channel->id))
            .desc("The total byte of write transaction per channel")
            .precision(0)
            ;

        read_latency_sum
            .name("read_latency_sum_"+to_string(channel->id))
            .desc("The memory latency cycles (in memory time domain) sum for all read requests in this channel")
            .precision(0)
            ;
        read_latency_avg
            .name("read_latency_avg_"+to_string(channel->id))
            .desc("The average memory latency cycles (in memory time domain) per request for all read requests in this channel")
            .precision(6)
            ;

        req_queue_length_sum
            .name("req_queue_length_sum_"+to_string(channel->id))
            .desc("Sum of read and write queue length per memory cycle per channel.")
            .precision(0)
            ;
        req_queue_length_avg
            .name("req_queue_length_avg_"+to_string(channel->id))
            .desc("Average of read and write queue length per memory cycle per channel.")
            .precision(6)
            ;

        read_req_queue_length_sum
            .name("read_req_queue_length_sum_"+to_string(channel->id))
            .desc("Read queue length sum per memory cycle per channel.")
            .precision(0)
            ;
        read_req_queue_length_avg
            .name("read_req_queue_length_avg_"+to_string(channel->id))
            .desc("Read queue length average per memory cycle per channel.")
            .precision(6)
            ;

        write_req_queue_length_sum
            .name("write_req_queue_length_sum_"+to_string(channel->id))
            .desc("Write queue length sum per memory cycle per channel.")
            .precision(0)
            ;
        write_req_queue_length_avg
            .name("write_req_queue_length_avg_"+to_string(channel->id))
            .desc("Write queue length average per memory cycle per channel.")
            .precision(6)
            ;

#ifndef INTEGRATED_WITH_GEM5
        record_read_hits
            .init(configs.get_core_num())
            .name("record_read_hits")
            .desc("record read hit count for this core when it reaches request limit or to the end")
            ;

        record_read_misses
            .init(configs.get_core_num())
            .name("record_read_misses")
            .desc("record_read_miss count for this core when it reaches request limit or to the end")
            ;

        record_read_conflicts
            .init(configs.get_core_num())
            .name("record_read_conflicts")
            .desc("record read conflict count for this core when it reaches request limit or to the end")
            ;

        record_write_hits
            .init(configs.get_core_num())
            .name("record_write_hits")
            .desc("record write hit count for this core when it reaches request limit or to the end")
            ;

        record_write_misses
            .init(configs.get_core_num())
            .name("record_write_misses")
            .desc("record write miss count for this core when it reaches request limit or to the end")
            ;

        record_write_conflicts
            .init(configs.get_core_num())
            .name("record_write_conflicts")
            .desc("record write conflict for this core when it reaches request limit or to the end")
            ;
#endif
    }

    ~Controller(){
        delete scheduler;
        delete rowpolicy;
        delete rowtable;
        delete channel;
        delete refresh;
        for (auto& file : cmd_trace_files)
            file.close();
        cmd_trace_files.clear();
    }

    void finish(long read_req, long dram_cycles) {
      cout<<"choice1count = "<<choice1count<<"\n";
      cout<<"choice2count = "<<choice2count<<"\n";
      cout<<"choice3count = "<<choice3count<<"\n";
      cout<<"arbiter1count = "<<arbiter1count<<"\n";
      cout<<"arbiter2count = "<<arbiter2count<<"\n";
      cout<<"arbiter3count = "<<arbiter3count<<"\n";
      
      
      read_latency_avg = read_latency_sum.value() / read_req;
      req_queue_length_avg = req_queue_length_sum.value() / dram_cycles;
      read_req_queue_length_avg = read_req_queue_length_sum.value() / dram_cycles;
      write_req_queue_length_avg = write_req_queue_length_sum.value() / dram_cycles;
      // call finish function of each channel
      channel->finish(dram_cycles);
    }

    /* Member Functions */
    Queue& get_queue(Request::Type type)
    {
        switch (int(type)) {
            case int(Request::Type::READ): return readq;
            case int(Request::Type::WRITE): return writeq;
            default: return otherq;
        }
    }

    bool enqueue(Request& req)
    {
        Queue& queue = get_queue(req.type);
        if (queue.max == queue.size())
            return false;

        req.arrive = clk;
        queue.q.push_back(req);
        // shortcut for read requests, if a write to same addr exists
        // necessary for coherence
        if (req.type == Request::Type::READ && find_if(writeq.q.begin(), writeq.q.end(),
                [req](Request& wreq){ return req.addr == wreq.addr;}) != writeq.q.end()){
            req.depart = clk + 1;
            pending.push_back(req);
            readq.q.pop_back();
        }
        return true;
    }
    void smart_Pdn_Pup_insert(){
        Queue *queue=&actq;
        Queue *queue1=&readq;
        Queue *queue2=&writeq;
        Queue *queue3=&otherq;
        int rank_no_violate=-1; //rank which has no command in all 3 queues, can be powered down
        int rank_violate=-1;      //rank which is powered down and has awaiting commands in any queue, needs to be powered up

        // for(int r=0; r<channel->spec->org_entry.count[int(Level::Rank)]; r++){
        for (auto r : channel->children){

            auto head=queue->q.begin();
            for(auto itr = head; itr!=queue->q.end(); itr++)
                if(itr->addr_vec[int(T::Level::Rank)]==r->id) {
                    rank_violate=r->id;
                    goto cnt;
                }

            head=queue1->q.begin();
            for(auto itr = head; itr!=queue1->q.end(); itr++)
                if(itr->addr_vec[int(T::Level::Rank)]==r->id) {
                rank_violate=r->id;
                goto cnt;
                }

            head=queue2->q.begin();
            for(auto itr = head; itr!=queue2->q.end(); itr++)
                if(itr->addr_vec[int(T::Level::Rank)]==r->id) {
                    rank_violate=r->id;
                    goto cnt;
                }


            rank_no_violate=r->id;

            cnt:

            if(rank_no_violate!=-1 && otherq.size()<32 ) {
                if (r->state == T::State::PowerUp) {

                    bool possible = true;
                    head = queue3->q.begin();
                    for (auto itr = head; itr != queue3->q.end(); itr++) {
                        if (itr->addr_vec[int(T::Level::Rank)] == r->id
                            && (itr->type == Request::Type::FACTPOWERDOWN
                                || itr->type == Request::Type::SACTPOWERDOWN
                                || itr->type == Request::Type::FPREPOWERDOWN
                                || itr->type == Request::Type::SPREPOWERDOWN)) {
                            possible = false;
                        }
                    }

                    if (possible /*&& !channel->spec->powerdown_pending[rank_no_violate]*/) {
                        vector<int> addr_vec(int(T::Level::MAX), -1);
                        addr_vec[0] = 0;
                        addr_vec[1] = rank_no_violate;
                        addr_vec[2] = 0;
                        addr_vec[3] = 0;
                        Request req(addr_vec, Request::Type::FACTPOWERDOWN, NULL);
                        bool res = enqueue(req);
                        assert(res);
                    }
                }
            }

            if(rank_violate!=-1 && otherq.size()<32) {
                if (r->state == T::State::FActPowerDown
                    || r->state == T::State::FPrePowerDown
                    || r->state == T::State::SActPowerDown
                    || r->state == T::State::SPrePowerDown) {

                    bool possible = true;
                    head = queue3->q.begin();
                    for (auto itr = head; itr != queue3->q.end(); itr++) {
                        if (itr->addr_vec[int(T::Level::Rank)] == r->id
                            && (itr->type == Request::Type::ACTPOWERUP || itr->type == Request::Type::PREPOWERUP)) {
                            possible = false;
                        }
                    }

                    if (possible /*&& !channel->spec->powerup_pending[rank_violate]*/) {
                        vector<int> addr_vec(int(T::Level::MAX), -1);
                        addr_vec[0] = 0;
                        addr_vec[1] = rank_violate;
                        addr_vec[2] = 0;
                        addr_vec[3] = 0;
                        Request req(addr_vec, Request::Type::ACTPOWERUP, NULL);
                        bool res = enqueue(req);
                        assert(res);
                    }
                }
            } // Rank voilate check
            rank_no_violate=-1; //reset these values so that they aren't used in next iteration
            rank_violate=-1;    //reset these values so that they aren't used in next iteration

        }
    }

    void tick()
    {
        clk++;
        if(SMART_PDN && otherq.size()<32){//check whether some rank can be powered down or some sleeping rank needs to be powered up
            smart_Pdn_Pup_insert();
        }
        if(clk%HISTORY_TIME==0) {
            rdwrratio=readCountLast10000/writeCountLast10000;
            if       (rdwrratio>1.2) {arbiter=1;arbiter1count++;}
            else if(rdwrratio>0.8) {arbiter=2;arbiter2count++;}
            else                           {arbiter=3;arbiter3count++;}
            readCountLast10000=1;
            writeCountLast10000=1;
        } // SUMIT Select appropriate arbiter(FSM) and reset these counters every HISTORY_TIME ticks

        if(clk%RND_TIME==0) {
            rndnum=rand()%3;
            if        ( rndnum == 0) {choice=1;choice1count++;}
            else if ( rndnum == 1) {choice=2;choice2count++;}
            else                             {choice=3;choice3count++;}
            //cout<<choice<<"\n";
        }
	
        req_queue_length_sum += readq.size() + writeq.size() + pending.size();
        read_req_queue_length_sum += readq.size() + pending.size();
        write_req_queue_length_sum += writeq.size();

        /*** 1. Serve completed reads ***/
        if (pending.size()) {
            Request& req = pending[0];
            if (req.depart <= clk) {
                if (req.depart - req.arrive > 1) { // this request really accessed a row
                    read_latency_sum += req.depart - req.arrive;
                    channel->update_serving_requests(
                            req.addr_vec.data(), -1, clk);
                }
                req.callback(req);
                pending.pop_front();
            }
        }

        /*** 2. Refresh scheduler ***/
        refresh->tick_ref();

        /*** 3. Should we schedule writes? ***/
        if (!write_mode) {
            // yes -- write queue is almost full or read queue is empty
            if (writeq.size() > int(wr_high_watermark * writeq.max) 
                /*|| readq.size() == 0*/) // Hasan: Switching to write mode when there are just a few
                // write requests, even if the read queue is empty, incurs a lot of overhead.
                // Commented out the read request queue empty condition
                write_mode = true;
        }
        else {
            // no -- write queue is almost empty and read queue is not empty
            if (writeq.size() < int(wr_low_watermark * writeq.max) && readq.size() != 0)
                write_mode = false;
        }

        /*** 4. Find the best command to schedule, if any ***/

        // First check the actq (which has higher priority) to see if there
        // are requests available to service in this cycle
        Queue* queue = &actq;
        Queue* queue1= &readq;
        Queue* queue2= &writeq;

        auto req = scheduler->get_head(queue->q,queue->q);
	
        if (req == queue->q.end() || !is_ready(req)) {
            queue = !write_mode ? &readq : &writeq;

            if (otherq.size())
                queue = &otherq;  // "other" requests are rare, so we give them precedence over reads/writes

            if( queue != &otherq && this->scheduler->type == Scheduler<T>::Type::AHB )
                req = scheduler->get_head(queue1->q,queue2->q);
            else
                req = scheduler->get_head(queue->q,queue->q);
       
        }

        if (req == queue->q.end() ||  ((this->scheduler->type == Scheduler<T>::Type::AHB)&&(req==queue1->q.end()||req==queue2->q.end())) || !is_ready(req)) {
            // we couldn't find a command to schedule -- let's try to be speculative
            auto cmd = T::Command::PRE;
            vector<int> victim = rowpolicy->get_victim(cmd);
            if (!victim.empty()){
                issue_cmd(cmd, victim);
                //updatehistory(cmd); updating PRE commands to history isn't useful in rdwrratio
            }
            return;  // nothing more to be done this cycle
        }

        if (req->is_first_command) {
            req->is_first_command = false;
            int coreid = req->coreid;
            if (req->type == Request::Type::READ || req->type == Request::Type::WRITE) {
                channel->update_serving_requests(req->addr_vec.data(), 1, clk);
            }
            int tx = (channel->spec->prefetch_size * channel->spec->channel_width / 8);
            if (req->type == Request::Type::READ) {
                if (is_row_hit(req)) {
                    ++read_row_hits[coreid];
                    ++row_hits;
                } else if (is_row_open(req)) {
                    ++read_row_conflicts[coreid];
                    ++row_conflicts;
                } else {
                    ++read_row_misses[coreid];
                    ++row_misses;
                }
                read_transaction_bytes += tx;
            } else if (req->type == Request::Type::WRITE) {
                if (is_row_hit(req)) {
                    ++write_row_hits[coreid];
                    ++row_hits;
                } else if (is_row_open(req)) {
                    ++write_row_conflicts[coreid];
                    ++row_conflicts;
                } else {
                    ++write_row_misses[coreid];
                    ++row_misses;
                }
                write_transaction_bytes += tx;
            }
        }

        // issue command on behalf of request
        auto cmd = get_first_cmd(req);
       /* if(channel->spec->is_poweringdown(cmd)) {
            channel->spec->update_powerdown_pending(get_addr_vec(cmd,req));
        }
        if(channel->spec->is_poweringup(cmd)) {
            channel->spec->update_powerup_pending(get_addr_vec(cmd,req));
        }*/
        issue_cmd(cmd, get_addr_vec(cmd, req));
	    /*if(int(cmd)==13)
	        cout<<"\nvalue of clk :"<<clk<<"\n";*/
        updatebankhistory(get_addr_vec(cmd,req));
        updatehistory(cmd);

        // check whether this is the last command (which finishes the request)
        //if (cmd != channel->spec->translate[int(req->type)]){
        if (!(channel->spec->is_accessing(cmd)
              || channel->spec->is_refreshing(cmd)/*
              || channel->spec->is_poweringdown(cmd)
              || channel->spec->is_poweringup(cmd)*/)) {
            if(channel->spec->is_opening(cmd) /*|| channel->spec->is_poweringdown(cmd)|| channel->spec->is_poweringup(cmd)*/) {
                // promote the request that caused issuing activation to actq
                actq.q.push_back(*req);
                queue->q.erase(req);
            }
            if (channel->spec->is_poweringdown(cmd) || channel->spec->is_poweringup(cmd)) {
                queue->q.erase(req);
            }
            return;
        }

        // set a future completion time for read requests
        if (req->type == Request::Type::READ) {
            req->depart = clk + channel->spec->read_latency;
            pending.push_back(*req);
        }

        if (req->type == Request::Type::WRITE) {
            channel->update_serving_requests(req->addr_vec.data(), -1, clk);
        }

        // remove request from queue
        queue->q.erase(req);

    }

    bool is_ready(list<Request>::iterator req)
    {
        typename T::Command cmd = get_first_cmd(req);
        return channel->check(cmd, req->addr_vec.data(), clk);
    }

    bool is_ready(typename T::Command cmd, const vector<int>& addr_vec)
    {
        return channel->check(cmd, addr_vec.data(), clk);
    }

    bool is_row_hit(list<Request>::iterator req)
    {
        // cmd must be decided by the request type, not the first cmd
        typename T::Command cmd = channel->spec->translate[int(req->type)];
        return channel->check_row_hit(cmd, req->addr_vec.data());
    }

    bool is_row_hit(typename T::Command cmd, const vector<int>& addr_vec)
    {
        return channel->check_row_hit(cmd, addr_vec.data());
    }

    bool is_row_open(list<Request>::iterator req)
    {
        // cmd must be decided by the request type, not the first cmd
        typename T::Command cmd = channel->spec->translate[int(req->type)];
        return channel->check_row_open(cmd, req->addr_vec.data());
    }

    bool is_row_open(typename T::Command cmd, const vector<int>& addr_vec)
    {
        return channel->check_row_open(cmd, addr_vec.data());
    }

    void update_temp(ALDRAM::Temp current_temperature)
    {
    }

    // For telling whether this channel is busying in processing read or write
    bool is_active() {
        return (channel->cur_serving_requests > 0);
    }

    // For telling whether this channel is under refresh
    bool is_refresh() {
        return clk <= channel->end_of_refreshing;
    }

    void set_high_writeq_watermark(const float watermark) {
        wr_high_watermark = watermark;
    }

    void set_low_writeq_watermark(const float watermark) {
        wr_low_watermark = watermark;
    }

    void record_core(int coreid) {
#ifndef INTEGRATED_WITH_GEM5
        record_read_hits[coreid] = read_row_hits[coreid];
        record_read_misses[coreid] = read_row_misses[coreid];
        record_read_conflicts[coreid] = read_row_conflicts[coreid];
        record_write_hits[coreid] = write_row_hits[coreid];
        record_write_misses[coreid] = write_row_misses[coreid];
        record_write_conflicts[coreid] = write_row_conflicts[coreid];
#endif
    }

private:

    void updatebankhistory(const vector<int>& addr_vec)
    {
        bankhistory[0] = bankhistory[1];
        bankhistory[1] = addr_vec[int(T::Level::Bank)];
    }
    
    void updatehistory(typename T::Command cmd)
    {
        if(cmd == T::Command::RD || cmd == T::Command::RDA || cmd == T::Command::WR || cmd == T::Command::WRA){
            if(cmd == T::Command::RD || cmd == T::Command::RDA)   readCountLast10000++;
            if(cmd == T::Command::WR || cmd == T::Command::WRA) writeCountLast10000++;
            history[0]=history[1];
            history[1]=cmd;
        }
    }
    
    typename T::Command get_first_cmd(list<Request>::iterator req)
    {
        typename T::Command cmd = channel->spec->translate[int(req->type)];
        return channel->decode(cmd, req->addr_vec.data());
    }

    // upgrade to an autoprecharge command
    void cmd_issue_autoprecharge(typename T::Command& cmd,
                                 const vector<int>& addr_vec) {

        // currently, autoprecharge is only used with closed row policy
        if(channel->spec->is_accessing(cmd) && rowpolicy->type == RowPolicy<T>::Type::ClosedAP) {
            // check if it is the last request to the opened row
            Queue* queue = write_mode ? &writeq : &readq;

            auto begin = addr_vec.begin();
            vector<int> rowgroup(begin, begin + int(T::Level::Row) + 1);

            int num_row_hits = 0;

            for (auto itr = queue->q.begin(); itr != queue->q.end(); ++itr) {
                if (is_row_hit(itr)) { 
                    auto begin2 = itr->addr_vec.begin();
                    vector<int> rowgroup2(begin2, begin2 + int(T::Level::Row) + 1);
                    if(rowgroup == rowgroup2)
                        num_row_hits++;
                }
            }

            if(num_row_hits == 0) {
                Queue* queue = &actq;
                for (auto itr = queue->q.begin(); itr != queue->q.end(); ++itr) {
                    if (is_row_hit(itr)) {
                        auto begin2 = itr->addr_vec.begin();
                        vector<int> rowgroup2(begin2, begin2 + int(T::Level::Row) + 1);
                        if(rowgroup == rowgroup2)
                            num_row_hits++;
                    }
                }
            }

            assert(num_row_hits > 0); // The current request should be a hit, 
            // so there should be at least one request
            // that hits in the current open row
            if(num_row_hits == 1) {
                if(cmd == T::Command::RD)
                    cmd = T::Command::RDA;
                else if (cmd == T::Command::WR)
                    cmd = T::Command::WRA;
                else
                    assert(false && "Unimplemented command type.");
            }
        }

    }

    void issue_cmd(typename T::Command cmd, const vector<int>& addr_vec)
    {
        cmd_issue_autoprecharge(cmd, addr_vec);
        assert(is_ready(cmd, addr_vec));
        channel->update(cmd, addr_vec.data(), clk);

        if(cmd == T::Command::PRE){
            if(rowtable->get_hits(addr_vec, true) == 0){
                useless_activates++;
            }
        }
 
        rowtable->update(cmd, addr_vec, clk);
        if (record_cmd_trace){
            // select rank
            auto& file = cmd_trace_files[addr_vec[1]];
            string& cmd_name = channel->spec->command_name[int(cmd)];
            file<<clk<<','<<cmd_name;
            // TODO bad coding here
            if (cmd_name == "PREA" || cmd_name == "REF")
                file<<endl;
            else{
                int bank_id = addr_vec[int(T::Level::Bank)];
                if (channel->spec->standard_name == "DDR4" || channel->spec->standard_name == "GDDR5")
                    bank_id += addr_vec[int(T::Level::Bank) - 1] * channel->spec->org_entry.count[int(T::Level::Bank)];
                file<<','<<bank_id<<endl;
            }
        }
        if (print_cmd_trace){
            printf("%5s %10ld:", channel->spec->command_name[int(cmd)].c_str(), clk);
            for (int lev = 0; lev < int(T::Level::MAX); lev++)
                printf(" %5d", addr_vec[lev]);
            printf("\n");
        }
    }
    vector<int> get_addr_vec(typename T::Command cmd, list<Request>::iterator req){
        return req->addr_vec;
    }
};

template <>
vector<int> Controller<SALP>::get_addr_vec(
    SALP::Command cmd, list<Request>::iterator req);

template <>
bool Controller<SALP>::is_ready(list<Request>::iterator req);

template <>
void Controller<ALDRAM>::update_temp(ALDRAM::Temp current_temperature);

template <>
void Controller<TLDRAM>::tick();

template <>
void Controller<TLDRAM>::cmd_issue_autoprecharge(typename TLDRAM::Command& cmd,
                                                    const vector<int>& addr_vec);

} /*namespace ramulator*/

#endif /*__CONTROLLER_H*/
