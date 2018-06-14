#include "DDR3.h"
#include "DDR3.h"
#include "DRAM.h"
#include <vector>
#include <functional>
#include <cassert>

using namespace std;
using namespace ramulator;

string DDR3::standard_name = "DDR3";

map<string, enum DDR3::Org> DDR3::org_map = {
        {"DDR3_512Mb_x4",  DDR3::Org::DDR3_512Mb_x4},
        {"DDR3_512Mb_x8",  DDR3::Org::DDR3_512Mb_x8},
        {"DDR3_512Mb_x16", DDR3::Org::DDR3_512Mb_x16},
        {"DDR3_1Gb_x4",    DDR3::Org::DDR3_1Gb_x4},
        {"DDR3_1Gb_x8",    DDR3::Org::DDR3_1Gb_x8},
        {"DDR3_1Gb_x16",   DDR3::Org::DDR3_1Gb_x16},
        {"DDR3_2Gb_x4",    DDR3::Org::DDR3_2Gb_x4},
        {"DDR3_2Gb_x8",    DDR3::Org::DDR3_2Gb_x8},
        {"DDR3_2Gb_x16",   DDR3::Org::DDR3_2Gb_x16},
        {"DDR3_4Gb_x4",    DDR3::Org::DDR3_4Gb_x4},
        {"DDR3_4Gb_x8",    DDR3::Org::DDR3_4Gb_x8},
        {"DDR3_4Gb_x16",   DDR3::Org::DDR3_4Gb_x16},
        {"DDR3_8Gb_x4",    DDR3::Org::DDR3_8Gb_x4},
        {"DDR3_8Gb_x8",    DDR3::Org::DDR3_8Gb_x8},
        {"DDR3_8Gb_x16",   DDR3::Org::DDR3_8Gb_x16},
};

map<string, enum DDR3::Speed> DDR3::speed_map = {
        {"DDR3_800D",  DDR3::Speed::DDR3_800D},
        {"DDR3_800E",  DDR3::Speed::DDR3_800E},
        {"DDR3_1066E", DDR3::Speed::DDR3_1066E},
        {"DDR3_1066F", DDR3::Speed::DDR3_1066F},
        {"DDR3_1066G", DDR3::Speed::DDR3_1066G},
        {"DDR3_1333G", DDR3::Speed::DDR3_1333G},
        {"DDR3_1333H", DDR3::Speed::DDR3_1333H},
        {"DDR3_1600H", DDR3::Speed::DDR3_1600H},
        {"DDR3_1600J", DDR3::Speed::DDR3_1600J},
        {"DDR3_1600K", DDR3::Speed::DDR3_1600K},
        {"DDR3_1866K", DDR3::Speed::DDR3_1866K},
        {"DDR3_1866L", DDR3::Speed::DDR3_1866L},
        {"DDR3_2133L", DDR3::Speed::DDR3_2133L},
        {"DDR3_2133M", DDR3::Speed::DDR3_2133M},
};


DDR3::DDR3(Org org, Speed speed) :
        org_entry(org_table[int(org)]),
        speed_entry(speed_table[int(speed)]),
        read_latency(speed_entry.nCL + speed_entry.nBL) {
    init_speed();
    init_prereq();
    init_rowhit(); // SAUGATA: added row hit function
    init_rowopen();
    init_lambda();
    init_timing();

    /*powerdown_pending = new bool[4];
    powerup_pending = new bool[4];
    for(int y=0;y<4;y++)powerup_pending[y]=false;
    for(int y=0;y<4;y++)powerdown_pending[y]=false;
    cout<<"\n powerup_pending size ="<<org_entry.count[int(Level::Rank)];
    cout<<"\n Powerup_pending initial contents : ";
    for(int y=0;y<4;y++)if(!this->powerup_pending[y])cout<<"false";cout<<"\n";
    cout<<"\n powerdown_pending size ="<<org_entry.count[int(Level::Rank)];
    cout<<"\n Powerdown_pending initial contents : ";
    for(int y=0;y<4;y++)cout<<powerdown_pending[y]<<" ";cout<<"\n";
    cout << powerdown_pending << " " << powerup_pending << std::endl;*/
}

DDR3::DDR3(const string &org_str, const string &speed_str) :
        DDR3(org_map[org_str], speed_map[speed_str]) {
    /*powerdown_pending = new bool[4];
    powerup_pending = new bool[4];
    for(int y=0;y<4;y++)powerup_pending[y]=false;
    for(int y=0;y<4;y++)powerdown_pending[y]=false;
    cout<<"\n powerup_pending size ="<<org_entry.count[int(Level::Rank)];
    cout<<"\n Powerup_pending initial contents : ";
    for(int y=0;y<4;y++)cout<<powerup_pending[y]<<" ";cout<<"\n";
    cout<<"\n powerdown_pending size ="<<org_entry.count[int(Level::Rank)];
    cout<<"\n Powerdown_pending initial contents : ";
    for(int y=0;y<4;y++)cout<<powerdown_pending[y]<<" ";cout<<"\n";*/
}

void DDR3::set_channel_number(int channel) {
    org_entry.count[int(Level::Channel)] = channel;
}

void DDR3::set_rank_number(int rank) {
    org_entry.count[int(Level::Rank)] = rank;
}

void DDR3::init_speed() {
    // nRRD, nFAW
    int page = (org_entry.dq * org_entry.count[int(Level::Column)]) >> 13;
    switch (speed_entry.rate) {
        case 800:
            speed_entry.nRRD = (page == 1) ? 4 : 4;
            speed_entry.nFAW = (page == 1) ? 16 : 20;
            break;
        case 1066:
            speed_entry.nRRD = (page == 1) ? 4 : 6;
            speed_entry.nFAW = (page == 1) ? 20 : 27;
            break;
        case 1333:
            speed_entry.nRRD = (page == 1) ? 4 : 5;
            speed_entry.nFAW = (page == 1) ? 20 : 30;
            break;
        case 1600:
            speed_entry.nRRD = (page == 1) ? 5 : 6;
            speed_entry.nFAW = (page == 1) ? 24 : 32;
            break;
        case 1866:
            speed_entry.nRRD = (page == 1) ? 5 : 6;
            speed_entry.nFAW = (page == 1) ? 26 : 33;
            break;
        case 2133:
            speed_entry.nRRD = (page == 1) ? 5 : 6;
            speed_entry.nFAW = (page == 1) ? 27 : 34;
            break;
        default:
            assert(false);
    }

    // nRFC, nXS
    int chip = org_entry.size;
    switch (speed_entry.rate) {
        case 800:
            speed_entry.nRFC = (chip == 512) ? 36 : (chip == 1 << 10) ? 44 : (chip == 1 << 11) ? 64 : (chip == 1 << 12) ? 104 : 140;
            break;
        case 1066:
            speed_entry.nRFC = (chip == 512) ? 48 : (chip == 1 << 10) ? 59 : (chip == 1 << 11) ? 86 : (chip == 1 << 12) ? 139 : 187;
            break;
        case 1333:
            speed_entry.nRFC = (chip == 512) ? 60 : (chip == 1 << 10) ? 74 : (chip == 1 << 11) ? 107 : (chip == 1 << 12) ? 174 : 234;
            break;
        case 1600:
            speed_entry.nRFC = (chip == 512) ? 72 : (chip == 1 << 10) ? 88 : (chip == 1 << 11) ? 128 : (chip == 1 << 12) ? 208 : 280;
            break;
        case 1866:
            speed_entry.nRFC = (chip == 512) ? 84 : (chip == 1 << 10) ? 103 : (chip == 1 << 11) ? 150 : (chip == 1 << 12) ? 243 : 327;
            break;
        case 2133:
            speed_entry.nRFC = (chip == 512) ? 96 : (chip == 1 << 10) ? 118 : (chip == 1 << 11) ? 171 : (chip == 1 << 12) ? 278 : 374;
            break;
        default:
            assert(false);
    }
    switch (speed_entry.rate) {
        case 800:
            speed_entry.nXS = (chip == 512) ? 40 : (chip == 1 << 10) ? 48 : (chip == 1 << 11) ? 68 : (chip == 1 << 12) ? 108 : 144;
            break;
        case 1066:
            speed_entry.nXS = (chip == 512) ? 54 : (chip == 1 << 10) ? 64 : (chip == 1 << 11) ? 91 : (chip == 1 << 12) ? 144 : 192;
            break;
        case 1333:
            speed_entry.nXS = (chip == 512) ? 67 : (chip == 1 << 10) ? 80 : (chip == 1 << 11) ? 114 : (chip == 1 << 12) ? 180 : 240;
            break;
        case 1600:
            speed_entry.nXS = (chip == 512) ? 80 : (chip == 1 << 10) ? 96 : (chip == 1 << 11) ? 136 : (chip == 1 << 12) ? 216 : 288;
            break;
        case 1866:
            speed_entry.nXS = (chip == 512) ? 94 : (chip == 1 << 10) ? 112 : (chip == 1 << 11) ? 159 : (chip == 1 << 12) ? 252 : 336;
            break;
        case 2133:
            speed_entry.nXS = (chip == 512) ? 107 : (chip == 1 << 10) ? 128 : (chip == 1 << 11) ? 182 : (chip == 1 << 12) ? 288 : 384;
            break;
        default:
            assert(false);
    }
}


void DDR3::init_prereq() {
    // RD

    prereq[int(Level::Rank)][int(Command::RD)] = [](DRAM<DDR3> *node, Command cmd, int id) {
        switch (int(node->state)) {
            case int(State::PowerUp):
                return Command::MAX;
            case int(State::SActPowerDown):
            case int(State::FActPowerDown):
            case int(State::SPrePowerDown):
            case int(State::FPrePowerDown):
                return Command::RD;
            case int(State::SelfRefresh):
                return Command::SRX;
            default:
                assert(false);
        }
    };
    prereq[int(Level::Bank)][int(Command::RD)] = [](DRAM<DDR3> *node, Command cmd, int id) {
        switch (int(node->state)) {
            case int(State::Closed):
                return Command::ACT;
            case int(State::Opened):
                if (node->row_state.find(id) != node->row_state.end())
                    return cmd;
                return Command::PRE;
            default:
                assert(false);
        }
    };

    // WR
    prereq[int(Level::Rank)][int(Command::WR)] = prereq[int(Level::Rank)][int(Command::RD)];
    prereq[int(Level::Bank)][int(Command::WR)] = prereq[int(Level::Bank)][int(Command::RD)];

    // REF
    prereq[int(Level::Rank)][int(Command::REF)] = [](DRAM<DDR3> *node, Command cmd, int id) {
        for (auto bank : node->children) {
            if (bank->state == State::Closed)
                continue;
            return Command::PREA;
        }
        return Command::REF;
    };

    // PDN_S_PRE
    prereq[int(Level::Rank)][int(Command::PDN_S_PRE)] = [](DRAM<DDR3> *node, Command cmd, int id) {
        switch (int(node->state)) {
            case int(State::PowerUp):
                // Send Precharge-all if any bank is open
                for (auto bank : node->children) {
                    if (bank->state == State::Opened) {
                        return Command::PREA;
                    }
                }
                // else pre-power-down
                return Command::PDN_S_PRE;
            case int(State::SActPowerDown):
            case int(State::FActPowerDown):
            case int(State::FPrePowerDown):
                assert(false && "Rank is already powered down");
            case int(State::SPrePowerDown):
                return Command::PDN_S_PRE;
            case int(State::SelfRefresh):
                assert(false && "Rank cannot be powered down when in self-refresh mode");
            default:
                assert(false);
        }
    };

    // PDN_F_PRE
    prereq[int(Level::Rank)][int(Command::PDN_F_PRE)] = [](DRAM<DDR3> *node, Command cmd, int id) {
        switch (int(node->state)) {
            case int(State::PowerUp):
                // Send Precharge-all if any bank is open
                for (auto bank : node->children) {
                    if (bank->state == State::Opened) {
                        return Command::PREA;
                    }
                }
                // else pre-power-down
                return Command::PDN_F_PRE;
            case int(State::SActPowerDown):
            case int(State::FActPowerDown):
            case int(State::SPrePowerDown):
                assert(false && "Rank is already powered down");
            case int(State::FPrePowerDown):
                return Command::PDN_F_PRE;
            case int(State::SelfRefresh):
                assert(false && "Rank cannot be powered down when in self-refresh mode");
            default:
                assert(false);
        }
    };

    // PDN_S_ACT
    prereq[int(Level::Rank)][int(Command::PDN_S_ACT)] = [](DRAM<DDR3> *node, Command cmd, int id) {
        switch (int(node->state)) {
            case int(State::PowerUp):
                return Command::PDN_S_ACT;
            case int(State::SActPowerDown):
                return Command::PDN_S_ACT;
            case int(State::FActPowerDown):
            case int(State::SPrePowerDown):
            case int(State::FPrePowerDown):
                assert(false && "Rank is already powered down");
            case int(State::SelfRefresh):
                assert(false && "Rank cannot be powered down when in self-refresh mode");
            default:
                assert(false);
        }
    };

    // PDN_F_ACT
    prereq[int(Level::Rank)][int(Command::PDN_F_ACT)] = [](DRAM<DDR3> *node, Command cmd, int id) {
        switch (int(node->state)) {
            case int(State::PowerUp):
                return Command::PDN_F_ACT;
            case int(State::FActPowerDown):
                return Command::PDN_F_ACT;
            case int(State::SActPowerDown):
            case int(State::SPrePowerDown):
            case int(State::FPrePowerDown):
                assert(false && "Rank is already powered down");
            case int(State::SelfRefresh):
                assert(false && "Rank cannot be powered down when in self-refresh mode");
            default:
                assert(false);
        }
    };

    // PUP_ACT
    prereq[int(Level::Rank)][int(Command::PUP_ACT)] = [](DRAM<DDR3> *node, Command cmd, int id) {
        switch (int(node->state)) {
            case int(State::PowerUp):
                assert(false && "Rank is powered up");
            case int(State::FActPowerDown):
            case int(State::SActPowerDown):
                return Command::PUP_ACT;
            case int(State::FPrePowerDown):
            case int(State::SPrePowerDown):
                return Command::PUP_PRE;
            case int(State::SelfRefresh):
                assert(false && "Cannot exit self-refresh mode with PUP_ACT");
            default:
                assert(false);
        }

    };

    // PUP_PRE

    prereq[int(Level::Rank)][int(Command::PUP_PRE)] = [](DRAM<DDR3> *node, Command cmd, int id) {

        switch (int(node->state)) {
            case int(State::PowerUp):
                assert(false && "Rank is powered up");
            case int(State::FActPowerDown):
            case int(State::SActPowerDown):
                return Command::PUP_ACT;
            case int(State::FPrePowerDown):
            case int(State::SPrePowerDown):
                return Command::PUP_PRE;
            case int(State::SelfRefresh):
                assert(false && "Cannot exit self-refresh mode with PUP_ACT");
            default:
                assert(false);
        }
    };

    // SR
    prereq[int(Level::Rank)][int(Command::SRE)] = [](DRAM<DDR3> *node, Command cmd, int id) {
        switch (int(node->state)) {
            case int(State::PowerUp):
                return Command::SRE;
            case int(State::SActPowerDown):
            case int(State::FActPowerDown):
                return Command::PUP_ACT;
            case int(State::SPrePowerDown):
            case int(State::FPrePowerDown):
                return Command::PUP_PRE;
            case int(State::SelfRefresh):
                return Command::SRE;
            default:
                assert(false);
        }
    };
}


// SAUGATA: added row hit check functions to see if the desired location is currently open
void DDR3::init_rowhit() {
    // RD
    rowhit[int(Level::Bank)][int(Command::RD)] = [](DRAM<DDR3> *node, Command cmd, int id) {
        switch (int(node->state)) {
            case int(State::Closed):
                return false;
            case int(State::Opened):
                if (node->row_state.find(id) != node->row_state.end())
                    return true;
                return false;
            default:
                assert(false);
        }
    };

    // WR
    rowhit[int(Level::Bank)][int(Command::WR)] = rowhit[int(Level::Bank)][int(Command::RD)];
}

void DDR3::init_rowopen() {
    // RD
    rowopen[int(Level::Bank)][int(Command::RD)] = [](DRAM<DDR3> *node, Command cmd, int id) {
        switch (int(node->state)) {
            case int(State::Closed):
                return false;
            case int(State::Opened):
                return true;
            default:
                assert(false);
        }
    };

    // WR
    rowopen[int(Level::Bank)][int(Command::WR)] = rowopen[int(Level::Bank)][int(Command::RD)];
}

void DDR3::init_lambda() {
    lambda[int(Level::Bank)][int(Command::ACT)] = [](DRAM<DDR3> *node, int id) {
        node->state = State::Opened;
        node->row_state[id] = State::Opened;
    };
    lambda[int(Level::Bank)][int(Command::PRE)] = [](DRAM<DDR3> *node, int id) {
        node->state = State::Closed;
        node->row_state.clear();
    };
    lambda[int(Level::Rank)][int(Command::PREA)] = [](DRAM<DDR3> *node, int id) {
        for (auto bank : node->children) {
            bank->state = State::Closed;
            bank->row_state.clear();
        }
        // Does the rank need to transition to pre-charge power-down mode?
        if (node->state == State::FActPowerDown || node->state == State::SActPowerDown) {
            // Yes, perform the state transition
            switch (int(node->state)) {
                case int(State::FActPowerDown):
                    node->state = State::FPrePowerDown;
                    break;
                case int(State::SActPowerDown):
                    node->state = State::SPrePowerDown;
                    break;
            }
        }
    };
    lambda[int(Level::Rank)][int(Command::REF)] = [](DRAM<DDR3> *node, int id) {};
    lambda[int(Level::Bank)][int(Command::RD)] = [](DRAM<DDR3> *node, int id) {};
    lambda[int(Level::Bank)][int(Command::WR)] = [](DRAM<DDR3> *node, int id) {};
    lambda[int(Level::Bank)][int(Command::RDA)] = [](DRAM<DDR3> *node, int id) {
        node->state = State::Closed;
        node->row_state.clear();
    };
    lambda[int(Level::Bank)][int(Command::WRA)] = [](DRAM<DDR3> *node, int id) {
        node->state = State::Closed;
        node->row_state.clear();
    };
    lambda[int(Level::Rank)][int(Command::PDN_F_PRE)] = [](DRAM<DDR3> *node, int id) {
        for (auto bank : node->children) {
            assert(bank->state == State::Closed && "Close all open banks before entering precharge powerdown.");
        }
        node->state = State::FPrePowerDown;
        //node->parent->spec->powerdown_pending[id]=false; // resets pending_powerdown when powerdown actually finishes
    };
    lambda[int(Level::Rank)][int(Command::PDN_S_PRE)] = [](DRAM<DDR3> *node, int id) {
        for (auto bank : node->children) {
            assert(bank->state == State::Closed && "Close all open banks entering calling precharge powerdown.");
        }
        node->state = State::SPrePowerDown;
        //node->parent->spec->powerdown_pending[id]=false; // resets pending_powerdown when powerdown actually finishes
    };
    lambda[int(Level::Rank)][int(Command::PDN_F_ACT)] = [](DRAM<DDR3> *node, int id) {
        node->state = State::FActPowerDown;
        //node->parent->spec->powerdown_pending[node->id]=false; // resets pending_powerdown when powerdown actually finishes
    };
    lambda[int(Level::Rank)][int(Command::PDN_S_ACT)] = [](DRAM<DDR3> *node, int id) {
        node->state = State::SActPowerDown;
        //node->parent->spec->powerdown_pending[id]=false; // resets pending_powerdown when powerdown actually finishes
    };
    lambda[int(Level::Rank)][int(Command::PUP_ACT)] = [](DRAM<DDR3> *node, int id) {
        node->state = State::PowerUp;
        //node->parent->spec->powerup_pending[node->id]=false; // resets pending_powerup when powerup actually finishes
    };
    lambda[int(Level::Rank)][int(Command::PUP_PRE)] = [](DRAM<DDR3> *node, int id) {
        node->state = State::PowerUp;
        //node->parent->spec->powerup_pending[id]=false; // resets pending_powerup when powerup actually finishes

    };
    lambda[int(Level::Rank)][int(Command::SRE)] = [](DRAM<DDR3> *node, int id) {
        node->state = State::SelfRefresh;
    };
    lambda[int(Level::Rank)][int(Command::SRX)] = [](DRAM<DDR3> *node, int id) {
        node->state = State::PowerUp;
    };
}


void DDR3::init_timing() {
    SpeedEntry &s = speed_entry;
    vector<TimingEntry> *t;

    /*** Channel ***/
    t = timing[int(Level::Channel)];

    // CAS <-> CAS
    t[int(Command::RD)].push_back({Command::RD, 1, s.nBL});
    t[int(Command::RD)].push_back({Command::RDA, 1, s.nBL});
    t[int(Command::RDA)].push_back({Command::RD, 1, s.nBL});
    t[int(Command::RDA)].push_back({Command::RDA, 1, s.nBL});
    t[int(Command::WR)].push_back({Command::WR, 1, s.nBL});
    t[int(Command::WR)].push_back({Command::WRA, 1, s.nBL});
    t[int(Command::WRA)].push_back({Command::WR, 1, s.nBL});
    t[int(Command::WRA)].push_back({Command::WRA, 1, s.nBL});


    /*** Rank ***/
    t = timing[int(Level::Rank)];

    // CAS <-> CAS
    t[int(Command::RD)].push_back({Command::RD, 1, s.nCCD});
    t[int(Command::RD)].push_back({Command::RDA, 1, s.nCCD});
    t[int(Command::RDA)].push_back({Command::RD, 1, s.nCCD});
    t[int(Command::RDA)].push_back({Command::RDA, 1, s.nCCD});
    t[int(Command::WR)].push_back({Command::WR, 1, s.nCCD});
    t[int(Command::WR)].push_back({Command::WRA, 1, s.nCCD});
    t[int(Command::WRA)].push_back({Command::WR, 1, s.nCCD});
    t[int(Command::WRA)].push_back({Command::WRA, 1, s.nCCD});
    t[int(Command::RD)].push_back({Command::WR, 1, s.nCL + s.nCCD + 2 - s.nCWL});
    t[int(Command::RD)].push_back({Command::WRA, 1, s.nCL + s.nCCD + 2 - s.nCWL});
    t[int(Command::RDA)].push_back({Command::WR, 1, s.nCL + s.nCCD + 2 - s.nCWL});
    t[int(Command::RDA)].push_back({Command::WRA, 1, s.nCL + s.nCCD + 2 - s.nCWL});
    t[int(Command::WR)].push_back({Command::RD, 1, s.nCWL + s.nBL + s.nWTR});
    t[int(Command::WR)].push_back({Command::RDA, 1, s.nCWL + s.nBL + s.nWTR});
    t[int(Command::WRA)].push_back({Command::RD, 1, s.nCWL + s.nBL + s.nWTR});
    t[int(Command::WRA)].push_back({Command::RDA, 1, s.nCWL + s.nBL + s.nWTR});

    // CAS <-> CAS (between sibling ranks)
    t[int(Command::RD)].push_back({Command::RD, 1, s.nBL + s.nRTRS, true});
    t[int(Command::RD)].push_back({Command::RDA, 1, s.nBL + s.nRTRS, true});
    t[int(Command::RDA)].push_back({Command::RD, 1, s.nBL + s.nRTRS, true});
    t[int(Command::RDA)].push_back({Command::RDA, 1, s.nBL + s.nRTRS, true});
    t[int(Command::RD)].push_back({Command::WR, 1, s.nBL + s.nRTRS, true});
    t[int(Command::RD)].push_back({Command::WRA, 1, s.nBL + s.nRTRS, true});
    t[int(Command::RDA)].push_back({Command::WR, 1, s.nBL + s.nRTRS, true});
    t[int(Command::RDA)].push_back({Command::WRA, 1, s.nBL + s.nRTRS, true});
    t[int(Command::RD)].push_back({Command::WR, 1, s.nCL + s.nBL + s.nRTRS - s.nCWL, true});
    t[int(Command::RD)].push_back({Command::WRA, 1, s.nCL + s.nBL + s.nRTRS - s.nCWL, true});
    t[int(Command::RDA)].push_back({Command::WR, 1, s.nCL + s.nBL + s.nRTRS - s.nCWL, true});
    t[int(Command::RDA)].push_back({Command::WRA, 1, s.nCL + s.nBL + s.nRTRS - s.nCWL, true});
    t[int(Command::WR)].push_back({Command::RD, 1, s.nCWL + s.nBL + s.nRTRS - s.nCL, true});
    t[int(Command::WR)].push_back({Command::RDA, 1, s.nCWL + s.nBL + s.nRTRS - s.nCL, true});
    t[int(Command::WRA)].push_back({Command::RD, 1, s.nCWL + s.nBL + s.nRTRS - s.nCL, true});
    t[int(Command::WRA)].push_back({Command::RDA, 1, s.nCWL + s.nBL + s.nRTRS - s.nCL, true});

    t[int(Command::RD)].push_back({Command::PREA, 1, s.nRTP});
    t[int(Command::WR)].push_back({Command::PREA, 1, s.nCWL + s.nBL + s.nWR});

    // CAS <-> PD
    t[int(Command::RD)].push_back({Command::PDN_F_ACT, 1, s.nCL + s.nBL + 1});
    t[int(Command::RD)].push_back({Command::PDN_S_ACT, 1, s.nCL + s.nBL + 1});
    t[int(Command::RD)].push_back({Command::PDN_F_PRE, 1, s.nCL + s.nBL + 1});
    t[int(Command::RD)].push_back({Command::PDN_S_PRE, 1, s.nCL + s.nBL + 1});

    t[int(Command::RDA)].push_back({Command::PDN_F_ACT, 1, s.nCL + s.nBL + 1});
    t[int(Command::RDA)].push_back({Command::PDN_S_ACT, 1, s.nCL + s.nBL + 1});
    t[int(Command::RDA)].push_back({Command::PDN_F_PRE, 1, s.nCL + s.nBL + 1});
    t[int(Command::RDA)].push_back({Command::PDN_S_PRE, 1, s.nCL + s.nBL + 1});

    t[int(Command::WR)].push_back({Command::PDN_F_ACT, 1, s.nCWL + s.nBL + s.nWR});
    t[int(Command::WR)].push_back({Command::PDN_S_ACT, 1, s.nCWL + s.nBL + s.nWR});
    t[int(Command::WR)].push_back({Command::PDN_F_PRE, 1, s.nCWL + s.nBL + s.nWR});
    t[int(Command::WR)].push_back({Command::PDN_S_PRE, 1, s.nCWL + s.nBL + s.nWR});

    t[int(Command::WRA)].push_back({Command::PDN_F_ACT, 1, s.nCWL + s.nBL + s.nWR + 1}); // +1 for pre
    t[int(Command::WRA)].push_back({Command::PDN_S_ACT, 1, s.nCWL + s.nBL + s.nWR + 1}); // +1 for pre
    t[int(Command::WRA)].push_back({Command::PDN_F_PRE, 1, s.nCWL + s.nBL + s.nWR + 1}); // +1 for pre
    t[int(Command::WRA)].push_back({Command::PDN_S_PRE, 1, s.nCWL + s.nBL + s.nWR + 1}); // +1 for pre

    t[int(Command::PUP_ACT)].push_back({Command::RD, 1, s.nXP});
    t[int(Command::PUP_ACT)].push_back({Command::RDA, 1, s.nXP});
    t[int(Command::PUP_ACT)].push_back({Command::WR, 1, s.nXP});
    t[int(Command::PUP_ACT)].push_back({Command::WRA, 1, s.nXP});

    t[int(Command::PUP_PRE)].push_back({Command::RD, 1, s.nXP});
    t[int(Command::PUP_PRE)].push_back({Command::RDA, 1, s.nXP});
    t[int(Command::PUP_PRE)].push_back({Command::WR, 1, s.nXP});
    t[int(Command::PUP_PRE)].push_back({Command::WRA, 1, s.nXP});

    // CAS <-> SR: none (all banks have to be precharged)

    // RAS <-> RAS
    t[int(Command::ACT)].push_back({Command::ACT, 1, s.nRRD});
    t[int(Command::ACT)].push_back({Command::ACT, 4, s.nFAW});
    t[int(Command::ACT)].push_back({Command::PREA, 1, s.nRAS});
    t[int(Command::PREA)].push_back({Command::ACT, 1, s.nRP});

    // RAS <-> REF
    t[int(Command::PRE)].push_back({Command::REF, 1, s.nRP});
    t[int(Command::PREA)].push_back({Command::REF, 1, s.nRP});
    t[int(Command::REF)].push_back({Command::ACT, 1, s.nRFC});

    // RAS <-> PD
    t[int(Command::ACT)].push_back({Command::PDN_F_ACT, 1, 1});
    t[int(Command::ACT)].push_back({Command::PDN_S_ACT, 1, 1});
    t[int(Command::ACT)].push_back({Command::PDN_F_PRE, 1, 1});
    t[int(Command::ACT)].push_back({Command::PDN_S_PRE, 1, 1});

    t[int(Command::PUP_ACT)].push_back({Command::ACT, 1, s.nXP});
    t[int(Command::PUP_ACT)].push_back({Command::PRE, 1, s.nXP});
    t[int(Command::PUP_ACT)].push_back({Command::PREA, 1, s.nXP});

    t[int(Command::PUP_PRE)].push_back({Command::ACT, 1, s.nXP});
    t[int(Command::PUP_PRE)].push_back({Command::PRE, 1, s.nXP});
    t[int(Command::PUP_PRE)].push_back({Command::PREA, 1, s.nXP});

    // RAS <-> SR
    t[int(Command::PRE)].push_back({Command::SRE, 1, s.nRP});
    t[int(Command::PREA)].push_back({Command::SRE, 1, s.nRP});
    t[int(Command::SRX)].push_back({Command::ACT, 1, s.nXS});

    // REF <-> REF
    t[int(Command::REF)].push_back({Command::REF, 1, s.nRFC});

    // REF <-> PD
    t[int(Command::REF)].push_back({Command::PDN_F_ACT, 1, 1});
    t[int(Command::REF)].push_back({Command::PDN_S_ACT, 1, 1});
    t[int(Command::REF)].push_back({Command::PDN_F_PRE, 1, 1});
    t[int(Command::REF)].push_back({Command::PDN_S_PRE, 1, 1});

    t[int(Command::PUP_ACT)].push_back({Command::REF, 1, s.nXP});
    t[int(Command::PUP_PRE)].push_back({Command::REF, 1, s.nXP});

    // REF <-> SR
    t[int(Command::SRX)].push_back({Command::REF, 1, s.nXS});

    // PD <-> PD
    t[int(Command::PDN_F_ACT)].push_back({Command::PUP_ACT, 1, s.nPD});
    t[int(Command::PDN_S_ACT)].push_back({Command::PUP_ACT, 1, s.nPD});
    t[int(Command::PDN_F_PRE)].push_back({Command::PUP_ACT, 1, s.nPD});
    t[int(Command::PDN_S_PRE)].push_back({Command::PUP_ACT, 1, s.nPD});

    t[int(Command::PDN_F_ACT)].push_back({Command::PUP_PRE, 1, s.nPD});
    t[int(Command::PDN_S_ACT)].push_back({Command::PUP_PRE, 1, s.nPD});
    t[int(Command::PDN_F_PRE)].push_back({Command::PUP_PRE, 1, s.nPD});
    t[int(Command::PDN_S_PRE)].push_back({Command::PUP_PRE, 1, s.nPD});

    t[int(Command::PUP_ACT)].push_back({Command::PDN_F_ACT, 1, s.nXP});
    t[int(Command::PUP_ACT)].push_back({Command::PDN_S_ACT, 1, s.nXP});

    t[int(Command::PUP_PRE)].push_back({Command::PDN_F_PRE, 1, s.nXP});
    t[int(Command::PUP_PRE)].push_back({Command::PDN_S_PRE, 1, s.nXP});

    // PD <-> SR
    t[int(Command::PUP_ACT)].push_back({Command::SRE, 1, s.nXP});
    t[int(Command::PUP_PRE)].push_back({Command::SRE, 1, s.nXP});

    t[int(Command::SRX)].push_back({Command::PDN_F_ACT, 1, s.nXS});
    t[int(Command::SRX)].push_back({Command::PDN_S_ACT, 1, s.nXS});
    t[int(Command::SRX)].push_back({Command::PDN_F_PRE, 1, s.nXS});
    t[int(Command::SRX)].push_back({Command::PDN_S_PRE, 1, s.nXS});

    // SR <-> SR
    t[int(Command::SRE)].push_back({Command::SRX, 1, s.nCKESR});
    t[int(Command::SRX)].push_back({Command::SRE, 1, s.nXS});


    /*** Bank ***/
    t = timing[int(Level::Bank)];

    // CAS <-> RAS
    t[int(Command::ACT)].push_back({Command::RD, 1, s.nRCD});
    t[int(Command::ACT)].push_back({Command::RDA, 1, s.nRCD});
    t[int(Command::ACT)].push_back({Command::WR, 1, s.nRCD});
    t[int(Command::ACT)].push_back({Command::WRA, 1, s.nRCD});

    t[int(Command::RD)].push_back({Command::PRE, 1, s.nRTP});
    t[int(Command::WR)].push_back({Command::PRE, 1, s.nCWL + s.nBL + s.nWR});

    t[int(Command::RDA)].push_back({Command::ACT, 1, s.nRTP + s.nRP});
    t[int(Command::WRA)].push_back({Command::ACT, 1, s.nCWL + s.nBL + s.nWR + s.nRP});

    // RAS <-> RAS
    t[int(Command::ACT)].push_back({Command::ACT, 1, s.nRC});
    t[int(Command::ACT)].push_back({Command::PRE, 1, s.nRAS});
    t[int(Command::PRE)].push_back({Command::ACT, 1, s.nRP});
}
