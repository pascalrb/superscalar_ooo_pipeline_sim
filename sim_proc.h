#ifndef SIM_PROC_H
#define SIM_PROC_H

#include <stdint.h>
#include <vector>
#include <queue>
#include <deque>

using namespace std;

#define ISA_REG_NUM (67) // r0-r66

typedef struct proc_params{
    unsigned long int rob_size;
    unsigned long int iq_size;
    unsigned long int width;
}proc_params;

typedef struct{
    bool        valid;
    uint32_t    rob_tag;
}rmt_t;

typedef struct {
    //uint32_t    pc;                               // we don't model data
    uint32_t    op_type;
    uint32_t    seq_no;                             // used to map instructions
    int32_t     dest, src1, src2;                   // maps to the rob_tag 
    int32_t     dest_orig, src1_orig, src2_orig;    // r0-r66
    bool        dest_ready, src1_ready, src2_ready;
    uint32_t    FE_sc;  // start cycle
    uint32_t    FE_tc;  // total cycle
    uint32_t    DE_sc, DE_tc, RN_sc, RN_tc, RR_sc, RR_tc, DI_sc, DI_tc;
    uint32_t    IS_sc, IS_tc, EX_sc, EX_tc, WB_sc, WB_tc, RT_sc, RT_tc;
    uint8_t     ex_lat; 

    int32_t     unlock_by1;  // debug info
    int32_t     unlock_by2;  // debug info
} instruction_t;


typedef struct {
    /*
      index of array of rob_e 
      indicates rob number
    */
    //uint32_t value;                     // not used; we don't model data
    //uint32_t    pc;                     // data is not modeled in this simulator
    uint32_t        seq_no;               // used to map instruction
    int32_t         dest;                 // mapped to rX register
    bool            dest_ready;           // has that instr executed yet?
    instruction_t   instr;                // instr that rob entry is mapped to
} rob_entry_t;


// Superscalar Out-Of-Order Pipeline Simulator
class SuperscalarOOOPipeline{
    private:
        int32_t rob_elem_cnt;    // debug info
        int32_t rob_ready_cnt;    // debug info
        uint32_t instr_count;
        uint32_t cycles_count;
        bool pipeline_empty;
        bool trace_depleted;

        uint32_t rob_size;
        uint32_t iq_size;
        uint32_t width;

        uint32_t rob_head;
        uint32_t rob_tail;
        vector<rob_entry_t> rob_table;
        vector<rmt_t> rmt_table;

        queue<instruction_t> DE_q;
        queue<instruction_t> RN_q;
        deque<instruction_t> RR_q;
        deque<instruction_t> DI_q;
        vector<instruction_t> issue_q_v;
        vector<instruction_t> EX_List_v;
        queue<instruction_t> WB_List_q;

        void fetch(FILE* fp);
        void decode();
        void rename();
        void reg_read();
        void dispatch();
        void issue();
        void execute();
        void writeback();
        void retire();
        bool advance_cycle();
        bool enough_space_in_ROB(uint32_t RN_q_size);
        void ex_wakeup_dependent_instrs(int rob_tag, int unlocker);
        void print_dyn_instr(instruction_t instr);

    public:

        //Constructor
        SuperscalarOOOPipeline(uint32_t rob_size, uint32_t issue_q_size, uint32_t num_of_function_units);
        //Desctructor
        //~SuperscalarOOOPipeline();

        void process_trace_file(FILE * fp);
        void print_sim_result();

};

SuperscalarOOOPipeline::SuperscalarOOOPipeline(uint32_t rs, uint32_t iqs, uint32_t w){
    rob_size = rs;
    iq_size = iqs;
    width = w;
    rob_head = 0;
    rob_tail = 0;

    instr_count = 0;
    cycles_count = 0;
    rob_elem_cnt = 0;
    rob_ready_cnt = 0;

    pipeline_empty = false;
    trace_depleted = false;

    //init rmt 
    rmt_t rmt_e;
    for(int i = 0; i<ISA_REG_NUM; i++){
        rmt_e.valid = false;                // rob tag is initially a don't-care
        rmt_table.push_back(rmt_e); 
    }

    //init rob
    rob_entry_t rob_e;
    for(uint32_t i = 0; i<rob_size; i++){
        rob_e.dest_ready = false;           /* .seq_no = 0, .dest=0, */
        rob_table.push_back(rob_e);
    }
}

//SuperscalarOOOPipeline::~SuperscalarOOOPipeline(){
//    
//}


void SuperscalarOOOPipeline::process_trace_file(FILE *fp){

    do {
        retire();
        writeback();
        execute();
        issue();
        dispatch();
        reg_read();
        rename();
        decode();
        fetch(fp);

    } while (advance_cycle());
}

bool SuperscalarOOOPipeline::advance_cycle(){

    //printf("advance_cycle - cycle: %d\n ----------------------------------------------------------------------------- \n", cycles_count);

    cycles_count++;

    if(trace_depleted && pipeline_empty){
        return false;
    }else{
        return true;
    }
}


void SuperscalarOOOPipeline::fetch(FILE *fp){

    //printf("fetch0 - trace_depltd: %d, DE_q_size: %lu\n", trace_depleted, DE_q.size());

    if(trace_depleted || !DE_q.empty()) {return;}

    //printf("fetch1 - iCount: %d, cycle: %d\n", instr_count, cycles_count);

    int op_type, dest, src1, src2;  // Variables are read from trace file
    uint32_t pc;                    // Variable holds the pc read from input file

    int instr_fetched = 0;
    instruction_t instr;

    while(instr_fetched < width){

        if (fscanf(fp, "%x %d %d %d %d", &pc, &op_type, &dest, &src1, &src2) != EOF){

            instr.seq_no        = instr_count;
            instr.op_type       = op_type;
            instr.dest_orig     = dest;
            instr.src1_orig     = src1;
            instr.src2_orig     = src2;
            instr.src1          = src1;
            instr.src2          = src2;
            instr.dest_ready    = false;
            instr.src1_ready    = false;
            instr.src2_ready    = false;
            instr.FE_sc         = cycles_count;
            instr.FE_tc         = 1;
            instr.DE_sc         = cycles_count+1;
            instr.EX_tc         = 1;
            if(op_type == 0)
                instr.ex_lat = 1;
            else if (op_type == 1)
                instr.ex_lat =  2;
            else if (op_type == 2)
                instr.ex_lat =  5;
            else{
                printf("Unsupported Instruction Type [%d]", op_type);
                exit(EXIT_FAILURE);
            }

            instr.unlock_by1 = -1;
            instr.unlock_by2 = -1;

            // advance instr to DE_q
            DE_q.push(instr);

            instr_count++;

        }else{
            trace_depleted = true;
            return;
        }

        instr_fetched++;
    }

}

void SuperscalarOOOPipeline::decode(){
    //printf("decode0 - DE_q.size: %lu, RN_q.size: %lu\n", DE_q.size(), RN_q.size());

    if(DE_q.empty() || !RN_q.empty()){ return;}

    instruction_t instr;

    //printf("decode1 - seq_no @ rob head: %d\n", rob_table[rob_head].instr.seq_no);

    //printf("decode2 - seq_no: ");
    while(!DE_q.empty()){
        instr = DE_q.front();
        instr.DE_tc = (cycles_count+1) - instr.DE_sc;
        instr.RN_sc = cycles_count+1;

        //printf("%d | ", instr.seq_no);
        RN_q.push(instr);
        DE_q.pop();
    }
    //printf("\n");
}

bool SuperscalarOOOPipeline::enough_space_in_ROB(uint32_t RN_bundle_size){
    if((rob_size - rob_elem_cnt) >= RN_bundle_size){
        return true;
    }else{
        return false;
    }
}

void SuperscalarOOOPipeline::rename(){

    //printf("rename0 - RN_q_size: %lu, rob_elem_cnt: %d\n", RN_q.size(), rob_elem_cnt);

    if(RN_q.empty() || !RR_q.empty() || !enough_space_in_ROB(RN_q.size())) {return;}

   // printf("rename1 - \n");//rob_head: %d, rob_tail: %d\n", rob_head, rob_tail);

    instruction_t instr;

    //printf("rename1 - seq_no: ");
    while(!RN_q.empty()){
        instr = RN_q.front();
        instr.RN_tc = (cycles_count+1) - instr.RN_sc;
        instr.RR_sc = cycles_count+1;

        //printf("%d | ", instr.seq_no);

        // Allocate a ROB entry for instruction 
        rob_table[rob_tail].seq_no = instr.seq_no;
        rob_table[rob_tail].dest = instr.dest_orig;   // robX = dest rX
        rob_table[rob_tail].dest_ready = false;

        // Rename source regs from RMT (if it has them (not -1))
        if(instr.src1_orig != -1){
            if(rmt_table[instr.src1_orig].valid){
                instr.src1 = rmt_table[instr.src1_orig].rob_tag; // rename to rob that it depends on

                if(rob_table[instr.src1].dest_ready){
                    instr.src1_ready = true;
                }
            }else{
                instr.src1_ready = true;                    // gets value from ARF (really should happen in RR stage but easier to update here)
            } 
        }else{
            instr.src1_ready = true;                    
        }
        if(instr.src2_orig != -1){
            if(rmt_table[instr.src2_orig].valid){
                instr.src2 = rmt_table[instr.src2_orig].rob_tag; 

                if(rob_table[instr.src2].dest_ready){
                    instr.src2_ready = true;
                }
            }else{
                instr.src2_ready = true;                    
            } 
        }else{
            instr.src2_ready = true;                    
        }

        // Rename dest regs into RMT (if it has one (not -1))
        if(instr.dest_orig != -1){
            rmt_table[instr.dest_orig].rob_tag = rob_tail;
            rmt_table[instr.dest_orig].valid = true;
        }
        instr.dest = rob_tail; 

        // Update rob tail for next instruction
        rob_tail = (rob_tail == rob_size-1) ? 0 : rob_tail + 1;
        rob_elem_cnt++;

        RR_q.push_back(instr);
        RN_q.pop();
    }
    //printf("\n");

}

void SuperscalarOOOPipeline::reg_read(){
    //printf("reg_read0 - RR_q.size: %lu, DI_q.size: %lu\n", RR_q.size(), DI_q.size());
    if(RR_q.empty() || !DI_q.empty() ) {return;}


    instruction_t instr;

    //printf("reg_read1 - seq_no: ");
    while(!RR_q.empty()){
        instr = RR_q.front();
        instr.RR_tc = (cycles_count+1) - instr.RR_sc;
        instr.DI_sc = cycles_count+1;

        if(rob_table[instr.src1].dest_ready){
            instr.src1_ready = true;            // "reading" reg value from rob if ready
        }
        if(rob_table[instr.src2].dest_ready){
            instr.src2_ready = true;
        }

        //printf("%d | ", instr.seq_no);

        DI_q.push_back(instr);
        RR_q.pop_front();
    }
    //printf("\n");

}

void SuperscalarOOOPipeline::dispatch(){
    //printf("dispatch0 - DI_q.size: %lu, IQ_size: %lu\n", DI_q.size(), issue_q_v.size());
    if(DI_q.empty() || (DI_q.size() > (iq_size - issue_q_v.size()))) {return;}

    instruction_t instr;

    //printf("dispatch1 - seq_no: ");
    while(!DI_q.empty()){
        instr = DI_q.front();
        instr.DI_tc = (cycles_count+1) - instr.DI_sc;
        instr.IS_sc = cycles_count+1;

        //printf("%d | ", instr.seq_no);
        issue_q_v.push_back(instr); 
        DI_q.pop_front();
    }
    //printf("\n");

}

void SuperscalarOOOPipeline::issue(){

    //printf("issue - IQ_size: %lu\n", issue_q_v.size());

    uint32_t issued_instr = 0;

    vector<int> ready_elems_idx;
    int dltd = 0;

    for(int i = 0; i < issue_q_v.size(); i++){

        if(issued_instr == width) {break;}

        if(issue_q_v[i].src1_ready && issue_q_v[i].src2_ready){
            //printf("  issue - READY -instr: %d, instr_src1_unlocker: %d, instr_src2_unlocker: %d\n", issue_q_v[i].seq_no, issue_q_v[i].unlock_by1, issue_q_v[i].unlock_by2);

            ready_elems_idx.push_back(i-dltd);  // indexes shift left with each removal
            dltd++;

            issue_q_v[i].IS_tc = (cycles_count+1) - issue_q_v[i].IS_sc;
            issue_q_v[i].EX_sc = cycles_count+1;

            EX_List_v.push_back(issue_q_v[i]);
            //it = issue_q_v.erase(it);

            issued_instr++;
        }else{
            //printf("  issue - NOT READY - instr: %d, instr_src1_unlocker: %d, instr_src2_unlocker: %d\n", issue_q_v[i].seq_no, issue_q_v[i].unlock_by1, issue_q_v[i].unlock_by2);
        }
    }
    for(int i = 0; i < ready_elems_idx.size(); i++){
        issue_q_v.erase(issue_q_v.begin() + ready_elems_idx[i]);
    }
}

/*
    Wake up instructions in IQ, DI, and RR that are dependent on instruction
    completing execute stage.
*/
void SuperscalarOOOPipeline::ex_wakeup_dependent_instrs(int rob_tag, int unlocker){

    //printf(" execute_bypass - rob_tag: %d, seq_no: %d\n", rob_tag, unlocker);

    for(uint32_t i = 0; i < issue_q_v.size(); i++){
        if(!issue_q_v[i].src1_ready && issue_q_v[i].src1 == rob_tag){
            issue_q_v[i].src1_ready = true;
            issue_q_v[i].unlock_by1 = unlocker;             // debug info
        }
        if(!issue_q_v[i].src2_ready && issue_q_v[i].src2 == rob_tag){
            issue_q_v[i].src2_ready = true;
            issue_q_v[i].unlock_by2 = unlocker;
        }
    }

    for(uint32_t i = 0; i < DI_q.size(); i++){
        if(!DI_q[i].src1_ready && DI_q[i].src1 == rob_tag){
            DI_q[i].src1_ready = true;
            DI_q[i].unlock_by1 = unlocker;
        }
        if(!DI_q[i].src2_ready && DI_q[i].src2 == rob_tag){
            DI_q[i].src2_ready = true;
            DI_q[i].unlock_by2 = unlocker;
        }
    }

    for(uint32_t i = 0; i < RR_q.size(); i++){
        if(!RR_q[i].src1_ready && RR_q[i].src1 == rob_tag){
            RR_q[i].src1_ready = true;
            RR_q[i].unlock_by1 = unlocker;
        }
        if(!RR_q[i].src2_ready && RR_q[i].src2 == rob_tag){
            RR_q[i].src2_ready = true;
            RR_q[i].unlock_by2 = unlocker;
        }
    }

}

void SuperscalarOOOPipeline::execute(){
    
    //printf("execute - EX_List_size: %lu\n", EX_List_v.size());

    vector<int> ready_elems_idx;
    int dltd = 0;

    for(int i = 0; i < EX_List_v.size(); i++){
        if(EX_List_v[i].EX_tc == EX_List_v[i].ex_lat){

            //printf("  ex - pushing to WB - seq_no: %d, (*it)_EX_tc: %d\n", EX_List_v[i].seq_no, EX_List_v[i].EX_tc);
        
            // wake up dependent instructions in IQ, DI, and RR
            EX_List_v[i].dest_ready = true;
            ex_wakeup_dependent_instrs(EX_List_v[i].dest, EX_List_v[i].seq_no);

            ready_elems_idx.push_back(i-dltd);
            dltd++;

            EX_List_v[i].WB_sc = cycles_count+1;
            WB_List_q.push(EX_List_v[i]);
            //it = EX_List_v.erase(it);
        }else{
            //printf("  ex - still executing seq_no: %d, (*it)_EX_tc: %d\n", EX_List_v[i].seq_no, EX_List_v[i].EX_tc);
            EX_List_v[i].EX_tc++;
            //it++;
        }
    }
    for(int i = 0; i < ready_elems_idx.size(); i++){
        EX_List_v.erase(EX_List_v.begin() + ready_elems_idx[i]);
    }

}

void SuperscalarOOOPipeline::writeback(){

    //printf("writeback - WB_List_q.size: %lu\n", WB_List_q.size());
    
    instruction_t instr;

    while(!WB_List_q.empty()){
        instr = WB_List_q.front();
        instr.WB_tc = (cycles_count+1) - instr.WB_sc;
        instr.RT_sc = cycles_count+1;

        rob_table[instr.dest].dest_ready = true;
        rob_ready_cnt++;
        rob_table[instr.dest].instr = instr;
        WB_List_q.pop();
    }
}

void SuperscalarOOOPipeline::retire(){

    //printf("retire - ready_cnt: %d, rob_cnt: %d, rob_head: %d, rob_tail: %d\n", rob_ready_cnt, rob_elem_cnt, rob_head, rob_tail);

    if(pipeline_empty) {return;}

    uint32_t retired_instr = 0;

    while(rob_elem_cnt !=0){

        if(retired_instr == width) {
            //printf("RETIRE_MAX_BUNDLE\n");
            break;
        }

        if(rob_table[rob_head].dest_ready){
            rob_table[rob_head].instr.RT_tc = (cycles_count+1) - rob_table[rob_head].instr.RT_sc;
            print_dyn_instr(rob_table[rob_head].instr);

            //update RMT
            if(rob_table[rob_head].dest != -1){
                if(rmt_table[rob_table[rob_head].dest].rob_tag == rob_head)
                    rmt_table[rob_table[rob_head].dest].valid = false;
            }

            // invalidate rob entry
            rob_elem_cnt--;
            rob_ready_cnt--;

            rob_head = (rob_head == rob_size-1) ? 0 : rob_head + 1;
            retired_instr++;
        }else{
            break;
        }

    }

    if (rob_head == rob_tail && trace_depleted){
        pipeline_empty = true;
    }
}


/*
    Called by retire() after it "retires the instruction into the ARF"
*/
void SuperscalarOOOPipeline::print_dyn_instr(instruction_t instr){
    printf("%d fu{%d} src{%d,%d} dst{%d} ", instr.seq_no, instr.op_type, instr.src1_orig, instr.src2_orig, instr.dest_orig);
    printf("FE{%d,%d} DE{%d,%d} RN{%d,%d} RR{%d,%d} ", instr.FE_sc, instr.FE_tc, instr.DE_sc, instr.DE_tc, instr.RN_sc, instr.RN_tc, instr.RR_sc, instr.RR_tc);
    printf("DI{%d,%d} IS{%d,%d} EX{%d,%d} WB{%d,%d} RT{%d,%d}\n", instr.DI_sc, instr.DI_tc, instr.IS_sc, instr.IS_tc, instr.EX_sc, instr.EX_tc, instr.WB_sc, instr.WB_tc, instr.RT_sc, instr.RT_tc);
}

void SuperscalarOOOPipeline::print_sim_result(){
    
    printf("# === Simulation Results ========\n");
    printf("# Dynamic Instruction Count    = %d\n", instr_count);
    printf("# Cycles                       = %d\n", cycles_count);
    printf("# Instructions Per Cycle (IPC) = %.2lf\n", ( (double) instr_count / (double) cycles_count ) );
}

#endif // SIM_PROC_H
